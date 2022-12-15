# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

""" Detect and follow fiducial tags. """
import logging
import math
import signal
import sys
import threading
import time
from sys import platform

import cv2
import numpy as np
from PIL import Image

import bosdyn.client
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api import geometry_pb2, image_pb2, trajectory_pb2, world_object_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, VISION_FRAME_NAME, get_a_tform_b,
                                         get_vision_tform_body)
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import LeaseClient
from bosdyn.client.math_helpers import Quat, SE3Pose, SE2Pose, angle_diff
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_id import RobotIdClient, version_tuple
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient

#pylint: disable=no-member
LOGGER = logging.getLogger()

# Use this length to make sure we're commanding the head of the robot
# to a position instead of the center.
BODY_LENGTH = 1.1

FIDUCIAL_NUMBER = 6
PITCH_OFFSET = np.radians(5)
DEFAULT_FOLLOW_DISTANCE = 1.0


class FollowFiducial(object):
    """ Detect and follow a fiducial with Spot."""

    def __init__(self, robot, options):
        # Robot instance variable.
        self._robot = robot
        self._robot_id = robot.ensure_client(RobotIdClient.default_service_name).get_id(timeout=0.4)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

        # Stopping Distance (x,y) offset from the tag and angle offset from desired angle.
        self._tag_offset = float(options.distance_margin) + BODY_LENGTH / 2.0  # meters

        # Maximum speeds.
        self._max_x_vel = 1.5
        self._max_y_vel = 1.5
        self._max_ang_vel = 1.5 #Must be < 1.6

        # Indicators for movement and image displays.
        self._standup = True  # Stand up the robot.
        self._movement_on = True  # Let the robot walk towards the fiducial.
        self._limit_speed = options.limit_speed  # Limit the robot's walking speed.
        self._avoid_obstacles = options.avoid_obstacles  # Disable obstacle avoidance.

        # Epsilon distance between robot and desired go-to point.
        self._x_eps = .01
        self._y_eps = .01
        self._angle_eps = .01

        # Indicator for if motor power is on.
        self._powered_on = False

        # Counter for the number of iterations completed.
        self._attempts = 0

        # Maximum amount of iterations without finding fiducial before powering off the motors.
        self._max_attempts = 100000

        # Maximum amount of iterations to keep tracking the current tag before looking for another
        self._max_attempts_before_new_fiducial = 50  # about 2.5 seconds

        # Tag we're currently tracking
        self._tag_id = None

        # Camera intrinsics for the current camera source being analyzed.
        self._intrinsics = None

        # Transform from the robot's camera frame to the baselink frame.
        # It is a math_helpers.SE3Pose.
        self._camera_tform_body = None

        # Transform from the robot's baselink to the world frame.
        # It is a math_helpers.SE3Pose.
        self._body_tform_world = None

        # Latest detected fiducial's position in the world.
        self._current_tag_world_pose = np.array([])

        # Heading angle based on the camera source which detected the fiducial.
        self._angle_desired = None
        self._pitch_desired = None

        # Dictionary mapping camera source to it's latest image taken.
        self._image = dict()

        # List of all possible camera sources.
        self._source_names = [
            src.name for src in self._image_client.list_image_sources() if
            (src.image_type == image_pb2.ImageSource.IMAGE_TYPE_VISUAL and "depth" not in src.name)
        ]
        print(self._source_names)

        # Dictionary mapping camera source to previously computed extrinsics.
        self._camera_to_extrinsics_guess = self.populate_source_dict()

        # Camera source which a bounding box was last detected in.
        self._previous_source = None

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_client.get_robot_state()

    @property
    def image(self):
        """Return the current image associated with each source name."""
        return self._image

    @property
    def image_sources_list(self):
        """Return the list of camera sources."""
        return self._source_names

    def populate_source_dict(self):
        """Fills dictionary of the most recently computed camera extrinsics with the camera source.
           The initial boolean indicates if the extrinsics guess should be used."""
        camera_to_extrinsics_guess = dict()
        for src in self._source_names:
            # Dictionary values: use_extrinsics_guess bool, (rotation vector, translation vector) tuple.
            camera_to_extrinsics_guess[src] = (False, (None, None))
        return camera_to_extrinsics_guess

    def check_if_version_has_world_objects(self, robot_id):
        """Check that software version contains world object service."""
        # World object service was released in spot-sdk version 1.2.0
        return version_tuple(robot_id.software_release.version) >= (1, 2, 0)

    def start(self):
        """Claim lease of robot and start the fiducial follower."""
        self._robot.time_sync.wait_for_sync()

        # Stand the robot up.
        if self._standup:
            self.power_on()
            blocking_stand(self._robot_command_client)

            # Delay grabbing image until spot is standing (or close enough to upright).
            time.sleep(.35)

        while self._attempts <= self._max_attempts:
            detected_fiducial = False
            fiducial_rt_world = None
            # Get the first fiducial object Spot detects with the world object service.
            fiducial = self.get_fiducial_objects()
            if fiducial is not None:
                vision_tform_fiducial = get_a_tform_b(
                    fiducial.transforms_snapshot, VISION_FRAME_NAME,
                    fiducial.apriltag_properties.frame_name_fiducial).to_proto()
                if vision_tform_fiducial is not None:
                    detected_fiducial = True
                    fiducial_rt_world = vision_tform_fiducial.position
            if detected_fiducial:
                # Go to the tag and stop within a certain distance
                self.go_to_tag(fiducial_rt_world, fiducial.transforms_snapshot)
                self._attempts = 0
            else:
                self._attempts += 1  #increment attempts at finding a fiducial
                if self._attempts > self._max_attempts_before_new_fiducial and self._tag_id is not None:
                    print("Giving up tracking fiducial", self._tag_id)
                    self._tag_id = None
            # time.sleep(0.001)

        # Power off at the conclusion of the example.
        if self._powered_on:
            self.power_off()

    def get_fiducial_objects(self):
        """Get all fiducials that Spot detects with its perception system."""
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(
            object_type=request_fiducials).world_objects
        for fiducial in fiducial_objects:
            curr_id = fiducial.apriltag_properties.tag_id
            if curr_id == self._tag_id or (self._tag_id is None and curr_id==FIDUCIAL_NUMBER and curr_id < 500): #-----------------------------------Added specific apriltag
                if self._tag_id is None:
                    print("Now tracking fiducial", curr_id)
                    self._tag_id = curr_id
                return fiducial
        # Return none if no fiducials are found.
        return None

    def power_on(self):
        """Power on the robot."""
        self._robot.power_on()
        self._powered_on = True
        print("Powered On " + str(self._robot.is_powered_on()))

    def power_off(self):
        """Power off the robot."""
        self._robot.power_off()
        print("Powered Off " + str(not self._robot.is_powered_on()))


    def go_to_tag(self, fiducial_rt_world, transforms_snapshot):
        """Use the position of the april tag in vision world frame and command the robot."""
        # Compute the go-to point (offset by .5m from the fiducial position) and the heading at
        # this point.
        new_tag_world_pose, new_angle_desired, new_pitch_desired = self.offset_tag_pose(
            fiducial_rt_world, self._tag_offset)

        # Check if the tag's point has changed.  If so send a new command.
        # The change in heading helps spot not lose the fiducial as it goes around corners.
        if (self._current_tag_world_pose.size != 0 and
            np.linalg.norm(self._current_tag_world_pose - new_tag_world_pose) <= self._x_eps and
            abs(angle_diff(new_angle_desired, self._angle_desired)) <= self._angle_eps):
            return

        self._current_tag_world_pose = new_tag_world_pose
        self._angle_desired = new_angle_desired
        self._pitch_desired = new_pitch_desired

        world_tform_desired = SE2Pose(self._current_tag_world_pose[0], self._current_tag_world_pose[1],
                                      self._angle_desired)
        robot_tform_world = get_a_tform_b(
            transforms_snapshot, BODY_FRAME_NAME,VISION_FRAME_NAME).get_closest_se2_transform()
        robot_tform_desired = robot_tform_world * world_tform_desired

        print("Going to position (in robot coords)", robot_tform_desired)
        #Command the robot to go to the tag in kinematic odometry framec
        mobility_params = self.set_mobility_params(new_pitch_desired)#----------------------------------------------------------Added pitch
        tag_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=self._current_tag_world_pose[0], goal_y=self._current_tag_world_pose[1],
            goal_heading=self._angle_desired, frame_name=VISION_FRAME_NAME, params=mobility_params,
            body_height=0.5, locomotion_hint=spot_command_pb2.HINT_AUTO)#----------------------------------------------------------Added height
        # TODO: make this velocity-based.

        end_time = 1.0
        if self._movement_on and self._powered_on:
            #Issue the command to the robot
            self._robot_command_client.robot_command(lease=None, command=tag_cmd,
                                                     end_time_secs=time.time() + end_time)
        return

    def final_state(self):
        """Check if the current robot state is within range of the fiducial position."""
        robot_state = get_vision_tform_body(self.robot_state.kinematic_state.transforms_snapshot)
        robot_angle = robot_state.rot.to_yaw()
        robot_pitch = robot_state.rot.to_pitch()
        if self._current_tag_world_pose.size != 0:
            x_dist = abs(self._current_tag_world_pose[0] - robot_state.x)
            y_dist = abs(self._current_tag_world_pose[1] - robot_state.y)
            yaw_angle = abs(self._angle_desired - robot_angle)
            pitch_angle = abs(self._pitch_desired - robot_pitch)
            if ((x_dist < self._x_eps) and (y_dist < self._y_eps) and (yaw_angle < self._angle_eps) and (pitch_angle < self._angle_eps)):
                return True
        return False

    def get_desired_angle(self, xhat):
        """Compute heading based on the vector from robot to object."""
        zhat = [0.0, 0.0, 1.0]
        yhat = np.cross(zhat, xhat)
        mat = np.array([xhat, yhat, zhat]).transpose()
        return Quat.from_matrix(mat).to_yaw()
    
    def get_desired_pitch(self,xhat): #-----------------------------------------------------Added angle to pitch to aim at apriltag
        """Compute heading based on the vector from robot to object."""
        # print("xhat",xhat)
        yhat = [0.0, 1.0, 0.0]
        zhat = np.cross(xhat, yhat)
        mat = np.array([xhat, yhat, zhat]).transpose()
        return Quat.from_matrix(mat).to_pitch() - PITCH_OFFSET

    def offset_tag_pose(self, object_rt_world, dist_margin=1.0):
        """Offset the go-to location of the fiducial and compute the desired heading."""
        robot_rt_world = get_vision_tform_body(self.robot_state.kinematic_state.transforms_snapshot)
        robot_to_object_ewrt_world = np.array(
            [object_rt_world.x - robot_rt_world.x, object_rt_world.y - robot_rt_world.y, object_rt_world.z-robot_rt_world.z]) #---------------------------------Added Z
        robot_to_object_ewrt_world_norm = robot_to_object_ewrt_world / np.linalg.norm(
            robot_to_object_ewrt_world)
        heading = self.get_desired_angle(robot_to_object_ewrt_world_norm)
        tilt = self.get_desired_pitch(robot_to_object_ewrt_world_norm)
        goto_rt_world = np.array([
            object_rt_world.x - robot_to_object_ewrt_world_norm[0] * dist_margin,
            object_rt_world.y - robot_to_object_ewrt_world_norm[1] * dist_margin
        ])
        return goto_rt_world, heading, tilt

    def set_mobility_params(self, desired_pitch):
        """Set robot mobility params to disable obstacle avoidance."""
        obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=False,
                                                    disable_vision_foot_obstacle_avoidance=False,
                                                    disable_vision_foot_constraint_avoidance=False,
                                                    obstacle_avoidance_padding=.001)
        body_control = self.set_default_body_control(pitch_in=desired_pitch)#-----------------------------------------------------Added angle to pitch to aim at apriltag
        if self._limit_speed:
            speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(
                linear=Vec2(x=self._max_x_vel, y=self._max_y_vel), angular=self._max_ang_vel))
            if not self._avoid_obstacles:
                mobility_params = spot_command_pb2.MobilityParams(
                    obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control,
                    locomotion_hint=spot_command_pb2.HINT_AUTO)
            else:
                mobility_params = spot_command_pb2.MobilityParams(
                    vel_limit=speed_limit, body_control=body_control,
                    locomotion_hint=spot_command_pb2.HINT_AUTO)
        elif not self._avoid_obstacles:
            mobility_params = spot_command_pb2.MobilityParams(
                obstacle_params=obstacles, body_control=body_control,
                locomotion_hint=spot_command_pb2.HINT_AUTO)
        else:
            #When set to none, RobotCommandBuilder populates with good default values
            mobility_params = None
        return mobility_params

    @staticmethod
    def set_default_body_control(pitch_in=np.radians(-25)):
        """Set default body control params to current body position"""
        footprint_R_body = geometry.EulerZXY(pitch=pitch_in)#-----------------------------------------------------Added angle to pitch to aim at apriltag.
        position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
        rotation = footprint_R_body.to_quaternion()
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)


class DisplayImagesAsync(object):
    """Display the images Spot sees from all five cameras."""

    def __init__(self, fiducial_follower):
        self._fiducial_follower = fiducial_follower
        self._thread = None
        self._started = False
        self._sources = []

    def get_image(self):
        """Retrieve current images (with bounding boxes) from the fiducial detector."""
        images = self._fiducial_follower.image
        image_by_source = []
        for s_name in self._sources:
            if s_name in images:
                image_by_source.append(images[s_name])
            else:
                image_by_source.append(np.array([]))
        return image_by_source

    def start(self):
        """Initialize the thread to display the images."""
        if self._started:
            return None
        self._sources = self._fiducial_follower.image_sources_list
        self._started = True
        self._thread = threading.Thread(target=self.update)
        self._thread.start()
        return self

    def update(self):
        """Update the images being displayed to match that seen by the robot."""
        while self._started:
            images = self.get_image()
            for i, image in enumerate(images):
                if image.size != 0:
                    original_height, original_width = image.shape[:2]
                    resized_image = cv2.resize(
                        image, (int(original_width * .5), int(original_height * .5)),
                        interpolation=cv2.INTER_NEAREST)
                    cv2.imshow(self._sources[i], resized_image)
                    cv2.moveWindow(self._sources[i],
                                   max(int(i * original_width * .5), int(i * original_height * .5)),
                                   0)
                    cv2.waitKey(1)

    def stop(self):
        """Stop the thread and the image displays."""
        self._started = False
        cv2.destroyAllWindows()


class Exit(object):
    """Handle exiting on SIGTERM."""

    def __init__(self):
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    def _sigterm_handler(self, _signum, _frame):
        self._kill_now = True

    @property
    def kill_now(self):
        """Return if sigterm received and program should end."""
        return self._kill_now


def main():
    """Command-line interface."""
    import argparse

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument("--distance-margin", default=DEFAULT_FOLLOW_DISTANCE,
                        help="Distance [meters] that the robot should stop from the fiducial.")
    parser.add_argument("--limit-speed", default=True, type=lambda x: (str(x).lower() == 'true'),
                        help="If the robot should limit its maximum speed.")
    parser.add_argument("--avoid-obstacles", default=False, type=lambda x:
                        (str(x).lower() == 'true'),
                        help="If the robot should have obstacle avoidance enabled.")
    parser.add_argument(
        "--use-world-objects", default=True, type=lambda x: (str(x).lower() == 'true'),
        help="If fiducials should be from the world object service or the apriltag library.")
    options = parser.parse_args()

    # If requested, attempt import of Apriltag library
    if not options.use_world_objects:
        try:
            global apriltag
            from apriltag import apriltag
        except ImportError as e:
            print("Could not import the AprilTag library. Aborting. Exception: ", str(e))
            return False

    # Create robot object.
    sdk = create_standard_sdk('FollowFiducialClient')
    robot = sdk.create_robot(options.hostname)

    fiducial_follower = None
    image_viewer = None
    try:
        with Exit():
            bosdyn.client.util.authenticate(robot)
            robot.start_time_sync()

            # Verify the robot is not estopped.
            assert not robot.is_estopped(), "Robot is estopped. " \
                                            "Please use an external E-Stop client, " \
                                            "such as the estop SDK example, to configure E-Stop."

            fiducial_follower = FollowFiducial(robot, options)
            #time.sleep(.1)
            lease_client = robot.ensure_client(LeaseClient.default_service_name)
            with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True,
                                                    return_at_exit=True):
                fiducial_follower.start()
    except RpcError as err:
        LOGGER.error("Failed to communicate with robot: %s", err)
    finally:
        if image_viewer is not None:
            image_viewer.stop()

    return False


if __name__ == "__main__":
    if not main():
        sys.exit(1)
