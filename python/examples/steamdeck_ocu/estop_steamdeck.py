from __future__ import print_function

import argparse
import math
import textwrap
import time
from enum import Enum

import signal
import atexit

from xbox_controller.xbox_joystick_factory import XboxJoystickFactory

import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.util
from bosdyn.api import estop_pb2
# from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.lease import LeaseClient, ResourceAlreadyClaimedError
# from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
# from bosdyn.geometry import EulerZXY

VELOCITY_BASE_SPEED = 2.0 # m/s
VELOCITY_BASE_ANGULAR = 1.5  # rad/sec
VELOCITY_CMD_DURATION = 0.25  # seconds
HEIGHT_MAX = 1.0  # m
ROLL_OFFSET_MAX = 0.4  # rad
YAW_OFFSET_MAX = 0.7805  # rad
PITCH_OFFSET_MAX = 0.7805  # rad
HEIGHT_CHANGE = 0.025  # m per command

# from stitch_front_images import stitch_front_images


# hostnames = [('10.0.0.32', 'Autumn'), ('10.0.0.29', 'Chara')]
# hostnames = [('10.0.0.27', 'Betty'), ('10.0.0.63','Fausto')]
# hostnames = [('10.0.0.27', 'Betty')]
hostnames = [('10.0.0.38', 'Donner'), ('10.0.0.34','Xander')]

"""
spot_ip_suffixes["Autumn"]=32
spot_ip_suffixes["Betty"]=27
spot_ip_suffixes["Chara"]=29
spot_ip_suffixes["Donner"]=38
spot_ip_suffixes["Eddie"]=62
spot_ip_suffixes["Fausto"]=63
spot_ip_suffixes["Gus"]=-1 # TODO IP
spot_ip_suffixes["Harriet"]=39
spot_ip_suffixes["Ishmael"]=52
spot_ip_suffixes["Jasper"]=60
spot_ip_suffixes["Kepler"]=61
spot_ip_suffixes["Xander"]=34
"""

class OperatorControlUnit:
    """XboxController class provides mapping between xbox controller commands and Spot API calls.

    Attributes:
        client_name: Common name of this program to use in the SDK calls.
        robot: Instance of the robot.
        command_client: Client for all the robot commands.
        lease_client: Client for the lease management.
        estop_client: Client for the E-Stop functionality.
        estop_keepalive: E-Stop keep-alive object.
        estop_buttons_pressed: Boolean used to determine when E-Stop button combination is released
                               in order to toggle the E-Stop
        mobility_params: Mobility parameters to use in each robot command.
        mode: Current robot movement type as RobotMode enum.
        has_robot_control: Boolean whether program has acquired robot control.
        motors_powered: Boolean whether the robot motors are powered.
        body_height: Current robot body height in meters from normal standing height.
        stand_yaw: Current robot body yaw in radians.
        stand_roll: Current robot body roll in radians.
        stand_pitch: Current robot body pitch in radians.
        stand_height_change: Boolean whether the height is changed in stand mode.
        stand_roll_change: Boolean whether the robot body roll is changed in stand mode.
        stand_pitch_change: Boolean whether the robot body pitch is changed in stand mode.
        stand_yaw_change:Boolean whether the robot body yaw is changed in stand mode.
    """

    def __init__(self):
        self.client_name = "OCU_Client"
        self.robots = []
        self.command_clients = []
        self.lease_clients = []
        self.lease_keep_alives = []
        self.estop_clients = []
        self.estop_keepalives = []
        self.estop_buttons_pressed = False
        self.mobility_params = []

        self.irobot = 0

        # Robot state
        self.mode = None
        self.modes = []
        self.has_robot_controls = []
        self.motors_powered = []

        self.pad_onpress = False
        self.pad_pressed = False

        self.body_height = 0.0
        self.stand_yaw = 0.0
        self.stand_roll = 0.0
        self.stand_pitch = 0.0

        self.stand_height_change = False
        self.stand_roll_change = False
        self.stand_pitch_change = False
        self.stand_yaw_change = False

    def initialize_robots(self):
        """Initializes SDK from command line arguments.

        Args:
            config: Command-line arguments as argparse object.
        """

        sdk = bosdyn.client.create_standard_sdk(self.client_name)

        for ii in range(len(hostnames)):
            print('trying', hostnames[ii][1])
            self.robots.append(sdk.create_robot(hostnames[ii][0], name=hostnames[ii][1]))
            bosdyn.client.util.authenticate(self.robots[ii])
            self.robots[ii].time_sync.wait_for_sync()

            # self.command_clients.append(self.robots[ii].ensure_client(RobotCommandClient.default_service_name))
            self.lease_clients.append(self.robots[ii].ensure_client(LeaseClient.default_service_name))
            self.estop_clients.append(self.robots[ii].ensure_client(EstopClient.default_service_name))
            # self.mobility_params.append(spot_command_pb2.MobilityParams(locomotion_hint=spot_command_pb2.HINT_AUTO))
            self.lease_keep_alives.append(None)
            self.estop_keepalives.append(None)
            self.modes.append(None)
            self.has_robot_controls.append(None)
            self.motors_powered.append(None)
            print(hostnames[ii][1], ' acquired')
        # Print controls
        print(
            textwrap.dedent("""\
| Button Combination | Functionality            |
|--------------------|--------------------------|
| A                  | Walk                     |
| B                  | Stand                    |
| X                  | Sit                      |
| Y                  | Stairs                   |
| D-Pad up           | Select all robots        |
| D-Pad left/right   | Toggle individual robot  |
| LB + :             |                          |
| - D-pad up/down    | Walk height              |
| - D-pad left       | Battery-Change Pose      |
| - D-pad right      | Self right               |
| - Y                | Jog                      |
| - A                | Amble                    |
| - B                | Crawl                    |
| - X                | Hop                      |
|                    |                          |
| If Stand Mode      |                          |
| - Left Stick       |                          |
| -- X               | Rotate body in roll axis |
| -- Y               | Control height           |
| - Right Stick      |                          |
| -- X               | Turn body in yaw axis    |
| -- Y               | Turn body in pitch axis  |
| Else               |                          |
| - Left Stick       | Move                     |
| - Right Stick      | Turn                     |
|                    |                          |
| LB + RB + B        | E-Stop selected robot(s) |
| Start              | Motor power & Control    |
| Back               | Exit                     |
        """))

        # Describe the necessary steps before one can command the robot.
        print("Before you can command the robot: \n" + \
            "\t1. Acquire a software E-Stop (Left Button + Right Button + B). \n" + \
            "\t2. Obtain a lease and power on the robot's motors (Start button).")

    def _shutdown(self):
        """Returns lease to power off.
        """
        if self.lease_keep_alives[self.irobot]:
            self.lease_keep_alives[self.irobot].shutdown()

    def _toggle_estop(self):
        """Toggles on/off E-Stop.
        """
        is_estop = False

        for ii in range(len(self.robots)):
            if self.estop_keepalives[ii]==None:
                if self.estop_clients[ii].get_status().stop_level == estop_pb2.ESTOP_LEVEL_NONE:
                    print("Taking E-Stop from another controller")

                #register endpoint with 9 second timeout
                estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=self.estop_clients[ii],
                                                                name=self.client_name,
                                                                estop_timeout=9.0)
                estop_endpoint.force_simple_setup()

                self.estop_keepalives[ii] = bosdyn.client.estop.EstopKeepAlive(estop_endpoint)
            else:
                self.estop_keepalives[ii].stop()
                self.estop_keepalives[ii].shutdown()
                self.estop_keepalives[ii] = None
                is_estop = True
        if is_estop:
            self._shutdown()
            sys.exit('E-Stop')

    def _gain_control(self):
        """Acquires lease of the robot to gain control.
        """

        for ii in range(len(self.robots)):
            if self.has_robot_controls[ii] or not self.estop_keepalives[ii]:
                return
            try:
                self.lease_clients[ii].acquire()
            except ResourceAlreadyClaimedError as exc:
                print("Another controller " + exc.response.lease_owner.client_name +
                    " has a lease. Close that controller"
                    ", wait a few seconds and press the Start button again.")
                return
            else:
                self.lease_keep_alives[ii] = bosdyn.client.lease.LeaseKeepAlive(
                    self.lease_clients[ii], return_at_exit=True)
                self.has_robot_controls[ii] = True

    def _power_motors(self):
        """Powers the motors on in the robot.
        """
        for ii in range(len(self.robots)):
            if self.motors_powered[ii] or \
            not self.has_robot_controls[ii] or \
            not self.estop_keepalives[ii] or \
            self.robots[ii].is_powered_on():
                return

            self.robots[ii].power_on(timeout_sec=20)
            self.robots[ii].is_powered_on()
            self.motors_powered[ii] = True

    def _issue_robot_command(self, command, endtime=None):
        """Check that the lease has been acquired and motors are powered on before issuing a command.

        Args:
            command: RobotCommand message to be sent to the robot.
            endtime: Time (in the local clock) that the robot command should stop.
        """
        if self.irobot >=0:
            if not self.has_robot_controls[self.irobot]:
                print("Must have control by acquiring a lease before commanding the robot.")
                return
            if not self.motors_powered[self.irobot]:
                print("Must have motors powered on before commanding the robot.")
                return
            
            self.command_clients[self.irobot].robot_command_async(command, end_time_secs=endtime)
        else:
            for ii in range(len(self.robots)):
                if not self.has_robot_controls[ii]:
                    print("Must have control by acquiring a lease before commanding the robot.")
                    return
                if not self.motors_powered[ii]:
                    print("Must have motors powered on before commanding the robot.")
                    return
                
                self.command_clients[ii].robot_command_async(command, end_time_secs=endtime)

    def _print_status(self):
        """Prints the current status of the robot: Name, E-Stop, Control, Powered-on, Current Mode.
        """

        # Move cursor back to the start of the line
        print(chr(13))
        
        if self.irobot == -1:
            print("Robot: All!   " , end="")
        else:
            print("Robot: " + str(hostnames[self.irobot][1])+" " , end="")
        if self.estop_keepalives[self.irobot]:
            print("\tAll E-Stops: Acquired    ", end="")
        else:
            print("\tAll E-Stops: Not Acquired", end="")
        if self.has_robot_controls[self.irobot]:
            print("\tLeases: Acquired    ", end="")
        if self.robots[self.irobot].is_powered_on():
            print("\tAll Motors: Powered On ", end="")
        if self.mode:
            print("\t" + "Mode: " + self.mode.name, end="")
            num_chars = len(self.mode.name)
            if num_chars < 6:  # 6 is the length of Stairs enum
                print(" " * (6 - num_chars), end="")
        
    def control_robots(self, frequency=100):
        """Controls robot from an Xbox controller.

        Mapping
        Button Combination    -> Functionality
        --------------------------------------
        LB + RB + B           -> E-Stop
        A                     -> Walk
        B                     -> Stand
        X                     -> Sit
        Y                     -> Stairs
        D-Pad                 -> cameras
        Back                  -> Exit
        LB +
          D-pad up/down       -> walk height
          D-pad left          -> Battery-Change Pose (roll over)
          D-pad right         -> Self right
          Y                   -> Jog
          A                   -> Amble
          B                   -> Crawl
          X                   -> Hop
        if stand
          Left Stick
            X                 -> rotate body in roll axis
            Y                 -> Control height
          Right Stick
            X                 -> Turn body in yaw axis
            Y                 -> Turn body in pitch axis
        else
          Left Stick          -> Move
          Right Stick         -> Turn
        Start                 -> Motor power and Control

        Args:
            frequency: Max frequency to send commands to robot
        """

        try:
            joy = XboxJoystickFactory.get_joystick()

            while not joy.back():
                start_time = time.time()

                left_x = joy.left_x()
                left_y = joy.left_y()
                right_x = joy.right_x()
                right_y = joy.right_y()

                #Detect the transition between no press and press
                if (joy.dpad_up() or joy.dpad_left() or joy.dpad_right()):
                    if not self.pad_pressed and not self.pad_onpress:
                        self.pad_pressed = True 
                        self.pad_onpress = True #This will only be true for one "frame" and is reset
                    else:
                        self.pad_pressed = True
                        self.pad_onpress = False
                else:
                    self.pad_pressed = False
                    self.pad_onpress = False

                #switch robot
                if joy.dpad_up() and self.pad_onpress:
                    # Select all robots
                    self.irobot = -1
                    # print("All robots selected!")
                elif joy.dpad_right() and self.pad_onpress:
                    self.irobot += 1
                    if self.irobot > len(hostnames)-1:
                        self.irobot = 0
                    # print(hostnames[self.irobot][1], " selected!")
                elif joy.dpad_left() and self.pad_onpress:
                    self.irobot -= 1
                    if self.irobot < 0:
                        self.irobot = len(hostnames)-1
                    # print(hostnames[self.irobot][1], " selected!")

                # Handle button combinations first
                # If E-Stop button combination is pressed, toggle E-Stop functionality only when
                # buttons are released.

                if joy.left_bumper() and joy.right_bumper() and joy.B():
                    self.estop_buttons_pressed = True
                else:
                    if self.estop_buttons_pressed:
                        self._toggle_estop()
                        self.estop_buttons_pressed = False

                if joy.left_bumper() and joy.dpad_up():
                    self._change_height(1)
                if joy.left_bumper() and joy.dpad_down():
                    self._change_height(-1)
                if joy.left_bumper() and joy.dpad_left() and self.pad_onpress:
                    self._battery_change_pose()
                if joy.left_bumper() and joy.dpad_right() and self.pad_onpress:
                    self._selfright()

                if joy.Y():
                    if joy.left_bumper():
                        self._jog()
                    else:
                        self._stairs()
                if joy.A():
                    if joy.left_bumper():
                        self._amble()
                    else:
                        self._walk()
                if joy.B() and not joy.right_bumper():
                    if joy.left_bumper():
                        self._crawl()
                    else:
                        self._stand()
                if joy.X():
                    if joy.left_bumper():
                        self._hop()
                    else:
                        self._sit()
                
                if joy.start():
                    ""
                    self._gain_control()
                    self._power_motors()

                self._print_status()

                # Make sure we maintain a max frequency of sending commands to the robot
                end_time = time.time()
                delta = 1 / frequency - (end_time - start_time)
                if delta > 0.0:
                    time.sleep(delta)

            joy.close()
        finally:
            # Close out when done
            self._shutdown()

def main(argv):

    controller = OperatorControlUnit()
    controller.initialize_robots()
    controller.control_robots(frequency = 100)

if __name__ == "__main__":
    import sys
    main(sys.argv[1:])