
from __future__ import print_function

import argparse
import math
import textwrap
import time
from enum import Enum

from xbox_joystick_factory import XboxJoystickFactory


def control_robot(frequency):
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
            l_t = joy.left_trigger()      

            print(joy.reading) 

            # Make sure we maintain a max frequency of sending commands to the robot
            end_time = time.time()
            delta = 1 / frequency - (end_time - start_time)
            if delta > 0.0:
                time.sleep(delta)

        joy.close()
    finally:
        # Close out when done
        print('close')



def main(argv):
    """Parses command line args.

    Args:
        argv: List of command-line arguments.
    """

    max_frequency = 100

    control_robot(max_frequency)


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])