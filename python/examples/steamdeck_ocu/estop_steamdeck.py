from __future__ import print_function

import textwrap
import time

from xbox_controller.xbox_joystick_factory import XboxJoystickFactory

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.util
from bosdyn.api import estop_pb2
from bosdyn.client.estop import EstopClient


# hostnames = [('10.0.0.32', 'Autumn'), ('10.0.0.29', 'Chara')]
# hostnames = [('10.0.0.27', 'Betty'), ('10.0.0.63','Fausto')]
hostnames = [('10.0.0.63','Fausto')]
# hostnames = [('10.0.0.38', 'Donner'), ('10.0.0.34','Xander')]

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

class EStopControlUnit:
    """XboxController class provides mapping between xbox controller commands and Spot API calls.

    Attributes:
        client_name: Common name of this program to use in the SDK calls.
        robot: Instance of the robot.
        estop_client: Client for the E-Stop functionality.
        estop_keepalive: E-Stop keep-alive object.
        estop_buttons_pressed: Boolean used to determine when E-Stop button combination is released
                               in order to toggle the E-Stop
        motors_powered: Boolean whether the robot motors are powered.
    """

    def __init__(self):
        self.client_name = "Steam Deck EStop"
        self.robots = []
        self.estop_clients = []
        self.estop_keepalives = []
        self.estop_buttons_pressed = False

        self.irobot = 0

        # Robot state
        self.motors_powered = []

        self.pad_onpress = False
        self.pad_pressed = False

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

            self.estop_clients.append(self.robots[ii].ensure_client(EstopClient.default_service_name))
            self.estop_keepalives.append(None)
            self.motors_powered.append(None)
            print(hostnames[ii][1], ' acquired')
        # Print controls
        print(
            textwrap.dedent("""\
| Button Combination | Functionality            |
|--------------------|--------------------------|
| LB + RB + B        | E-Stop all robot(s)      |
| Back               | Exit                     |
        """))

        # Describe the necessary steps before one can command the robot.
        print("Before you can command the robot: \n" + \
            "\t1. Acquire a software E-Stop (Left Button + Right Button + B). \n")

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
            sys.exit('E-Stop')

    def _print_status(self):
        """Prints the current status of the robot: Name, E-Stop, Control, Powered-on, Current Mode.
        """

        # Move cursor back to the start of the line
        print(chr(13), end="")
        
        print("Robots: " + ''.join([str(hostname[1])+" " for hostname in hostnames]) , end="")
        
        if self.estop_keepalives[self.irobot]:
            print("\tAll E-Stops: Acquired    ", end="")
        else:
            print("\tAll E-Stops: Not Acquired", end="")

        
        if self.robots[self.irobot].is_powered_on():
            print("\tAll Motors: Powered On ", end="")
        
    def control_robots(self, frequency=100):
        """Controls robot from an Xbox controller.

        Mapping
        Button Combination    -> Functionality
        --------------------------------------
        LB + RB + B           -> E-Stop
        Back                  -> Exit

        Args:
            frequency: Max frequency to send commands to robot
        """

        joy = XboxJoystickFactory.get_joystick()

        while not joy.back():
            start_time = time.time()

            # Handle button combinations first
            # If E-Stop button combination is pressed, toggle E-Stop functionality only when
            # buttons are released.

            if joy.left_bumper() and joy.right_bumper() and joy.B():
                self.estop_buttons_pressed = True
            else:
                if self.estop_buttons_pressed:
                    self._toggle_estop()
                    self.estop_buttons_pressed = False

            self._print_status()

            # Make sure we maintain a max frequency of sending commands to the robot
            end_time = time.time()
            delta = 1 / frequency - (end_time - start_time)
            if delta > 0.0:
                time.sleep(delta)

        joy.close()

def main(argv):
    controller = EStopControlUnit()
    controller.initialize_robots()
    controller.control_robots(frequency = 100)

if __name__ == "__main__":
    import sys
    main(sys.argv[1:])