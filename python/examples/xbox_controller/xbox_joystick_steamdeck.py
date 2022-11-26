# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

""" Xbox 360 controller Python support for Linux OSes.

Based on:
https://github.com/FRC4564/Xbox/blob/master/xbox.py

You'll need to first install xboxdrv:
    sudo apt-get install xboxdrv

See http://pingus.seul.org/~grumbel/xboxdrv/ for details on xboxdrv
"""

import select
import subprocess
import time

from xbox_joystick import XboxJoystick

# Need to parse this: https://git.launchpad.net/ubuntu/+source/joystick/tree/utils/jstest.c#n184
"""
012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
Axes:  0:     0  1:     0  2:     0  3:     0  4:     0  5:     0  6:     0  7:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 
"""

class XboxJoystickSteamdeck(XboxJoystick):
    """Initializes the joystick/wireless receiver.

    Launches 'xboxdrv' as a subprocess and checks that the wired joystick or wireless receiver is
    attached. Calling any of the Joystick methods will cause a refresh to occur, if refreshTime
    has elapsed. Routinely call a Joystick method, at least once per second, to avoid overfilling
    the event buffer.

    Attributes:
        proc: Subprocess running the xboxdrv Xbox driver.
        pipe: Pipe to the xboxdrv subprocess where to read data from.
        reading: Latest read buffer from the xbox controller.
        refresh_time: Absolute time when next refresh (read results from xboxdrv stdout pipe) is
            to occur.
        refresh_delay: Joystick refresh rate.
        connect_status: Attribute derived from base class. Boolean that stores connection status,
            set to True once controller is detected and stays on.
    """

    def __init__(self, refresh_rate=100):
        """ Init method.

        Args:
            refresh_rate: Determines the maximum rate at which events are polled from xboxdrv.
        """

        super().__init__()
        try:
            self.proc = subprocess.Popen(['jstest', '/dev/input/js0'],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT) #, bufsize=0
        except FileNotFoundError as e:
            raise Exception(
                'Error opening SteamDeck Controller.  Have you started the daemon with "scc-daemon start"? It is available at https://github.com/kozec/sc-controller.'
            ) from e


        self.pipe = self.proc.stdout
        self.reading = '0' * 173  #initialize stick readings to all zeros
        self.refresh_time = 0
        self.refresh_delay = 1.0 / refresh_rate
        for ii in range(4):
            self.pipe.readline()
        for ii in range(19):
            self.pipe.read(173)

        # Read responses from 'xboxdrv' for up to 3 seconds, looking for controller to respond
        found = False
        wait_time = time.time() + 10
        while wait_time > time.time() and not found:
            readable, writeable, exception = select.select([self.pipe], [], [], 0)
            if readable:
                response = self.pipe.read(173)
                # print(response)
                # Hard fail if we see this, so force an error
                # if response[0:7] == b'No Xbox':
                #     raise IOError('No Xbox controller/receiver found')
                # Success if we see the following
                # if response[0:12].lower() == b'press ctrl-c':
                #     found = True
                # If we see 140 char line, we are seeing valid input
                if len(response) == 173:
                    found = True
                    self.connect_status = True
                    self.reading = response
        # if the controller wasn't found, then halt
        if not found:
            self.close()
            raise IOError('Unable to detect Xbox controller/receiver - Run python as sudo')
        
        # while 1:
        #     readable, writeable, exception = select.select([self.pipe], [], [], 0)
        #     print(bool(readable), self.pipe.readable(), self.pipe.read(173))
    

    def refresh(self):
        """Used by all Joystick methods to read the most recent events from xboxdrv.

        If a valid event response is found, then the controller is flagged as 'connected'.
        """

        # Refresh the joystick readings based on regular defined frequency
        if self.refresh_time < time.time():
            self.refresh_time = time.time() + self.refresh_delay  # Set next refresh time
            # If there is text available to read from xboxdrv, then read it.
            # readable, writeable, exception = select.select([self.pipe], [], [], 0)
            readable = self.pipe.readable()
            # print(bool(readable))

            # readable is garbage. Need a better way to get the last available line.
            if readable:
                # Read every line that is available. We only need to decode the last one.
                while readable:
                    response = self.pipe.read(173)
                    # A zero length response means controller has been unplugged.
                    if len(response) == 0:
                        raise IOError('Xbox controller disconnected')
                    readable, writeable, exception = select.select([self.pipe], [], [], 0)
                if len(response) == 173:
                    self.connect_status = True
                    self.reading = response
                else:  #Any other response means we have lost wireless or controller battery
                    self.connect_status = False
            # else:
            #     self.reading = '0' * 173  #initialize stick readings to all zeros

            # print("   ", end="")
            # for j in range(18):
            #     for i in range(10):
            #         print(i, end="")
            # print()
            # print(self.reading)
            # print(bool(readable))
            # print(int(self.reading[10:16]))
            # print(self.reading[106:109])
            # print(int(self.reading[30:36])>=0)

    def connected(self):
        self.refresh()
        return self.connect_status

    def left_x(self, deadzone=1000):
        self.refresh()
        raw = int(self.reading[10:16])
        return self.axis_scale(raw, deadzone)

    def left_y(self, deadzone=1000):
        self.refresh()
        raw = -int(self.reading[20:26])
        return self.axis_scale(raw, deadzone)

    def left_trigger(self, deadzone=1000):
        self.refresh()
        raw = int(self.reading[30:36])
        # print(self.axis_scale(raw, deadzone))
        # return int(self.reading[30:36])>=0
        # return int(self.reading[30:36]) / 32767.0
        return (self.axis_scale(raw, deadzone)+1)/2.0

    def right_x(self, deadzone=1000):
        self.refresh()
        raw = int(self.reading[40:46])
        return self.axis_scale(raw, deadzone)

    def right_y(self, deadzone=1000):
        self.refresh()
        raw = int(self.reading[50:56])
        return self.axis_scale(raw, deadzone)
    
    def right_trigger(self, deadzone=1000):
        self.refresh()
        raw = int(self.reading[60:66])
        return (self.axis_scale(raw, deadzone)+1)/2.0

    def axis_scale(self, raw, deadzone):
        if abs(raw) < deadzone:
            return 0.0

        if raw < 0:
            return (raw + deadzone) / (32767.0 - deadzone)
        else:
            return (raw - deadzone) / (32767.0 - deadzone)

    def dpad_up(self):
        self.refresh()
        return int(self.reading[80:86])<0

    def dpad_down(self):
        self.refresh()
        return int(self.reading[80:86])>0

    def dpad_left(self):
        self.refresh()
        return int(self.reading[70:76])<0

    def dpad_right(self):
        self.refresh()
        return int(self.reading[70:76])>0

    def A(self):
        self.refresh()
        return int(self.reading[99:102]==b'on ')

    def B(self):
        self.refresh()
        return int(self.reading[106:109]==b'on ')

    def X(self):
        self.refresh()
        return int(self.reading[113:116]==b'on ')

    def Y(self):
        self.refresh()
        return int(self.reading[120:123]==b'on ')

    def left_bumper(self):
        self.refresh()
        return int(self.reading[127:130]==b'on ')

    def right_bumper(self):
        self.refresh()
        return int(self.reading[134:137]==b'on ')

    def back(self):
        self.refresh()
        return int(self.reading[140:144]==b'on ')

    def start(self):
        self.refresh()
        return int(self.reading[148:151]==b'on ')

    def left_thumbstick(self):
        self.refresh()
        return int(self.reading[162:165]==b'on ')

    def right_thumbstick(self):
        self.refresh()
        return int(self.reading[169:172]==b'on ')

    def close(self):
        self.proc.kill()
