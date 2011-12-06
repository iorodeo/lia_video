#!/usr/bin/env python
"""
Copyright 2011  IO Rodeo Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
from __future__ import division
import sys
import os
import os.path
import platform
import time
import serial


DEBUG = False 

CHANNEL_COUNT = 4

SERIAL_BAUDRATE = 115200

SERIAL_COMMAND_GET_123 = 0
SERIAL_COMMAND_SET_COMPUTERCONTROL_MODE = 1
SERIAL_COMMAND_SET_STANDALONE_MODE = 2
SERIAL_COMMAND_SET_CURRENT_VALUES = 3
SERIAL_COMMAND_SET_CURRENT_VALUE = 4
SERIAL_COMMAND_SET_BNC_MODES = 5
SERIAL_COMMAND_SET_BNC_MODE = 6

def remap(x,in_min,in_max,out_min,out_max):
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min

class CurrentController(object):

    def __init__(self,port=''):
        self.current_min = 0
        self.current_max = 1000
        self.output_min = 0
        self.output_max = 1023
        # Set default com port
        self.osType = platform.system()
        self.port_list_linux = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3']
        self.port_list_windows = ['com1','com2','com3','com4']
        self.port_index = 0
        if port == '':
            if self.osType == 'Linux':
                self.port_list = self.port_list_linux
                self.port = self.port_list[self.port_index]
            else:
                self.port_list = self.port_list_windows
                self.port = self.port_list_windows[self.port_index]
        else:
            self.port = port
            self.port_list = [port]

        self.serial = serial.Serial(self.port,SERIAL_BAUDRATE,timeout=1)
        self.serial.open()
        time.sleep(2.0)
        self.test_serial_port()
        self.set_computercontrol_mode()
        self.set_current_values([self.current_min,self.current_min,self.current_min,self.current_min])
        time.sleep(0.25)

    def close(self):
        self.set_standalone_mode()
        self.close_serial_port()

    def close_serial_port(self):
        self.serial.close()

    def test_serial_port(self):
        if DEBUG:
            print "Testing port " + self.port
        serial_list = [SERIAL_COMMAND_GET_123]
        self.serial.write(str(serial_list))
        read_line = self.serial.readline()
        if (read_line == '') or (int(read_line) != 123):
            self.port_index += 1
            if self.port_index < len(self.port_list):
                self.port = self.port_list[self.port_index]
                self.test_serial_port()
            else:
                raise RuntimeError('Did not receive expected response on serial port, check port and connections.')
        if DEBUG:
            print "Test successful!"

    def set_computercontrol_mode(self):
        if DEBUG:
            print "Setting computercontrol mode."
        serial_list = [SERIAL_COMMAND_SET_COMPUTERCONTROL_MODE]
        self.serial.write(str(serial_list))

    def set_standalone_mode(self):
        if DEBUG:
            print "Setting standalone mode."
        serial_list = [SERIAL_COMMAND_SET_STANDALONE_MODE]
        self.serial.write(str(serial_list))

    def condition_channel(self,channel):
        if type(channel) == str:
            channel = channel.lower()
            if channel == 'a':
                channel = 0
            elif channel == 'b':
                channel = 1
            elif channel == 'c':
                channel = 2
            elif channel == 'd':
                channel = 3
            elif channel == 'all':
                channel = -1

        if channel not in range(-1,CHANNEL_COUNT):
            raise RuntimeError('Invalid channel, must be -1,0,1,2,3 or "all", "a", "b", "c", or "d"')

        return channel

    def turn_off_channel(self,channel='all'):
        channel = self.condition_channel(channel)
        if DEBUG:
            print "Turning off channel " + str(channel)

        if channel != -1:
            self.set_current_value(channel,self.current_min)
        else:
            self.set_current_values([self.current_min]*CHANNEL_COUNT)

    def set_current_value(self,channel='a',value=0):
        if (value < self.current_min) or (value > self.current_max):
            raise RuntimeError('Invalid current value, must be between ' + str(self.current_min) + ' - ' + self.current_max)

        channel = self.condition_channel(channel)

        if channel != -1:
            if DEBUG:
                print "Setting channel " + str(channel) + " to current value " + str(value)

            value = self.remap_output(value)
            serial_list = [SERIAL_COMMAND_SET_CURRENT_VALUE,channel,value]
            self.serial.write(str(serial_list))
        else:
            self.set_current_values([value]*CHANNEL_COUNT)

    def set_current_values(self,value_list=[0]*CHANNEL_COUNT):
        if (type(value_list) != list) and (type(value_list) != tuple):
            raise RuntimeError('set_current_values argument must be a list or tuple.')

        if (len(value_list) != CHANNEL_COUNT):
            raise RuntimeError('set_current_values argument length must equal ' + str(CHANNEL_COUNT))
        serial_list = [SERIAL_COMMAND_SET_CURRENT_VALUES]

        if DEBUG:
            print "Setting current values to " + str(value_list)

        value_list = self.remap_output(value_list)
        serial_list.extend(value_list)
        self.serial.write(str(serial_list))

    def set_bnc_modes(self,bnc_mode_list=[False]*CHANNEL_COUNT):
        if (type(bnc_mode_list) != list) and (type(bnc_mode_list) != tuple):
            raise RuntimeError('bnc_mode_list argument must be a list or tuple.')

        bnc_mode_list = [bool(bnc_mode) for bnc_mode in bnc_mode_list]
        serial_list = [SERIAL_COMMAND_SET_BNC_MODES]
        if len(bnc_mode_list) == CHANNEL_COUNT:
            if DEBUG:
                print "Setting bnc modes to " + str(bnc_mode_list)

            bnc_mode_list = [int(bnc_mode) for bnc_mode in bnc_mode_list]
            serial_list.extend(bnc_mode_list)
        else:
            raise RuntimeError('bnc_mode_list argument must be a list or tuple with length equal to ' + str(CHANNEL_COUNT))

        self.serial.write(str(serial_list))

    def set_bnc_mode(self,channel='a',bnc_mode=False):
        bnc_mode = bool(bnc_mode)
        channel = self.condition_channel(channel)
        if channel != -1:
            if DEBUG:
                print "Setting channel " + str(channel) + " to bnc mode " + str(bnc_mode)

            bnc_mode = int(bnc_mode)

            serial_list = [SERIAL_COMMAND_SET_BNC_MODE,channel,bnc_mode]
            self.serial.write(str(serial_list))
        else:
            self.set_bnc_modes([bnc_mode]*CHANNEL_COUNT)

    def remap_output(self,value):
        try:
            value_remapped = [remap(v,self.current_min,self.current_max,self.output_min,self.output_max) for v in value]
        except TypeError:
            value_remapped = remap(value,self.current_min,self.current_max,self.output_min,self.output_max)

        return value_remapped

# -----------------------------------------------------------------------
if __name__ == '__main__':
    cc = CurrentController()
    # cc.set_bnc_mode('b',True)
    # cc.set_bnc_mode('all',True)
    # cc.set_bnc_modes([False,False,False,False])

    for i in range(3):
        cc.set_current_values([100,0,500,100])
        time.sleep(1)
        cc.set_current_values([500,0,100,500])
        time.sleep(1)
        cc.turn_off_channel('c')
        cc.set_current_value('a',50)
        time.sleep(1)
    cc.set_current_values([0,0,0,0])
    # cc.set_bnc_modes()
    cc.close()
