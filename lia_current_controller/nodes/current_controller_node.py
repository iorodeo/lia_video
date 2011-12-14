#!/usr/bin/env python
import roslib
roslib.load_manifest('lia_current_controller')
import rospy
import threading
import lia_current_controller

from lia_services.srv import SetCurrentCmd
from lia_services.srv import SetCurrentCmdResponse

class CurrentController(object):

    """
    ROS node which provides a simple interface to IO Rodeo's LED
    current controller.
    """
    def __init__(self,port='/dev/ledcontrol'):

        self.lock = threading.Lock()
        self.channel_list = ['a','b','c','d']
        self.state_list = ['on', 'off']
        self.min_value = 0
        self.max_value = 1000

        self.dev = lia_current_controller.CurrentController(port)

        rospy.init_node('current_controller')
        rospy.on_shutdown(self.on_shutdown)
        rospy.set_current_srv = rospy.Service(
                'set_current',
                SetCurrentCmd,
                self.handle_set_current
                )
        with self.lock:
            for chan in self.channel_list:
                self.dev.turn_off_channel(chan)

    def run(self):
        rospy.spin()

    def handle_set_current(self,req):
        """
        Handles requests to set current values and to turn the channels
        on/off.
        """
        with self.lock:
            channel = req.channel.lower()
            if not channel in self.channel_list:
                return SetCurrentCmdResponse(False)

            state = req.state.lower()
            if not state in self.state_list: 
                return SetCurrentCmdResponse(False)

            if state == 'on':
                value = req.value
                if value < self.min_value or value > self.max_value:
                    return SetCurrentCmdResponse(False)
                else:
                    self.dev.set_current_value(channel,value)
                    return SetCurrentCmdResponse(True)
            elif  state == 'off':
                self.dev.turn_off_channel(channel)
                return SetCurrentCmdResponse(True)

    def on_shutdown(self):
        with self.lock:
            for chan in self.channel_list:
                self.dev.turn_off_channel(chan)
            self.dev.set_standalone_mode()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = CurrentController()
    node.run()

    
