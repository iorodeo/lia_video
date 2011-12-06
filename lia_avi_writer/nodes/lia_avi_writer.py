#!/usr/bin/env python
import roslib
roslib.load_manifest('lia_avi_writer')
import rospy
import sys
import os
import os.path
import cv
import threading
import math
import redis
import lia_config
from lia_web_interface import db_tools

# Messages
from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError
from lia_messages.msg import ProgressMsg

# Services
from lia_services.srv import RecordingCmd
from lia_services.srv import RecordingCmdResponse
from lia_services.srv import SetCurrentCmd

class LIA_AVI_Writer(object):
    """
    Avi writer for the light induced arousal application. 
    """

    def __init__(self,topic):

        # Video recording parameters
        self.topic = topic
        self.record_t = 10.0 
        self.start_t = 0.0
        self.current_t = 0.0
        self.progress_t = 0.0 
        self.frame_count = 0
        self.recording_message = 'stopped'
        self.frame_rate = 24.0 # This is a kludge - get from image topic
        self.writer = None 
        self.done = True 
        self.cv_image_size = None
        self.filename = os.path.join(os.environ['HOME'],'default.avi')

        # Current pulse parameters 
        self.pulse_channel = 'a'
        self.pulse_controller = None

        self.lock = threading.Lock()
        self.redis_db = redis.Redis('localhost', db=lia_config.redis_db)
        self.bridge = CvBridge()
        rospy.init_node('avi_writer')

        # Set up publications
        self.progress_msg = ProgressMsg()
        self.progress_pub = rospy.Publisher('progress',ProgressMsg)

        # Subscribe to messages
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_handler)

        # Set up services
        self.recording_srv = rospy.Service(
                'recording_cmd', 
                RecordingCmd, 
                self.handle_recording_cmd
                )

        # Set up proxy for set current service
        rospy.wait_for_service('set_current')
        self.set_current_proxy = rospy.ServiceProxy('set_current',SetCurrentCmd)

    def run(self):
        rospy.spin()

    def handle_recording_cmd(self,req):
        """
        Handles avi recording commands - starts and stops avi recording.
        """
        with self.lock:

            if self.cv_image_size is None:
                # If we don't have and image yet - we can't get started, return fail.
                return RecordingCmdResponse(False)

            self.filename = req.filename
            self.record_t = req.duration

            command = req.command.lower()
            if command == 'start':
                # Get start time and create video writer
                self.writer = cv.CreateVideoWriter(
                        self.filename,
                        cv.CV_FOURCC('D','I','V','X'),
                        self.frame_rate,
                        self.cv_image_size,
                        ) 

                if self.writer is None:
                    response = False
                else:
                    response = True
                    self.start_t = rospy.get_time()
                    self.frame_count = 0
                    self.done = False
                    self.recording_message = 'recording'

                    # Setup pulse controller 
                    trial_values = db_tools.get_dict(self.redis_db, 'trial_values')
                    self.pulse_controller = Pulse_Controller(
                            trial_values,
                            self.pulse_channel,
                            self.set_current_proxy,
                            )

            elif command == 'stop':
                self.done = True
                del self.writer
                self.writer = None
                self.recording_message = 'stopped'
                response = True

            return RecordingCmdResponse(response)

    def image_handler(self,data): 
        """
        Writes frames to avi file.
        """
        self.current_t = rospy.get_time()

        # Convert to opencv image and then to ipl_image
        cv_image = self.bridge.imgmsg_to_cv(data,'bgr8')
        ipl_image = cv.GetImage(cv_image)

        with self.lock:
            if self.cv_image_size == None:
                self.cv_image_size = cv.GetSize(cv_image)

        if not self.done:

            # Write video frame
            cv.WriteFrame(self.writer,ipl_image)

            # Update times and frame count - these are used elsewhere so we 
            # need the lock
            with self.lock:
                self.frame_count += 1
                self.progress_t = self.current_t - self.start_t
                self.pulse_controller.update(self.progress_t)

                # Check to see if we are done recording - if so stop writing frames 
                if self.current_t >= self.start_t + self.record_t:
                    self.done = True
                    del self.writer
                    self.writer = None
                    self.pulse_controller.set_pulse_low()
                    self.recording_message = 'finished'
                    self.redis_db.set('recording_flag',0)

        # Publish progress message
        with self.lock:
            self.progress_msg.frame_count = self.frame_count
            self.progress_msg.record_t = self.record_t
            self.progress_msg.progress_t = self.progress_t
            self.progress_msg.recording_message = self.recording_message
        self.progress_pub.publish(self.progress_msg)



class Pulse_Controller(object):

    def __init__(self, trial_values, set_current_func, channel):
        self.state = 'low'
        self.count = 0
        self.next_change_t = trial_values['pulse_start_time']
        self.duration = trial_values['pulse_high_time']
        self.period = trial_values['pulse_period']
        self.set_current_func = set_current_func
        self.channel = channel
        self.set_pulse_low()

    def set_pulse_high(self):
        self.set_current_func(self.channel,'on',self.pulse_current)
        self.state = 'high'

    def set_pulse_low(self):
        self.set_current_func(self.channel,'off',0)
        self.state = 'low'

    def update(self,t):
        if self.count < self.number_of_pulses and self.t >= self.next_change_t:
            if self.state == 'low':
                self.set_pulse_high()
                self.next_change_t = t+self.duration
            elif self.state == 'high':
                self.set_pulse_low()
                self.next_change_t = t+self.period-self.duration
                self.count += 1



# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = LIA_AVI_Writer(topic)
    node.run()


