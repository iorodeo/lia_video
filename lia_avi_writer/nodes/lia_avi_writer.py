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
from lia_web_interface import file_tools

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
        self.timing_fid = None

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

                    # Get trial values and setup pulse controller 
                    trial_values = db_tools.get_dict(self.redis_db, 'trial_values')
                    self.pulse_controller = Pulse_Controller(
                            trial_values,
                            self.pulse_channel,
                            self.set_current_proxy,
                            )

                    # Write settings file and open timing file
                    log_values = db_tools.get_dict(self.redis_db, 'log_values')
                    data_directory = log_values['data_directory']
                    settings_suffix = log_values['settings_file_suffix']
                    timing_suffix = log_values['timing_file_suffix']
                    metadata_filenames = file_tools.get_metadata_filenames(
                            self.filename,
                            data_directory,
                            settings_suffix,
                            timing_suffix,
                            )
                    settings_filename, timing_filename = metadata_filenames
                    settings_fid = open(settings_filename,'w')
                    for k,v in trial_values.iteritems():
                        settings_fid.write('{0}: {1}\n'.format(k,v))
                    settings_fid.close()
                    self.timing_fid = open(timing_filename,'w')
                   
            elif command == 'stop':
                self.cleanup_after_recording()
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
                self.update_timing_file()

                # Check to see if we are done recording - if so stop writing frames 
                if self.current_t >= self.start_t + self.record_t:
                    self.cleanup_after_recording()

        # Publish progress message
        with self.lock:
            self.progress_msg.frame_count = self.frame_count
            self.progress_msg.record_t = self.record_t
            self.progress_msg.progress_t = self.progress_t
            self.progress_msg.recording_message = self.recording_message
        self.progress_pub.publish(self.progress_msg)

    def update_timing_file(self): 
        """
        Updates information in timing file - frame count, time and pulse
        state.
        """
        if self.timing_fid is not None: 
            self.timing_fid.write('{0} '.format(self.frame_count))
            self.timing_fid.write('{0} '.format(self.progress_t))
            if self.pulse_controller.state == 'high':
                self.timing_fid.write('{0}\n'.format(1))
            else:
                self.timing_fid.write('{0}\n'.format(0))

    def cleanup_after_recording(self):
        """
        Cleans up after recording is stopped - closes open files, turns off current
        controller.

        Should alwars be called with the lock.
        """
        self.done = True
        del self.writer
        self.writer = None
        self.pulse_controller.set_pulse_low()
        self.pulse_controller.turn_off()
        self.recording_message = 'finished'
        self.redis_db.set('recording_flag',0)
        self.timing_fid.close()




class Pulse_Controller(object):

    def __init__(self, trial_values, channel, set_current_func):
        self.state = 'low'
        self.count = 0
        self.pulse_start_time = trial_time_to_secs(trial_values['pulse_start_time'])
        self.pulse_high_time = trial_time_to_secs(trial_values['pulse_high_time'])
        self.pulse_period = trial_time_to_secs(trial_values['pulse_period'])
        self.number_of_pulses = trial_values['number_of_pulses']
        self.pulse_current = trial_values['pulse_current']
        self.set_current_func = set_current_func
        self.channel = channel
        self.set_pulse_low()
        self.next_change_time = self.pulse_start_time

    def set_pulse_high(self):
        self.set_current_func(self.channel,'on',self.pulse_current)
        self.state = 'high'

    def set_pulse_low(self):
        self.set_current_func(self.channel,'on', 0)
        self.state = 'low'

    def turn_off(self):
        self.set_current_func(self.channel,'off',0)

    def update(self,t):
        if self.count < self.number_of_pulses and t >= self.next_change_time:
            if self.state == 'low':
                self.set_pulse_high()
                self.next_change_time += self.pulse_high_time
            elif self.state == 'high':
                self.set_pulse_low()
                self.count += 1
                self.next_change_time = self.pulse_start_time + self.count*self.pulse_period


def trial_time_to_secs(tt):
    return tt[0]*60*60 + tt[1]*60 + tt[2]

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    topic = sys.argv[1]
    node = LIA_AVI_Writer(topic)
    node.run()


