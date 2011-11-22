#!/usr/bin/env python
import roslib
roslib.load_manifest('lia_fps_monitor')
import rospy
import sys
import threading
import math

from sensor_msgs.msg import CameraInfo
from lia_messages.msg import FPSInfoMsg

class FPS_Monitor(object):
    """
    Receives camera info messages for a topic and computes frequency stats

    This is based on ROSTopicHz
    """
    def __init__(self, topic, window_size=75, filter_expr=None):
        self.topic = topic
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.times =[]
        self.filter_expr = filter_expr
        
        # can't have infinite window size due to memory restrictions
        if window_size < 0:
            window_size = 50000
        self.window_size = window_size

        rospy.init_node('fps_monitor')

        self.fps_msg = FPSInfoMsg()
        self.fps_pub = rospy.Publisher('fps_info',FPSInfoMsg)
        self.image_info_sub = rospy.Subscriber(self.topic,CameraInfo,self.handle_camera_info)
                
    def handle_camera_info(self, m):
        # ignore messages that don't match filter
        if self.filter_expr is not None and not self.filter_expr(m):
            return
        with self.lock:
            curr_rostime = rospy.get_rostime()

            # time reset
            if curr_rostime.is_zero():
                if len(self.times) > 0:
                    print("time has reset, resetting counters")
                    self.times = []
                return
            
            curr = curr_rostime.to_sec()
            if self.msg_t0 < 0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_tn = curr
                self.times = []
            else:
                self.times.append(curr - self.msg_tn)
                self.msg_tn = curr

            #only keep statistics for the last 10000 messages so as not to run out of memory
            if len(self.times) > self.window_size - 1:
                self.times.pop(0)
                
        self.publish_fps_info()

    def publish_fps_info(self):
        """
        print the average publishing rate to screen
        """
        if not self.times:
            return
        elif self.msg_tn == self.last_printed_tn:
            print("no new messages")
            return
        with self.lock:
            #frequency
            
            # kwc: In the past, the rate decayed when a publisher
            # dies.  Now, we use the last received message to perform
            # the calculation.  This change was made because we now
            # report a count and keep track of last_printed_tn.  This
            # makes it easier for users to see when a publisher dies,
            # so the decay is no longer necessary.
            
            n = len(self.times)
            #rate = (n - 1) / (rospy.get_time() - self.msg_t0)
            mean = sum(self.times) / n
            rate = 1./mean if mean > 0. else 0

            #std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) /n)

            # min and max
            max_delta = max(self.times)
            min_delta = min(self.times)

            self.last_printed_tn = self.msg_tn

        #print("average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%(rate, min_delta, max_delta, std_dev, n+1))
        # Publish frequecy information
        self.fps_msg.rate = rate
        self.fps_msg.min_delta = min_delta
        self.fps_msg.max_delta = max_delta
        self.fps_msg.std_dev = std_dev
        self.fps_msg.window = n+1
        self.fps_pub.publish(self.fps_msg)

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__=='__main__':
    topic = sys.argv[1]
    node = FPS_Monitor(topic)
    node.run()
