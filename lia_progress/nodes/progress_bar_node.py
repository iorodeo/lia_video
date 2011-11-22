#!/usr/bin/env python
import roslib
roslib.load_manifest('lia_progress')
import rospy
import numpy
import cv

from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError
from lia_messages.msg import ProgressMsg

class Progress_Bar(object):

    def __init__(self):
        self.image_shape = (15,640,3)
        self.empty_color = (230,230,230)
        self.fill_color = (0,0,200)
        self.base_array = 255*numpy.ones(self.image_shape,dtype=numpy.uint8)
        for i in range(0,3):
            self.base_array[:,:,i] = self.empty_color[i]
        self.bridge = CvBridge()
        rospy.init_node('progress_bar')

        # Pulications
        self.pub = rospy.Publisher('image_progress_bar', Image)

        # Subscriptions
        self.sub = rospy.Subscriber('progress',ProgressMsg,self.handle_progress_msg)


    def handle_progress_msg(self,data):
        frame_count = data.frame_count 
        progress_t = data.progress_t 
        record_t = data.record_t
        image_array = numpy.array(self.base_array)
        if record_t > 0:
            fill_ind = int(self.image_shape[1]*progress_t/record_t)
        else:
            fill_ind = self.image_shape[1]
        for i in range(0,3):
            image_array[:,:fill_ind,i] = self.fill_color[i]
        cv_image = cv.fromarray(image_array)
        rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
        self.pub.publish(rosimage)

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Progress_Bar()
    node.run()


