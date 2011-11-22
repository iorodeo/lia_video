#!/usr/bin/env python
import roslib
roslib.load_manifest('lia_fps_monitor')
import rospy
import cv
import math

import Image as PILImage
import ImageDraw as PILImageDraw
import ImageFont as PILImageFont

from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError
from lia_messages.msg import FPSInfoMsg

class FPS_Image(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.font = PILImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 20)
        self.fill = (0,00,0)
        self.bg_color = (255,255,255)
        #self.bg_color = (200,200,255)
        self.image_size = (105,28)
        self.text_pos = (0,4)
        rospy.init_node('fps_info_image')

        self.pub = rospy.Publisher('image_fps', Image)
        self.sub = rospy.Subscriber('fps_info',FPSInfoMsg, self.handle_fps_msg)

    def handle_fps_msg(self,data):
        # Extract keyword arguments
        fps = data.rate

        # Create PIL image and write text to it
        pil_image = PILImage.new('RGB',self.image_size,self.bg_color)
        draw = PILImageDraw.Draw(pil_image)

        fps_text = 'fps %2.1f'%(fps,)
        draw.text(self.text_pos,fps_text,font=self.font,fill=self.fill)

        # Convert to opencv image, then to ROS image and publish
        cv_image = cv.CreateImageHeader(pil_image.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_image, pil_image.tostring())
        rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
        self.pub.publish(rosimage)

    def run(self):
        rospy.spin()

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    node = FPS_Image()
    node.run()
