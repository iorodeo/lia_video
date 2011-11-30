#!/usr/bin/env python
import roslib
roslib.load_manifest('lia_progress')
import rospy
import cv
import math
import os.path

import Image as PILImage
import ImageDraw as PILImageDraw
import ImageFont as PILImageFont

from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError
from lia_messages.msg import ProgressMsg

class Progress_Info(object):

    def __init__(self):
        self.bridge = CvBridge()
        file_dir, file_name = os.path.split(__file__)
        font_path = '%s/../fonts/FreeMono.ttf'%(file_dir,)
        self.font = PILImageFont.truetype(font_path, 20)
        self.fill = (0,0,0)
        rospy.init_node('progress_info')
        self.pub = rospy.Publisher('image_progress_message', Image)
        self.sub = rospy.Subscriber('progress',ProgressMsg, self.handle_progress_msg)


    def handle_progress_msg(self,data): 
        # Extract keyword arguments
        message = data.recording_message 
        frame_count = data.frame_count 
        record_t = data.record_t 
        progress_t = data.progress_t

        # Get minutes and seconds
        record_hr, record_min, record_sec = get_hr_min_sec(record_t)
        progress_hr, progress_min, progress_sec = get_hr_min_sec(progress_t)

        # Create PIL image and write text to it
        pil_image = PILImage.new('RGB',(640,30),(255,255,255))
        draw = PILImageDraw.Draw(pil_image)

        message_text = '%s,'%(message,)
        draw.text((0,10),message_text,font=self.font,fill=self.fill)

        frame_text = 'frame %d'%(frame_count,)
        draw.text((160,10),frame_text,font=self.font,fill=self.fill)

        time_data = (progress_hr,progress_min,progress_sec,record_hr,record_min,record_sec)
        time_text = '%02d:%02d:%02d/%02d:%02d:%02d'%time_data
        draw.text((430,10),time_text,font=self.font,fill=self.fill)

        # Convert to opencv image, then to ROS image and publish
        cv_image = cv.CreateImageHeader(pil_image.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_image, pil_image.tostring())
        rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
        self.pub.publish(rosimage)

    def run(self):
        rospy.spin()

def get_hr_min_sec(t):
    t_hr = int(math.floor(t/3600.0))
    t_rem = t - t_hr*3600.0
    t_min = int(math.floor(t_rem/60.0))
    t_sec = int(math.floor(t_rem - 60*t_min))
    return t_hr, t_min, t_sec

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    node = Progress_Info()
    node.run()
