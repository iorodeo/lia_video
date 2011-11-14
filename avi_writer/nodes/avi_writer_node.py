#!/usr/bin/env python
import roslib
roslib.load_manifest('avi_writer')
import rospy
import sys
import cv
import numpy
import threading
import math

import Image as PILImage
import ImageDraw as PILImageDraw
import ImageFont as PILImageFont

# Messages
from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError

class AVI_Writer(object):

    def __init__(self,topic):
        self.lock = threading.Lock()
        self.filename = '/home/wbd/test.avi'
        self.topic = topic
        self.writer = None 
        self.frame_rate = 5.0 # This is a kludge - get from image topic
        self.done = False

        self.bridge = CvBridge()
        rospy.init_node('avi_writer')

        self.record_t = 1*60.0 
        self.start_t = 0.0
        self.current_t = 0.0
        self.progress_t = 0.0 
        self.frame_count = 0
        self.recording_message = 'recording'

        self.progress_bar = Progress_Bar()
        self.progress_message = Progress_Message()
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_handler)

    def run(self):
        rospy.spin()

    def image_handler(self,data): 

        if not self.done:

            self.current_t = rospy.get_time()

            # Convert to opencv image and then to ipl_image
            cv_image = self.bridge.imgmsg_to_cv(data,'bgr8')
            ipl_image = cv.GetImage(cv_image)

            # Create video writer - will move this to the start recording service
            if self.writer is None:
                self.writer = cv.CreateVideoWriter(
                        self.filename,
                        cv.CV_FOURCC('D','I','V','X'),
                        self.frame_rate,
                        cv.GetSize(cv_image)
                        ) 
                self.start_t = self.current_t 
                self.frame_count = 0
                if self.writer is None:
                    print "error creating video writer"
            self.progress_t = self.current_t - self.start_t

            # Convert to cvMat to IplImage and write video frame
            cv.WriteFrame(self.writer,ipl_image)
            
            # Update frame counter and shutdown when its time.
            self.frame_count += 1
            print 'frame: ', self.frame_count
            if self.current_t >= self.start_t + self.record_t:
                print 'done'
                self.done = True
                del self.writer
                self.writer = None
                self.recording_message = 'finished'

        # Update progress messages
        self.progress_bar.update(
                frame_count = self.frame_count,
                record_t = self.record_t,
                progress_t = self.progress_t
                )

        self.progress_message.update(
                message = self.recording_message, 
                frame_count =  self.frame_count,
                record_t = self.record_t,
                progress_t = self.progress_t
                )


class Progress_Bar(object):

    def __init__(self):
        self.image_shape = (15,640,3)
        self.empty_color = (230,230,230)
        self.fill_color = (0,0,200)
        self.base_array = 255*numpy.ones(self.image_shape,dtype=numpy.uint8)
        for i in range(0,3):
            self.base_array[:,:,i] = self.empty_color[i]
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('image_progress_bar', Image)

    def update(self,**kwargs): 
        frame_count = kwargs['frame_count']
        progress_t = kwargs['progress_t']
        record_t = kwargs['record_t']
        image_array = numpy.array(self.base_array)
        fill_ind = int(self.image_shape[1]*progress_t/record_t)
        for i in range(0,3):
            image_array[:,:fill_ind,i] = self.fill_color[i]
        cv_image = cv.fromarray(image_array)
        rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
        self.pub.publish(rosimage)

class Progress_Message(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('image_progress_message', Image)
        self.font = PILImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 20)
        self.fill = (0,0,0)
        self.count = 0

    def update(self,**kwargs): 
        # Extract keyword arguments
        message = kwargs['message']
        frame_count = kwargs['frame_count']
        record_t = kwargs['record_t']
        progress_t = kwargs['progress_t']

        # Get minutes and seconds
        record_min, record_sec = get_min_and_sec(record_t)
        progress_min, progress_sec = get_min_and_sec(progress_t)

        # Create PIL image and write text to it
        pil_image = PILImage.new('RGB',(640,30),(255,255,255))
        draw = PILImageDraw.Draw(pil_image)

        message_text = '%s,'%(message,)
        draw.text((0,10),message_text,font=self.font,fill=self.fill)

        frame_text = 'frame %d'%(frame_count,)
        draw.text((160,10),frame_text,font=self.font,fill=self.fill)

        time_text = '%d:%02d/%d:%02d'%(progress_min,progress_sec,record_min,record_sec)
        draw.text((450,10),time_text,font=self.font,fill=self.fill)

        # Convert to opencv image, then to ROS image and publish
        cv_image = cv.CreateImageHeader(pil_image.size, cv.IPL_DEPTH_8U, 3)
        cv.SetData(cv_image, pil_image.tostring())
        rosimage = self.bridge.cv_to_imgmsg(cv_image,'rgb8')
        self.pub.publish(rosimage)

        self.count+=1


def get_min_and_sec(t):
    t_min = int(math.floor(t/60.0))
    t_sec = int(math.floor(t - 60*t_min))
    return t_min, t_sec



# -----------------------------------------------------------------------------
if __name__ == '__main__':
    topic = sys.argv[1]
    node = AVI_Writer(topic)
    node.run()


