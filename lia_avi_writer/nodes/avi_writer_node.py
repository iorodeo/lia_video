#!/usr/bin/env python
import roslib
roslib.load_manifest('lia_avi_writer')
import rospy
import sys
import os
import os.path
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

# Services
from lia_services.srv import RecordingCmd
from lia_services.srv import RecordingCmdResponse

class AVI_Writer(object):

    def __init__(self,topic):
        self.lock = threading.Lock()
        self.filename = os.path.join(os.environ['HOME'],'default.avi')
        self.topic = topic
        self.writer = None 
        self.frame_rate = 24.0 # This is a kludge - get from image topic
        self.done = True 
        self.cv_image_size = None

        self.bridge = CvBridge()
        rospy.init_node('avi_writer')

        self.record_t = 1*60.0 
        self.start_t = 0.0
        self.current_t = 0.0
        self.progress_t = 0.0 
        self.frame_count = 0
        self.recording_message = 'stopped'

        self.progress_bar = Progress_Bar()
        self.progress_message = Progress_Message()
        self.image_sub = rospy.Subscriber(self.topic,Image,self.image_handler)
        self.recording_srv = rospy.Service(
                'recording_cmd', 
                RecordingCmd, 
                self.handle_recording_cmd
                )

    def run(self):
        rospy.spin()

    def handle_recording_cmd(self,req):
        print 'recording_cmd'

        with self.lock:

            if self.cv_image_size is None:
                # If we don't have and image yet - we can't get started, return fail.
                return RecordingCmdResponse(False)

            self.filename = req.filename
            self.record_t = req.duration

            command = req.command.lower()
            if command == 'start':
                print 'start'
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

            elif command == 'stop':
                print 'stop'
                self.done = True
                del self.writer
                self.writer = None
                self.recording_message = 'stopped'
                response = True

            return RecordingCmdResponse(response)

    def image_handler(self,data): 
        self.current_t = rospy.get_time()

        # Convert to opencv image and then to ipl_image
        cv_image = self.bridge.imgmsg_to_cv(data,'bgr8')
        ipl_image = cv.GetImage(cv_image)

        with self.lock:
            if self.cv_image_size == None:
                self.cv_image_size = cv.GetSize(cv_image)

        if not self.done:

            # Update times and frame count - these are used elsewhere so we 
            # need the lock
            with self.lock:
                self.progress_t = self.current_t - self.start_t
                self.frame_count += 1
            print 'frame: ', self.frame_count

            # Write video frame
            cv.WriteFrame(self.writer,ipl_image)
            
            # Check to see if we are done recording - if so stop writing frames 
            if self.current_t >= self.start_t + self.record_t:
                with self.lock:
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


