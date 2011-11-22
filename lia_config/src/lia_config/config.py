import roslib
roslib.load_manifest('lia_config')
import rospy

# Interfaces and ports 
iface = 'eth0'
camera_mjpeg_port = rospy.get_param('/mjpeg_server_camera/port') 
progress_mjpeg_port = rospy.get_param('/mjpeg_server_progress/port')

# ROS image topics 
camera_topic = rospy.get_param('lia_image_throttle')
progress_bar_topic = '/image_progress_bar_throttle'
progress_message_topic = '/image_progress_message_throttle'
#progress_bar_topic = '/image_progress_bar'
#progress_message_topic = '/image_progress_message'

# Redis database
redis_db = 10
