"""
Web interface for the LIA (Light Induced Arousal) Video Acquisition System.

"""
## Tornado web server
#from tornado.wsgi import WSGIContainer
#from tornado.httpserver import HTTPServer
#from tornado.ioloop import IOLoop

# Flask web framework imports
import sys
import os
import flask
import flaskext.sijax

import redis
import atexit

import iface_tools
import display_tools
import db_tools
import time_tools
import file_tools
import config

import roslib
roslib.load_manifest('lia_web_interface')
import rospy

# ROS Services
from lia_services.srv import RecordingCmd

# Set up application and database
app = flask.Flask(__name__)

db = redis.Redis('localhost', db=config.redis_db)
db_tools.set_dict(db,'setup_values',config.setup_defaults)
db.set('recording_flag', 0)

# Check for data directory - create if needed
file_tools.check_dir(config.data_directory)

# The path where the sijax extension puts its javascript files
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = os.path.join('.',os.path.dirname(__file__), 'static/json2.js') 

# Setup sijax
flaskext.sijax.init_sijax(app)

# Get hostaddr use ip address if possible
try:
    hostaddr= iface_tools.get_ip_addr('eth0')
except KeyError, NameError:
    print 'unable to get ip - using localhost'
    hostaddr = 'localhost'

# Basic set of keyword consants for use in calls to render_template
# Counld move these to the database .... 
BASE_KWARGS = {
        'page_header': config.page_header,
        'tab_dict': config.tab_dict,
        'tab_order': config.tab_order,
        'camera_topic': config.camera_topic,
        'progress_bar_topic': config.progress_bar_topic,
        'progress_message_topic': config.progress_message_topic,
        'capture_tab_image': config.capture_tab_image,
        'fullsize_tab_image': config.fullsize_tab_image,
        'camera_mjpeg_port': config.camera_mjpeg_port,
        'progress_mjpeg_port': config.progress_mjpeg_port,
        'hostaddr': hostaddr,
        }

@app.route('/')
def index():
    return flask.redirect(flask.url_for('capture'))

@flaskext.sijax.route(app, '/capture', methods=['GET','POST'])
def capture():
    """
    Handles requests for the capture control tab. Note, sijax is used to handle
    updating the text on the recording button - so that all computers viewing 
    the capture display are in sync. A javascript timer is used to periodically
    query for the recording state.
    """

    # Get recording flag and setup values from database
    recording_flag = db.get('recording_flag')
    setup_values = db_tools.get_dict(db,'setup_values')
    proxy_error_msg = ''

    # Define sijax request handler
    def update_recording_button(obj_response):
        recording_button_text = display_tools.get_recording_button_text(recording_flag)
        obj_response.html("#recording_button",recording_button_text)

    if flask.g.sijax.is_sijax_request:
        # This is a sijax request - let sijax handle it
        flask.g.sijax.register_callback('update_recording_button', update_recording_button)
        return flask.g.sijax.process_request()

    else:

        # This is a normal request

        # If this is post toggle state of recording flag
        if flask.request.method == 'POST':

            if recording_flag == 0:
                # Start recording
                recording_flag = 1
                recording_cmd = 'start'
            else:
                # Stop recording
                recording_flag = 0
                recording_cmd = 'stop'

            # Use Ros service to send command to avi writer 
            fullpath_filename = os.path.join(config.data_directory,setup_values['movie_file'])
            t_min, t_sec = setup_values['recording_duration']
            recording_duration = 60.0*t_min + 1.0*t_sec
            try:
                recording_cmd_proxy = rospy.ServiceProxy('recording_cmd',RecordingCmd)
                response = recording_cmd_proxy( 
                        recording_cmd,
                        fullpath_filename,
                        recording_duration,
                        )
            except rospy.ServiceException, e:
                proxy_error_msg = str(e)
        else:
            pass
        db.set('recording_flag',recording_flag)

        # Set the recording button text based on whether or not we
        # are currently recording - sijax may make this unecessary, but it doesn't hurt
        recording_button_text = display_tools.get_recording_button_text(recording_flag)

        # Create the kwargs to pass to the render template function
        kwargs = dict(BASE_KWARGS)
        kwargs['current_tab'] = config.tab_dict['capture']['tab']
        kwargs['setup_display'] = display_tools.get_setup_display(setup_values)
        kwargs['recording_button_text'] = recording_button_text
        kwargs['proxy_error_msg'] = proxy_error_msg
        return flask.render_template('capture.html',**kwargs)

@app.route('/setup',methods=['GET','POST'])
def setup():
    """
    Handles request for the setup or (change settings) tab.
    """
    t_min = None
    recording_flag = db.get('recording_flag')
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = config.tab_dict['setup']['tab']

    setup_values = db_tools.get_dict(db,'setup_values')

    if flask.request.method == 'POST':
        if 'submit_values' in flask.request.form:
            setup_values['movie_file'] = str(flask.request.form['movie_file'])
            t_min = int(flask.request.form['duration_min'])
            t_sec = int(flask.request.form['duration_sec'])
            setup_values['recording_duration'] = (t_min, t_sec)
            db_tools.set_dict(db,'setup_values', setup_values)
    else:
        pass

    kwargs.update(setup_values)
    
    if recording_flag == 0:
        kwargs['disabled'] = ''
        kwargs['test'] = type(t_min)
        return flask.render_template('setup.html',**kwargs)
    else:
        kwargs['disabled'] = 'disabled'
        return flask.render_template('setup.html',**kwargs)

@app.route('/fullsize_view', methods=['GET'])
def fullsize_view():
    """
    Handles requests for the fullsize view page
    """
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = config.tab_dict['fullsize_view']['tab']
    scale_options = config.fullsize_scale_options

    new_scale = flask.request.args.get('scale','1.0')
    if not new_scale in scale_options:
        current_scale = scale_options[0]
    else:
        current_scale = new_scale

    #Set image size
    scale = float(current_scale)
    image_width = int(scale*config.fullsize_tab_image['width'])
    image_height = int(scale*config.fullsize_tab_image['height'])

    # Set arguments to send to renderer
    kwargs['current_scale'] = current_scale
    kwargs['scale_options'] = scale_options
    kwargs['image_width'] = image_width
    kwargs['image_height'] = image_height

    return flask.render_template('fullsize_view.html',**kwargs)

@app.route('/info')
def info():
    """
    Handles requests for info tab 
    """
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = config.tab_dict['info']['tab'] 
    uname = os.uname()

    # Get computer info
    computer_info = [
            ('Hostname:', uname[1] ),
            ('IP Address:', kwargs['hostaddr'] ), 
            ('System:', '%s %s'%(uname[0], uname[3]) ),
            ('Machine:', uname[4]),
                ]
    computer_info = display_tools.get_colored_list(computer_info,color_vals=('c2','c1'))

    # Get ROS topic info
    ros_topics = [topic[0] for topic in rospy.get_published_topics()]
    ros_topics.sort()
    ros_topics = display_tools.get_colored_list(ros_topics,color_vals=('c2','c1'))

    # Get db and port info
    db_port_info = [
            ('Redis Database #:', '%s'%(config.redis_db,)),
            ('Camera MJPEG Port:', '%s'%(config.camera_mjpeg_port,)),
            ('Progress MJPEG Port:', '%s'%(config.progress_mjpeg_port,)),
            ]
    db_port_info = display_tools.get_colored_list(db_port_info,color_vals=('c2','c1'))

    # Create keyword arguments for render_template
    kwargs['data_directory'] = config.data_directory
    kwargs['computer_info'] = computer_info
    kwargs['ros_topics'] = ros_topics
    kwargs['db_port_info'] = db_port_info
    
    return flask.render_template('info.html',**kwargs)

@app.route('/docs')
def docs():
    """
    Handles requests for the docs (or Manual) tab.
    """
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = config.tab_dict['docs']['tab'] 
    return flask.render_template('docs.html',**kwargs)

def cleanup():
    for k in db.keys('*'):
        db.delete(k)
atexit.register(cleanup)


# -----------------------------------------------------------------------------
if __name__ == '__main__': 
    if 1:
        app.debug = True 
        app.run()
    if 0:
        app.run(host='0.0.0.0')

    if 0:
        http_server = HTTPServer(WSGIContainer(app))
        http_server.listen(5000)
        IOLoop.instance().start()
