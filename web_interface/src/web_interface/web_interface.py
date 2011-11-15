"""
Web interface for the LIA (Light Induced Arousal) Video Acquisition System.

"""
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
import config

import roslib
roslib.load_manifest('web_interface')
import rospy

# ROS Services
from lia_services.srv import RecordingCmd

# Set up application and database
app = flask.Flask(__name__)
db = redis.Redis('localhost', db=10)
db_tools.set_dict(db,'setup_values',config.setup_defaults)
db.set('recording_flag', 0)

# The path where the sijax extension puts its javascript files
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
## json2.js library if you want to support for  browsers that don't support JSON natively (like IE <= 7)
#app.config["SIJAX_JSON_URI"] = '/home/wbd/pyenv/wbdpy/lib/python2.7/site-packages/sijax/js/json2.js'

# Setup sijax
flaskext.sijax.init_sijax(app)

# Get hostname use ip address if possible
try:
    hostname = iface_tools.get_ip_addr('eth0')
except KeyError, NameError:
    print 'unable to get ip - using localhost'
    hostname = 'localhost'

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
        'hostname': hostname,
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

    # Get recording flag from database
    recording_flag = db.get('recording_flag')
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
            try:
                recording_cmd_proxy = rospy.ServiceProxy('recording_cmd',RecordingCmd)
                response = recording_cmd_proxy(recording_cmd,'dummy_file.avi')
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
        setup_values = db_tools.get_dict(db,'setup_values')
        kwargs['setup_display'] = display_tools.get_setup_display(setup_values)
        kwargs['recording_button_text'] = recording_button_text
        kwargs['proxy_error_msg'] = proxy_error_msg
        return flask.render_template('capture.html',**kwargs)

@app.route('/setup',methods=['GET','POST'])
def setup():
    """
    Handles request for the setup or (change settings) tab.
    """
    recording_flag = db.get('recording_flag')
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = config.tab_dict['setup']['tab']
    setup_values = db_tools.get_dict(db,'setup_values')
    kwargs.update(setup_values)
    
    if recording_flag == 0:
        kwargs['disabled'] = ''
        return flask.render_template('setup.html',**kwargs)
    else:
        kwargs['disabled'] = 'disabled'
        return flask.render_template('setup.html',**kwargs)

@app.route('/fullsize_view', methods=['GET','POST'])
def fullsize_view():
    """
    Handles requests for the fullsize view page
    """
    if flask.request.method == 'POST':
        current_scale = flask.request.form['scale']
    else:
        current_scale = '1.0'
        
    # Set image size
    scale = float(current_scale)
    image_width = int(scale*config.fullsize_tab_image['width'])
    image_height = int(scale*config.fullsize_tab_image['height'])

    # Set arguments to send to renderer
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = config.tab_dict['fullsize_view']['tab']
    scale_options = [str(0.1*x) for x in range(10,1,-1)]
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
    if 0:
        app.debug = True
        app.run()
    if 1:
        app.run(host='0.0.0.0')
