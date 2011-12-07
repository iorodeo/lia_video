#!/usr/bin/env python
"""
Web interface for the LIA (Light Induced Arousal) Video Acquisition System.

"""
# Tornado web server
from tornado.wsgi import WSGIContainer
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop

# Flask web framework imports
import flask
import flaskext.sijax

import re
import sys
import os
import redis
import atexit
import time

import config
import display_tools
import db_tools
import file_tools
import form_tools
import config_tools
import user_tools

import roslib
roslib.load_manifest('lia_web_interface')
import rospy

# ROS Services
from lia_services.srv import RecordingCmd

# Set up application and database
app = flask.Flask(__name__)
db = redis.Redis('localhost', db=config.redis_db)
db_tools.init_db(db)

# Check for data directory - create if needed
log_values = db_tools.get_dict(db,'log_values')
file_tools.check_dir(log_values['data_directory'])

# The path where the sijax extension puts its javascript files
app.config["SIJAX_STATIC_PATH"] = os.path.join('.', os.path.dirname(__file__), 'static/js/sijax/')
app.config["SIJAX_JSON_URI"] = os.path.join('.',os.path.dirname(__file__), 'static/json2.js') 

# Add secret key - not so secrete really
app.config['SECRET_KEY'] = 'development key' 

# Setup sijax
flaskext.sijax.init_sijax(app)

# Routes
# --------------------------------------------------------------------------------

@app.route('/')
def index():
    if not flask.session.get('logged_in'):
        return flask.redirect(flask.url_for('login'))
    else:
        return flask.redirect(flask.url_for('capture'))

@app.route('/login', methods=['GET','POST'])
def login():
    error = None
    if flask.request.method == 'POST':
        if flask.request.form['username'] != config.username: 
            error = 'Invalid username'
        elif flask.request.form['password'] != config.password:
            error = 'Invalid password'
        else:
            flask.session['logged_in'] = True
            flask.flash('You were logged in')
            return flask.redirect(flask.url_for('capture'))
    kwargs = dict(form_tools.get_base_kwargs())
    kwargs['error'] = error
    return flask.render_template('user_login.html', **kwargs)

@app.route('/logout')
def logout():
    flask.session.pop('logged_in', None)
    flask.flash('You were logged out')
    return flask.redirect(flask.url_for('index'))

@flaskext.sijax.route(app, '/capture', methods=['GET','POST'])
@user_tools.check_login
def capture():
    """
    Handles requests for the capture control tab. Note, sijax is used to handle
    updating the text on the recording button - so that all computers viewing 
    the capture display are in sync. A javascript timer is used to periodically
    query for the recording state.
    """
    if flask.g.sijax.is_sijax_request:
        # This is a sijax request - let sijax handle it
        flask.g.sijax.register_callback('update_recording_button', update_recording_button)
        flask.g.sijax.register_callback('start_stop_recording', start_stop_recording)
        return flask.g.sijax.process_request()

    else:
        recording_flag = db.get('recording_flag')
        trial_values = db_tools.get_dict(db,'trial_values')
        log_values = db_tools.get_dict(db,'log_values')
        recording_button_text = display_tools.get_recording_button_text(recording_flag)

        # Create the kwargs to pass to the render template function
        kwargs = dict(form_tools.get_base_kwargs())
        kwargs['current_tab'] = config.tab_dict['capture']['tab']
        kwargs['log_display'] = display_tools.get_log_display(log_values)
        kwargs['trial_display'] = display_tools.get_trial_display(trial_values)
        kwargs['recording_button_text'] = recording_button_text

        return flask.render_template('capture.html',**kwargs)

@app.route('/fullsize_view', methods=['GET'])
@user_tools.check_login
def fullsize_view():
    """
    Handles requests for the fullsize view page
    """
    kwargs = dict(form_tools.get_base_kwargs())
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

@app.route('/trial_settings',methods=['GET','POST'])
@user_tools.check_login
def trial_settings():
    """
    Handles request for the trial settings tab.
    """
    recording_flag = db.get('recording_flag')
    kwargs = dict(form_tools.get_base_kwargs())
    kwargs['current_tab'] = config.tab_dict['trial_settings']['tab']

    trial_values = db_tools.get_dict(db,'trial_values')
    saved_trials = db_tools.get_dict(db,'saved_trials')

    values_set_flag = False
    no_save_name_flag = False
    trial_name_exists_flag = False
    trial_saved_flag = False
    saved_trial_name = ''
    loaded_trial_name = ''
    deleted_trial_names = []

    if flask.request.method == 'POST':

        if 'set_values' in flask.request.form:

            # Extract trial values from request form
            trial_values = form_tools.update_trial_values(trial_values,flask.request.form)
            db_tools.set_dict(db,'trial_values', trial_values)
            values_set_flag = True

        elif 'save_values' in flask.request.form:
            trial_name = str(flask.request.form['save_name'])
            if trial_name:
                # Extract trial values form request form
                trial_values = form_tools.update_trial_values(trial_values,flask.request.form)
                trial_names = [v['name'] for k,v in saved_trials.iteritems()]
                if trial_name in trial_names:
                    trial_name_exists_flag = True
                else:
                    trial_tag = trial_name.replace(' ', '_')
                    saved_trials[trial_tag] = {'name': trial_name, 'values': trial_values}
                    db_tools.set_dict(db,'saved_trials',saved_trials)
                    trial_saved_flag = True
                    saved_trial_name = trial_name
            else:
                no_save_name_flag = True

        elif 'delete_selected_trials' in flask.request.form:
            deleted_trial_tags = form_tools.find_all_prefix_match('checkbox_',flask.request.form)
            for tag in deleted_trial_tags:
                try:
                    deleted_trial_names.append(saved_trials[tag]['name'])
                    del saved_trials[tag]
                except KeyError:
                    pass
                db_tools.set_dict(db,'saved_trials',saved_trials)

        else:
            # Check if this is a request to delete a saved trial or to load
            # a saved trial. 
            delete_tag = form_tools.check_for_delete_tag(flask.request.form)
            load_tag = form_tools.check_for_load_tag(flask.request.form)
            if delete_tag is not None:
                # This is a request to delete a saved trial
                try:
                    deleted_trial_names.append(saved_trials[delete_tag]['name'])
                    del saved_trials[delete_tag]
                except KeyError:
                    pass
                db_tools.set_dict(db,'saved_trials',saved_trials)

            if load_tag is not None:
                # This is a request to load a saved trial
                try:
                    trial_values = saved_trials[load_tag]['values']
                    loaded_trial_name = saved_trials[load_tag]['name']
                except KeyError:
                    pass
                db_tools.set_dict(db,'trial_values', trial_values)


    kwargs.update(trial_values)
    kwargs['time_labels'] = display_tools.get_time_labels() 
    kwargs['trial_display'] = display_tools.get_trial_display(trial_values)

    kwargs['no_save_name_flag'] = no_save_name_flag
    kwargs['trial_name_exists_flag'] = trial_name_exists_flag
    kwargs['values_set_flag'] = values_set_flag
    kwargs['trial_saved_flag'] = trial_saved_flag
    kwargs['flags'] = {
            'no_save_name': no_save_name_flag,
            'trial_name_exists': trial_name_exists_flag,
            'values_set': values_set_flag,
            'trial_saved': trial_saved_flag,
            }
    kwargs['saved_trials'] = saved_trials
    kwargs['saved_trial_name'] = saved_trial_name
    kwargs['loaded_trial_name'] = loaded_trial_name
    kwargs['deleted_trial_names'] = deleted_trial_names

    # Create display data for saved trials
    saved_trials_display = []
    for trial_tag, trial_dict in saved_trials.iteritems():
        trial_name = trial_dict['name']
        trial_values = trial_dict['values']
        trial_display = display_tools.get_trial_display(trial_values)
        saved_trials_display.append((trial_name, trial_tag, trial_display))
    saved_trials_display.sort()
    saved_trials_display = display_tools.get_colored_list(saved_trials_display,color_vals=('c2','c1'))
    kwargs['saved_trials_display'] = saved_trials_display
    
    if recording_flag == 0:
        kwargs['disabled'] = ''
    else:
        kwargs['disabled'] = 'disabled'
    return flask.render_template('trial_settings.html',**kwargs)

@app.route('/logging', methods=['GET','POST'])
@user_tools.check_login
def logging():
    """
    Handlers requests for the logging tab
    """
    recording_flag = db.get('recording_flag')
    kwargs = dict(form_tools.get_base_kwargs())
    kwargs['current_tab'] = config.tab_dict['logging']['tab']
    log_values = db_tools.get_dict(db,'log_values')

    set_values_flag = False
    deleted_avi_files = []
    if flask.request.method == 'POST':

        if 'set_values' in flask.request.form:
            log_values = form_tools.update_log_values(log_values,flask.request.form)
            existing_avi = file_tools.get_existing_avi(log_values['data_directory'])
            db_tools.set_dict(db,'log_values', log_values)
            set_values_flag = True
        if 'delete_selected_files' in flask.request.form:
            selected_avi_files = form_tools.get_selected_avi_files(flask.request.form)
            file_tools.delete_data_files(selected_avi_files, log_values)
            deleted_avi_files = selected_avi_files

    kwargs.update(log_values)
    kwargs['set_values_flag'] = set_values_flag
    kwargs['log_display'] = display_tools.get_log_display(log_values,show_file_suffix=True)
    kwargs['deleted_avi_files'] = deleted_avi_files

    # Get list of existing avi files with size and time information
    existing_avi = file_tools.get_existing_avi(log_values['data_directory'],info=True)
    avi_display = display_tools.get_colored_list(existing_avi,color_vals=('c2','c1'))

    kwargs['avi_display'] = avi_display 
    if recording_flag == 0:
        kwargs['disabled'] = ''
    else:
        kwargs['disabled'] = 'disabled'
    return flask.render_template('logging.html', **kwargs)

@app.route('/info')
@user_tools.check_login
def info():
    """
    Handles requests for info tab 
    """
    kwargs = dict(form_tools.get_base_kwargs())
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
    log_values = db_tools.get_dict(db,'log_values')
    kwargs['data_directory'] = log_values['data_directory']
    kwargs['computer_info'] = computer_info
    kwargs['ros_topics'] = ros_topics
    kwargs['db_port_info'] = db_port_info
    
    return flask.render_template('info.html',**kwargs)

@app.route('/docs')
@user_tools.check_login
def docs():
    """
    Handles requests for the docs (or Manual) tab.
    """
    kwargs = dict(form_tools.get_base_kwargs())
    kwargs['current_tab'] = config.tab_dict['docs']['tab'] 
    return flask.render_template('docs.html',**kwargs)

def cleanup():
    for k in db.keys('*'):
        if not str(k)=='saved_trials':
            db.delete(k)
    db.save()

atexit.register(cleanup)


# Sijax request handlers 
# ----------------------------------------------------------------------------------
def update_recording_button(obj_response):
    """
    Handles update request for the recording button text - keeps different clients in 
    sync.
    """
    recording_flag = db.get('recording_flag')
    trial_values = db_tools.get_dict(db,'trial_values')
    log_values = db_tools.get_dict(db,'log_values')

    # Update recording button
    recording_button_text = display_tools.get_recording_button_text(recording_flag)
    obj_response.html("#recording_button",recording_button_text)

    # Update trial and log values
    trial_display = display_tools.get_trial_display(trial_values)
    for item in trial_display:
        name = item['name'].replace(' ','_')
        obj_response.html('#%s_value'%(name,),item['value'])
        obj_response.html('#%s_units'%(name,),item['units'])

    log_display = display_tools.get_log_display(log_values)
    for item in log_display:
        name = item['name'].replace(' ','_')
        obj_response.html('#%s'%(name,), item['value'])
    return

def start_stop_recording(obj_response): 
    """
    Handles requests to start/stop recording video
    """

    recording_flag = db.get('recording_flag')
    trial_values = db_tools.get_dict(db,'trial_values')
    log_values = db_tools.get_dict(db,'log_values')

    # Get movie file name
    movie_file = log_values['movie_file']
    if log_values['append_datetime'] == 'yes': 
        movie_file = file_tools.add_datetime_suffix(movie_file)

    if recording_flag == 0:  # Start recording

        # Check to see if file exists and if overwrite=no abort recording
        is_existing_avi = file_tools.is_existing_avi(log_values['data_directory'],movie_file)
        if log_values['overwrite'] == 'no' and is_existing_avi:
            obj_response.html('#avi_exists_message', 'Aborted - movie file exists, overwrite=no')
            return

        else:
            recording_flag = 1
            recording_cmd = 'start'

    else: # Stop recording

        recording_flag = 0
        recording_cmd = 'stop'

    # Use Ros service to send command to avi writer 
    fullpath_filename = os.path.join(log_values['data_directory'],movie_file)
    t_hr, t_min, t_sec = trial_values['recording_duration']
    recording_duration = 3600.0*t_hr+ 60.0*t_min + 1.0*t_sec

    try:
        recording_cmd_proxy = rospy.ServiceProxy('recording_cmd',RecordingCmd)
        response = recording_cmd_proxy( 
                recording_cmd,
                fullpath_filename,
                recording_duration,
                )
        proxy_error_message = ''
    except rospy.ServiceException, e:
        proxy_error_message = str(e)
        recording_flag = 0

    db.set('recording_flag',recording_flag)
    recording_button_text = display_tools.get_recording_button_text(recording_flag)
    obj_response.html('#recording_button',recording_button_text)
    obj_response.html('#avi_exists_message','')
    obj_response.html('#proxy_error_message', proxy_error_message)
    return

# -----------------------------------------------------------------------------
if __name__ == '__main__': 

    try:
        server = sys.argv[1]
    except IndexError:
        server = 'tornado'

    if server == 'debug':
        print ' * using debug server'
        app.debug = True 
        app.run()
    elif server == 'builtin':
        print ' * using builtin server'
        app.run(host='0.0.0.0')
    elif server == 'tornado':
        print ' * using tornado server'
        http_server = HTTPServer(WSGIContainer(app))
        http_server.listen(5000)
        IOLoop.instance().start()
    else: 
        raise ValueError, 'uknown server option %s'%(server,)
