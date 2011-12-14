"""
Configuration information for lia video recording system
"""
import roslib
roslib.load_manifest('lia_web_interface')
from lia_config import *
import os

page_header = 'LIA - Video Acquisition System'
server_port = 5000

tab_dict = {
        'capture': {
            'tab':'tab1',
            'label': 'Capture Control',
            },
        'fullsize_view': {
            'tab': 'tab2',
            'label': 'Fullsize View',
            },
        'trial_settings': {
            'tab': 'tab3',
            'label': 'Trial Settings',
            },
        'logging': {
            'tab': 'tab4',
            'label': 'Log Settings',
            },
        'info': {
            'tab': 'tab5',
            'label': 'System Info',
            },
        'docs': { 
            'tab': 'tab6',
            'label': 'Manual',
            },
        }

tab_order = ['capture', 'fullsize_view', 'trial_settings', 'logging', 'info','docs']

time_units = 'hr:min:sec'

trial_values_info = [ 
        {
            'name'      : 'Recording Duration', 
            'tag'       : 'recording_duration', 
            'value'     : (0,10,0), 
            'units'     : time_units,
            'type'      : 'time',
            },
        {   
            'name'      : 'Pulse Start Time',
            'tag'       : 'pulse_start_time',
            'value'     : (0,3,0), 
            'units'     : time_units,
            'type'      : 'time',
            },
        {
            'name'      : 'Pulse High Time',
            'tag'       : 'pulse_high_time',
            'value'     : (0,1,0), 
            'units'     : time_units,
            'type'      : 'time',
            },
        {
            'name'      : 'Pulse Period',
            'tag'       : 'pulse_period',
            'value'     : (0,0,30), 
            'units'     : time_units,
            'type'      : 'time',
            },
        {
            'name'       : 'Number of Pulses',
            'tag'        : 'number_of_pulses',
            'value'      : 2,
            'units'      : '',
            'type'      : 'number',
            },
        {
            'name'      : 'Pulse Current (Ch A)',
            'tag'       : 'pulse_current',
            'value'     : 100,
            'units'     : 'mA',
            'type'      : 'number'
            },
        ]

log_values_info = [ 
        {
            'name': 'Data Directory',
            'tag': 'data_directory',
            'value': os.path.join(os.environ['HOME'],'lia_data'),
            'type': 'string_readonly',
            'comment': '',
            },
        { 
            'name': 'Movie File', 
            'tag': 'movie_file',
            'value': 'default.avi', 
            'type': 'string',
            'comment': '(.avi) extension added automatically',
            },
        {
            'name': 'Overwrite',
            'tag': 'overwrite',
            'value': 'yes',
            'type': 'checkbox',
            'comment': '',
            },
        {
            'name': 'Append Datetime',
            'tag': 'append_datetime',
            'value': 'yes', 
            'type': 'checkbox',
            'comment': '',
            },
        {
            'name': 'Settings File Suffix',
            'tag': 'settings_file_suffix',
            'value': '_settings', 
            'type': 'string_readonly',
            'comment': '', 
            },
        {
            'name': 'Timing File Suffix',
            'tag': 'timing_file_suffix',
            'value': '_timing', 
            'type': 'string_readonly',
            },

        ]

# Image sizes for different views
fullsize_tab_image = {
        'width'  : 1280,
        'height' : 1024,
        }

capture_tab_image = {
        'width'  : fullsize_tab_image['width']/2,
        'height' : fullsize_tab_image['height']/2,
        }

# Fullsize tab scale options
fullsize_scale_options = [str(0.1*x) for x in range(10,0,-1)]

# Image quality option
mjpeg_quality = 80 

# For development only
username = 'admin'
password = 't3st'

# Temporary debug flag 
yale_temp_debug = False 




