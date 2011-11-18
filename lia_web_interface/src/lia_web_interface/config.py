"""
Configuration information for lia video recording system
"""
import roslib
roslib.load_manifest('lia_web_interface')
from lia_config import *
import os

# User interface -----------------------------------------------------
page_header = 'LIA - Video Acquisition System'

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

trial_values_default = {
        'recording_duration': (10,0),
        'number_pulses': 2,
        'pulse_start_time': (3,0),
        'pulse_period': 60,
        'pulse_duty_cycle': 50,
        'pulse_power': 50,
        }

log_values_default = {
        'data_directory': os.path.join(os.environ['HOME'],'lia_data'),
        'movie_file': 'default.avi',
        'overwrite': 'yes',
        'append_datetime': 'yes',
        'settings_file_suffix': '_settings',
        'timing_file_suffix': '_timing',
        }

# Image sizes for different views
fullsize_tab_image = {
        'width': 1280,
        'height': 1024,
        }

capture_tab_image = {
        'width': fullsize_tab_image['width']/2,
        'height': fullsize_tab_image['height']/2,
        }

# Fullsize tab scale options
fullsize_scale_options = [str(0.1*x) for x in range(10,0,-1)]




