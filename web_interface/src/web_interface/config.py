page_header = 'LIA - Video Acquisition System'

tab_dict = {
        'capture': {
            'tab':'tab1',
            'label': 'Capture Control',
            },
        'setup': {
            'tab': 'tab2',
            'label': 'Change Settings',
            },
        'fullsize_view': {
            'tab': 'tab3',
            'label': 'Fullsize View',
            },
        'info': {
            'tab': 'tab4',
            'label': 'System Info',
            },
        'docs': { 
            'tab': 'tab5',
            'label': 'Manual',
            },
        }

tab_order = ['capture', 'setup', 'fullsize_view', 'info','docs']

setup_defaults = {
        'movie_file': 'default.avi',
        'overwrite': 'yes',
        'recording_duration': (10,0),
        'number_pulses': 2,
        'pulse_start_time': (3,0),
        'pulse_period': 60,
        'pulse_duty_cycle': 50,
        'pulse_power': 50,
        }

camera_mjpeg_port = 8080
progress_mjpeg_port = 8181

camera_topic = '/camera/image_throttle'
progress_bar_topic = '/image_progress_bar'
progress_message_topic = '/image_progress_message'

fullsize_tab_image = {
        'width': 1280,
        'height': 1024,
        }

capture_tab_image = {
        'width': fullsize_tab_image['width']/2,
        'height': fullsize_tab_image['height']/2,
        }

iface = 'eth0'





