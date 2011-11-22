import math
import os.path
import config
import iface_tools

def update_trial_values(trial_values, form):
    """ 
    Get trial values from request form - this is kind of explicit could be better
    """
    number_of_pulses = int(form['number_of_pulses'])
    recording_duration = get_time_values('recording_duration',form)
    pulse_start_time = get_time_values('pulse_start_time',form)
    pulse_period = get_time_values('pulse_period',form)
    pulse_high_time = get_time_values('pulse_high_time',form)
    pulse_power = int(form['pulse_power'])

    # Clamp power value at 100%
    if pulse_power > 100:
        pulse_power = 100

    # Update trial values, data base and set flag
    trial_values['recording_duration'] = recording_duration 
    trial_values['number_of_pulses'] = number_of_pulses
    trial_values['pulse_start_time'] = pulse_start_time
    trial_values['pulse_high_time'] = pulse_high_time
    trial_values['pulse_power'] = pulse_power
    trial_values['pulse_period'] = pulse_period
    return trial_values

def get_time_values(name,form):
    """
    Extract time values min and secs from the form.
    """
    try:
        t_hr = int(form['%s_hr'%(name,)])
    except ValueError:
        t_hr= 0
    try:
        t_min = int(form['%s_min'%(name,)])
    except ValueError:
        t_min = 0
    try:
        t_sec = int(form['%s_sec'%(name,)])
    except ValueError:
        t_sec = 0
    return  normalize_min_sec(t_hr, t_min, t_sec)


def update_log_values(log_values, form):
    """
    Gets the log values form the request form 
    """
    movie_file = str(form['movie_file'])
    if not movie_file:
        movie_file = config.log_values_default['movie_file']
    base, ext = os.path.splitext(movie_file)
    base = base.replace(' ','_')
    movie_file = '%s.avi'%(base,)

    log_values['movie_file'] = movie_file 
    if 'overwrite' in form:
        log_values['overwrite'] = 'yes'
    else:
        log_values['overwrite'] = 'no'

    if 'append_datetime' in form:
        log_values['append_datetime'] = 'yes'
    else:
        log_values['append_datetime'] = 'no'
    return log_values

def normalize_min_sec(t_hr, t_min,t_sec):
    """
    Normalize minute and seconds
    """
    t = 3600.0*t_hr + 60.0*t_min + t_sec
    t_hr_norm = int(math.floor(t/3600.0))
    t_rem = int(t - 3600*t_hr_norm)

    t_min_norm = int(math.floor(t_rem/60.0))
    t_sec_norm = int(t_rem - 60*t_min_norm)
    return t_hr_norm, t_min_norm, t_sec_norm

def get_selected_avi_files(form):
    """
    Gets the selected avi files from the form
    """
    avi_files = []
    for name in form:
        base, ext = os.path.splitext(str(name))
        if ext == '.avi':
            avi_files.append(str(name))
    return avi_files

def check_for_delete_tag(form):
    return find_prefix_match('delete_',form)

def check_for_load_tag(form):
    return find_prefix_match('load_',form)

def find_prefix_match(prefix,data):
    matching_key = None
    for key in data: 
        try: 
            key[0:len(prefix)] == prefix 
            matching_key = key[len(prefix):]
            break
        except IndexError:
            pass
    return matching_key

def find_all_prefix_match(prefix,data):
    matching = []
    for key in data: 
        try: 
            key[0:len(prefix)] == prefix 
            matching_key = key[len(prefix):]
            matching.append(matching_key)
        except IndexError:
            pass
    return matching

def get_base_kwargs():
    """
    Basic set of keyword consants for use in calls to render_template
    Counld move these to the database .... 
    """
    # Get hostaddr use ip address if possible
    try:
        hostaddr= iface_tools.get_ip_addr('eth0')
    except KeyError, NameError:
        print 'unable to get ip - using localhost'
        hostaddr = 'localhost'

    base_kwargs = {
            'page_header': config.page_header,
            'tab_dict': config.tab_dict,
            'tab_order': config.tab_order,
            'camera_topic': config.camera_topic,
            'progress_bar_topic': config.progress_bar_topic,
            'progress_message_topic': config.progress_message_topic,
            'fps_topic': config.fps_topic,
            'capture_tab_image': config.capture_tab_image,
            'fullsize_tab_image': config.fullsize_tab_image,
            'camera_mjpeg_port': config.camera_mjpeg_port,
            'progress_mjpeg_port': config.progress_mjpeg_port,
            'fps_mjpeg_port': config.fps_mjpeg_port,
            'hostaddr': hostaddr,
            }
    return base_kwargs

