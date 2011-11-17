import math
import config
import os.path

def update_trial_values(trial_values, form):
    """ 
    Get trial values from request form - this is kind of explicit could be better
    """
    try:
        t_min = int(form['duration_min'])
    except ValueError:
        t_min = 0
    try:
        t_sec = int(form['duration_sec'])
    except ValueError:
        t_sec = 0
    t_min, t_sec = normalize_min_sec(t_min, t_sec)
    recording_duration = (t_min, t_sec)

    t_min = int(form['pulse_start_time_min'])
    t_sec = int(form['pulse_start_time_sec'])
    t_min, t_sec = normalize_min_sec(t_min, t_sec)
    pulse_start_time = (t_min, t_sec)

    number_pulses = int(form['number_pulses'])
    pulse_period = int(form['pulse_period'])
    pulse_duty_cycle = int(form['pulse_duty_cycle'])
    pulse_power = int(form['pulse_power'])

    # Clamp % values at 100%
    if pulse_duty_cycle > 100:
        pulse_duty_cycle = 100
    if pulse_power > 100:
        pulse_power = 100

    # Update trial values, data base and set flag
    trial_values['recording_duration'] = recording_duration 
    trial_values['number_pulses'] = number_pulses
    trial_values['pulse_start_time'] = pulse_start_time
    trial_values['pulse_duty_cycle'] = pulse_duty_cycle
    trial_values['pulse_power'] = pulse_power
    trial_values['pulse_period'] = pulse_period
    return trial_values

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

def normalize_min_sec(t_min,t_sec):
    """
    Normalize minute and seconds
    """
    t = 60.0*t_min + t_sec
    t_min_norm = int(math.floor(t/60.0))
    t_sec_norm = int(t - 60*t_min_norm)
    return t_min_norm, t_sec_norm

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

