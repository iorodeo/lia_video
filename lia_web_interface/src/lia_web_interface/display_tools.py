"""
A collection of service functions usefull for createing the web display.
"""

def get_log_display(log_values, show_file_suffix=False):
    """
    Creates a display list for a given set of log values
    """
    log_display = [ 
            {
                'name': 'Data Directory',
                'value': log_values['data_directory'],
                'class': 'c2',
                },
            { 
                'name': 'Movie File', 
                'value': log_values['movie_file'],
                'class': 'c1',
                },
            {
                'name': 'Overwrite',
                'value': log_values['overwrite'],
                'class': 'c2',
                },
            {
                'name': 'Append Datetime',
                'value': log_values['append_datetime'],
                'class': 'c1',
                },
            ]
    if show_file_suffix:
        suffix_display = [
            {
                'name': 'Settings File Suffix',
                'value': log_values['settings_file_suffix'],
                'class': 'c2',
                },
            {
                'name': 'Timing File Suffix',
                'value': log_values['timing_file_suffix'],
                'class': 'c1',
                },
            ]
        log_display.extend(suffix_display)
    return log_display

def get_trial_display(trial_values): 
    """
    Creates a display list for a given set of trial values.
    """
    trial_display = [ 
            {
                'name': 'Recording Duration', 
                'value': '%d min %d sec'%trial_values['recording_duration'],
                'class': 'c2',
                },
            {
                'name': 'Number of Pulses',
                'value': '%d'%(trial_values['number_pulses'],),
                'class': 'c1',
                },
            {   
                'name': 'Pulse Start Time',
                'value': '%d min %d sec'%trial_values['pulse_start_time'],
                'class': 'c2',
                },
            {
                'name': 'Pulse Period',
                'value': '%d sec'%(trial_values['pulse_period'],),
                'class': 'c1',
                },
            {
                'name': 'Pulse Duty Cycle',
                'value': '%d%%'%(trial_values['pulse_duty_cycle'],),
                'class': 'c2',
                },
            {
                'name': 'Pulse Power',
                'value': '%d%%'%(trial_values['pulse_power'],),
                'class': 'c1',
                }
            ]
    return trial_display

def get_recording_button_text(recording_flag):
    """
    Returns the recording button text given the recording flag
    """
    if recording_flag == 0:
        recording_button_text = 'Start Recording'
    else:
        recording_button_text = 'Stop Recording'
    return recording_button_text


def get_colored_list(input_list, color_vals=('c1', 'c2')):
    output_list = []
    for i, data in enumerate(input_list):
        color = color_vals[i%2]
        if type(data) in (list,tuple):
            colored_data = list(data) + [color]
        else:
            colored_data = (data, color)
        output_list.append(colored_data)
    return output_list



