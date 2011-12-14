"""
A collection of service functions usefull for createing the web display.
"""
import config

def get_log_display(log_values, show_file_suffix=False, color_values=('c2','c1')):
    """
    Creates a display list for a given set of log values
    """
    log_display = []
    for i, item in enumerate(config.log_values_info):
        if ('suffix' in item['tag']) and (not show_file_suffix) :
            continue
        value = log_values[item['tag']]
        display = dict(item)
        display['value'] = value
        display['class'] = color_values[i%len(color_values)]
        log_display.append(display)
    return log_display

def get_trial_display(trial_values,color_values=('c2','c1')): 
    """
    Creates a display list for a given set of trial values.

    This is really kind of klunky ... 
    """
    trial_display = []
    for i, item in enumerate(config.trial_values_info):
        value = trial_values[item['tag']]
        display = dict(item)
        display['class'] = color_values[i%len(color_values)]
        display['value_num'] = value
        if item['type'] == 'time':
            display['value'] = time_display(value)
        elif item['type'] == 'number':
            display['value'] = number_display(value)
        else:
            raise ValueError, 'unknown trial values type: %s'%(item['type'],)
        trial_display.append(display)
    return trial_display


def time_display(values):
    return '%02d:%02d:%02d'%values

def number_display(value):
    return '%d'%(value,)

def get_time_labels():
    return zip((0,1,2),('hr','min','sec'))

def get_recording_button_text(recording_flag):
    """
    Returns the recording button text given the recording flag
    """
    if not recording_flag:
        recording_button_text = 'Start Recording'
    else:
        recording_button_text = 'Stop Recording'
    return recording_button_text

def get_colored_list(input_list, color_vals=('c2', 'c1')):
    """
    Get a list with every element sequentially colored using
    the given color valus. 
    """
    output_list = []
    for i, data in enumerate(input_list):
        color = color_vals[i%len(color_vals)]
        if type(data) in (list,tuple):
            colored_data = list(data) + [color]
        else:
            colored_data = (data, color)
        output_list.append(colored_data)
    return output_list



