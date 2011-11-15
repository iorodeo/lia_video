
def get_setup_display(setup_values): 
    """
    Creates a display list for a given set of setup values.
    """
    setup_display = [ 
            { 
                'name': 'Movie File', 
                'value': '%s, overwrite = %s'%(setup_values['movie_file'], setup_values['overwrite']),
                'class': 'c1',
                },
            {
                'name': 'Recording Duration', 
                'value': '%d min %d sec'%setup_values['recording_duration'],
                'class': 'c2',
                },
            {
                'name': 'Number of Pulses',
                'value': '%d'%(setup_values['number_pulses'],),
                'class': 'c1',
                },
            {   
                'name': 'Pulse Start Time',
                'value': '%d min %d sec'%setup_values['pulse_start_time'],
                'class': 'c2',
                },
            {
                'name': 'Pulse Period',
                'value': '%d sec'%(setup_values['pulse_period'],),
                'class': 'c1',
                },
            {
                'name': 'Pulse Duty Cycle',
                'value': '%d%%'%(setup_values['pulse_duty_cycle'],),
                'class': 'c2',
                },
            {
                'name': 'Pulse Power',
                'value': '%d%%'%(setup_values['pulse_power'],),
                'class': 'c1',
                }
            ]
    return setup_display

def get_recording_button_text(recording_flag):
    """
    Returns the recording button text given the recording flag
    """
    if recording_flag == 0:
        recording_button_text = 'Start Recording'
    else:
        recording_button_text = 'Stop Recording'
    return recording_button_text
