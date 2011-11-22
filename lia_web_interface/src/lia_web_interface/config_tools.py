
def get_defaults(info):
    """
    Extracts the default values from the info structure.
    """
    trial_defaults = {}
    for item in info:
        trial_defaults[item['tag']] = item['value']
    return trial_defaults

