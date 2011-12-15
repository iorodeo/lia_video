"""
Some tools for interacting with the Redis database
"""
import cPickle as pickle
import config
import config_tools

def init_db(db):
    trial_values_default = config_tools.get_defaults(config.trial_values_info)
    log_values_default = config_tools.get_defaults(config.log_values_info)
    set_dict(db,'trial_values',trial_values_default)
    set_dict(db,'log_values', log_values_default)
    if not db.exists('saved_trials'): 
        set_dict(db,'saved_trials', {})
    set_bool(db,'recording_flag',False)
    set_int(db,'ir_light_value',config.dflt_ir_light_value)

def set_dict(db,key,user_dict):
    """
    Stores a python dictionary in a redis key in the given database. It does
    this by pickling the dictionary.
    """
    dict_str = pickle.dumps(user_dict)
    return db.set(key,dict_str)

def get_dict(db,key):
    """
    Retrieves a dictionary, by key, which has been stored in the redis
    database.
    """
    dict_str = str(db.get(key))
    user_dict = pickle.loads(dict_str)
    return user_dict

def set_bool(db,key,value):
    """
    Sets boolean value in redis database.
    """
    if value: 
        value_str = str(True)
    else: 
        value_str = str(False)
    db.set(key,value_str)

def get_bool(db,key):
    """
    Get boolean values from redis database
    """
    value_str = str(db.get(key))
    if value_str == 'True':
        return True
    else:
        return False

def set_int(db,key,value):
    """
    Set integer value from redis database
    """
    value_str = str(value)
    db.set(key,value_str)

def get_int(db,key):
    """
    Get integer value from redis database
    """
    value_str = str(db.get(key))
    return int(value_str)
