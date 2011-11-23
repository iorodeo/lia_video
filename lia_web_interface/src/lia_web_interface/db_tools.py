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
    db.set('recording_flag', 0)

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
