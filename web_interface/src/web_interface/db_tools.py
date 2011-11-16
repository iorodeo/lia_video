"""
Some tools for interacting with the Redis database
"""
import cPickle as pickle

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
