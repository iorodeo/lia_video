import cPickle as pickle

def set_dict(db,key,user_dict):
    dict_str = pickle.dumps(user_dict)
    return db.set(key,dict_str)

def get_dict(db,key):
    dict_str = str(db.get(key))
    user_dict = pickle.loads(dict_str)
    return user_dict
