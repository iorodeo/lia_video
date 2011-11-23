import flask
import functools

def check_login(fn):
    """
    Decorator for checking that users are logged in.
    """
    @functools.wraps(fn)
    def deco(*args, **kwargs):
        if not flask.session.get('logged_in'):
            return flask.redirect(flask.url_for('login'))
        else:
            return fn(*args,**kwargs)
    return deco
            



    
