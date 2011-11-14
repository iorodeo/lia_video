import flask
import redis
import atexit
import cPickle as pickle
import db_tools
import time_tools

TITLE = 'LIA - Video Acquisition System'

TAB_DICT = {
        'capture': {
            'tab':'tab1',
            'label': 'Capture',
            },
        'setup': {
            'tab': 'tab2',
            'label': 'Setup',
            },
        'info' : {
            'tab': 'tab3',
            'label': 'Info',
            },
        'docs': { 
            'tab': 'tab4',
            'label': 'Manual',
            },
        }

TAB_ORDER = ['capture', 'setup', 'info', 'docs']

BASE_KWARGS = {
        'title': TITLE,
        'tab_dict': TAB_DICT,
        'tab_order': TAB_ORDER,
        'camera': '/image_raw',
        'progress_bar': '/image_progress_bar',
        'progress_message': 'image_progress_message',
        }

SETUP_DEFAULTS = {
        'movie_file': 'default.avi',
        'overwrite': 'yes',
        'recording_duration': (10,0),
        'number_pulses': 2,
        'pulse_start_time': (3,0),
        'pulse_period': 60,
        'pulse_duty_cycle': 50,
        'pulse_power': 50,
        }

app = flask.Flask(__name__)
db = redis.Redis('localhost', db=10)
db_tools.set_dict(db,'setup_values',SETUP_DEFAULTS)
db.set('recording', 0)

@app.route('/')
def index():
    return flask.redirect(flask.url_for('capture'))

@app.route('/capture')
def capture():
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = TAB_DICT['capture']['tab']
    return flask.render_template('capture.html',**kwargs)

@app.route('/setup',methods=['GET','POST'])
def setup():
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = TAB_DICT['setup']['tab']
    setup_values = db_tools.get_dict(db,'setup_values')
    kwargs.update(setup_values)
    return flask.render_template('setup.html',**kwargs)

@app.route('/info')
def info():
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = TAB_DICT['info']['tab'] 
    return flask.render_template('info.html',**kwargs)

@app.route('/docs')
def docs():
    kwargs = dict(BASE_KWARGS)
    kwargs['current_tab'] = TAB_DICT['docs']['tab'] 
    return flask.render_template('docs.html',**kwargs)

def cleanup():
    for k in db.keys('*'):
        db.delete(k)
atexit.register(cleanup)


# -----------------------------------------------------------------------------
if __name__ == '__main__': 
    app.debug = True
    app.run()
