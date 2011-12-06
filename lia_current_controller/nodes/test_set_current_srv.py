import roslib
roslib.load_manifest('lia_current_controller')
import rospy

from lia_services.srv import SetCurrentCmd

def set_current_srv(chan,state,value):
    """
    A simple command for testing the set current srv
    """
    rospy.wait_for_service('set_current')
    try:
        proxy = rospy.ServiceProxy('set_current',SetCurrentCmd)
        proxy(chan,state,value)
    except rospy.ServiceException, e:
        print 'service call failed: {0}'.format(str(e))


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import sys
    chan = sys.argv[1]
    state = sys.argv[2]
    value = int(sys.argv[3])
    set_current_srv(chan,state,value)
        
