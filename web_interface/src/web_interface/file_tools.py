"""
A collection of tools useful for dealing with files and directories.
"""
import os
import os.path

def check_dir(dirname):
    """
    Checks for directory and creates it if it doesn't exist.
    """
    if os.path.isdir(dirname):
        return True
    else:
        os.mkdir(dirname)

#------------------------------------------------------------------------------
if __name__ == '__main__':

    check_for_dir('/home/wbd')
