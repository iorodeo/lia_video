"""
A collection of tools useful for dealing with files and directories.
"""
import os
import os.path
import time

def add_datetime_suffix(filename):
    """
    Adds datetime suffix to the given filename.
    """
    datetime = time.ctime(time.time())
    datetime = datetime.replace(' ', '_')
    base, ext = os.path.splitext(filename)
    filename = '%s_%s%s'%(base,datetime,ext)
    return filename

def is_existing_avi(data_dir, filename):
    """
    Check if given file is an existing file in the data directory
    """
    avi_files = get_existing_avi(data_dir, info=False)
    if filename in avi_files:
        return True
    else:
        return False

def get_existing_avi(data_dir,info=False):
    """
    Returns a list of the existing avi movie files.
    """
    all_files = os.listdir(data_dir)
    avi_files = []
    
    for f in all_files:
        base, ext = os.path.splitext(f)
        if ext == '.avi':
            avi_files.append(f)
    avi_files.sort()

    if not info:
        return avi_files 
    else:
        avi_info_files = []
        for i, f in enumerate(avi_files):
            base,ext = os.path.splitext(f)
            f_path = os.path.join(data_dir,f)
            f_size = os.path.getsize(f_path)
            f_size = bytes_2_megabytes(f_size)
            f_size = '%1.1f'%(f_size,)
            f_time = os.path.getmtime(f_path)
            f_time = time.ctime(f_time)
            avi_info_files.append((f,f_size,f_time))
            #avi_info_files.append(('%s.mpg'%(base,),f_size,f_time))
        return avi_info_files

def delete_data_files(avi_files, data_directory):
    """
    Delete avi movie files in the list as well as any associated
    files - e.g., _settings.txt and _timing.txt files.
    """
    for f in avi_files:
        f_base, f_ext = os.path.splitext(f)
        if f_ext == '.avi':
            f_path = os.path.join(data_directory,f)
            try:
                os.remove(f_path)
            except OSError:
                pass
            
def check_dir(dirname):
    """
    Checks for directory and creates it if it doesn't exist.
    """
    if os.path.isdir(dirname):
        return True
    else:
        os.mkdir(dirname)

def bytes_2_megabytes(value):
    """
    Convert bytes to megabytes
    """
    return value/float(1048576)

#------------------------------------------------------------------------------
if __name__ == '__main__':

    check_for_dir('/home/wbd')
