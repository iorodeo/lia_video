import math

def time2minsec(t):
    """
    Converts time in secs to minutes and seconds
    """
    t_min = int(math.floor(t/60.0))
    t_sec = int(math.floor(t - 60*t_min)) 
    return t_min, t_sec

def minsec2time(t_min,t_sec):
    """
    Converts minutes and seconds into time in seconds
    """
    t = 60.0*float(t_min) + float(t_sec)
    return t
