import cv

w = cv.CreateVideoWriter('mytest.avi', cv.CV_FOURCC('M','J','P','G'), 5.0, (100,100))
print cv.__file__
