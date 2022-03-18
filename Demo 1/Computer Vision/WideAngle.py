####################################################################### 
# NAME:     Johnathan Evans
# CLASS:    EENG-350
# TITLE:        Demo-1
# FUNCTION:     Report an angle with a 1" marker in front of the camera
# RUNNING:   Use Pi, press go
# RESOURCE:     https://docs.opencv.org/4.1.0/d6/d00/tutorial_py_root.html
# PURPOSE:   openCV resource


import serial
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math
#import time
print(cv.__version__)
def rad(deg):#quick finction to convert rads to degrees
    return (deg*math.pi)/180
def deg(rad):#quick finction to convert degrees to rads
    return (rad*180)/math.pi
#variables
blueHSV=105#100 or 95 dependingon type of tape
deltaHSV=10
cols = int(672)
rows = int(496)
horizontalFOV=rad(55.5)#FOV
verticalFOV=rad(44.1)#FOV



#init videocapture
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
    
    

while True:
    # Capture frame-by-frame
    ret, img = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    #resize for quicker processing  
    imgtuple = (cols,rows)
    img = cv.resize(img, imgtuple)
    


    #convert to hsv to create a mask
    imghsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    lower = np.array([blueHSV-deltaHSV,50,50])
    upper = np.array([blueHSV+deltaHSV,255,255])
    mask = cv.inRange(imghsv, lower, upper)

    #some image modifications
    kernel = np.ones((5,5),np.uint8)#create kernel
    mask = cv.erode(mask,kernel,iterations=2)#dialate to fill in gap
    mask = cv.blur(mask,(5,5))#blur to smooth
    mask = cv.blur(mask,(5,5))#blur to smooth
    crop = mask[int(rows/2):(rows-1), 0:(cols-1)]
    #find the average location
    nonzero = np.nonzero(crop)
    if len(nonzero[0])==0:
        print('No markers found')
    else:
        #find angle
        location = nonzero[1].mean()
        phi = (deg(horizontalFOV)/2)*(location-cols/2)/(cols/2)
        #=====================Angle Output=======================
        print(int(phi))
        
        
    
    # Display the resulting frame
    cv.imshow('frame', crop)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()    

