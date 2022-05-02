####################################################################### 
# NAME:     Johnathan Evans
# CLASS:    EENG-350
# TITLE:        Demo-1
# FUNCTION:     Assist a robot in moving by supplying an angle and distance
# RUNNING:   Use Pi, press go
# RESOURCE:     https://docs.opencv.org/4.1.0/d6/d00/tutorial_py_root.html
# PURPOSE:   openCV resource
#######################################################################

import serial
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math

#import matplotlib.pyplot as plt
print(cv.__version__)


def rad(deg):#quick finction to convert rads to degrees
    return (deg * math.pi) / 180
def deg(rad):#quick finction to convert degrees to rads
    return (rad * 180) / math.pi


MIN_CONSECUTIVE_SAME_ANGLE_COUNT = 1    # Number of concurrent angle readings that must be same before returning tape angle
blueHSV = 103#100 or 95 depending on type of tape
deltaHSV = 10
cols = int(672)
rows = int(496)
horizontalFOV=rad(59)#FOV
verticalFOV=rad(43.85)#FOV
frameSizeX=int(cols/1.0)
frameSizeY=int(rows/4.0)
cameraHeightIn=7#height of camera from ground
cameraToWheelOffsetIn=abs(cameraHeightIn/math.tan(verticalFOV))#distance form the camera's closest viewto the wheels
crossTolerance = 0.90 #percentage of horizontal screen to scan for cross
showImg=True#decides if the frame is shown on the screen or not
camera = PiCamera()


###############################################################
#####Detect Corners
###############################################################




def waitTime(secondsToWait):
    startTime = time.time()
    while time.time() < startTime + secondsToWait:
        pass
    return False

#################################################################
# Camera States
#################################################################




#takes pic and masks
def take_picture():
    rawCapture = PiRGBArray(camera)
    try:
        camera.capture(rawCapture, format="bgr")
        img = rawCapture.array
    except:
        print("Failed Image Capture")
    #resize for quicker processing
    imgtuple = (cols,rows)
    img = cv.resize(img, imgtuple)
    img = crop_out_noise(img)
    #image Processing
    #img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    imghsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    lower = np.array([blueHSV-deltaHSV,50,50])
    upper = np.array([blueHSV+deltaHSV,255,255])
    mask = cv.inRange(imghsv, lower, upper)
    kernel = np.ones((5,5),np.uint8)#create kernel
#     mask = cv.erode(mask,kernel,iterations=2)#dialate to fill in gap
#     mask = cv.blur(mask,(5,5))#blur to smooth
#     mask = cv.blur(mask,(5,5))#blur to smooth
    
    
    return img#set a global variable for functions to reference 


#alters the image size to isolate the floor in fromt of the robot
def crop_out_noise(img):
    crop = img[int(rows-frameSizeY):int(rows*(0.95)), int(cols/2-frameSizeX/2):int(cols/2+frameSizeX/2)]
    return crop

    


#locates and points to a tape that may be outside of the screen
def measure_angle():
    consecutiveSameAngleCount = 0
    prevTapeAngle = TAPE_NOT_FOUND_SET
    
    # Wait until tape angle reported is the same for MIN_CONSECUTIVE_SAME_ANGLE_COUNT consecutive loops before returning angle
    while (consecutiveSameAngleCount < MIN_CONSECUTIVE_SAME_ANGLE_COUNT):
        mask = wide_angle_()
#        cv.imshow('frame',mask)
        #--------------------------------------\/Image Measurment\/--------------------------------------
        nonzero = np.nonzero(mask)
        if len(nonzero[0])==0:
            tapeAngle = TAPE_NOT_FOUND_SET
    #        print('No markers found')#tape not found flag = -126
        else:
            #find angle
            location = nonzero[1].mean()
            phi = (deg(horizontalFOV)/2)*(location-cols/2)/(cols/2)
            #=====================Angle Output=======================
            if phi>=0:
                tapeAngle = int(phi+0.5)#0.5 for more accurate integer rounding
            else:
                tapeAngle = int(phi-0.5)#0.5 for more accurate integer rounding
        #--------------------------------------/\Image Measurment/\--------------------------------------
#        print(tapeAngle)
        if tapeAngle == prevTapeAngle:
            consecutiveSameAngleCount += 1
        
        prevTapeAngle = tapeAngle
            
    return tapeAngle


def show_img(img2show):
    cv.imshow('frame', img2show)
    cv.waitKey()
    
def is_cross(mask):
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        tapeAngle = -126 #tape not found flag = -126
    else:
        maxTolerance = max(nonzero[1])
        minTolerance = min(nonzero[1])
        print(maxTolerance)
        print(minTolerance)
        print(crossTolerance*frameSizeX)
        if (maxTolerance-minTolerance>(crossTolerance*frameSizeX)):
            return True
    return False


src = take_picture()
print(str(is_cross(src)))
cv.imshow("frame",src)
cv.waitKey()