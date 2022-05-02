
import serial
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import math

crossTolerance=0.60

def is_cross(mask):
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        tapeAngle = TAPE_NOT_FOUND_SET #tape not found flag 
    else:
        maxTolerance = max(nonzero[1])
        minTolerance = min(nonzero[1])
        #print(maxTolerance)
        #print(minTolerance)
        #print(crossTolerance*frameSizeX)
        if (maxTolerance-minTolerance>(crossTolerance*cols/1.05)):
            return True
    return False

def crop_out_noise(img,frameSizeY,frameSizeX):
    crop = img[int(rows-frameSizeY):int(rows*(0.93)), int(cols/2-frameSizeX/2):int(cols/2+frameSizeX/2)]
    #0.95 is to eliminate the strip of rgb error on the open cv
    return crop

def take_picture(cropY,cropX):
    rawCapture = PiRGBArray(camera)
    try:
        camera.capture(rawCapture, format="bgr")
        img = rawCapture.array
    except:
        print("Failed Image Capture")
    #resize for quicker processing
    imgtuple = (cols,rows)
    img = cv.resize(img, imgtuple)
    img = crop_out_noise(img,int(rows/cropY),int(cols/cropX))
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
    
    return mask#set a global variable for functions to reference 

#import matplotlib.pyplot as plt
###print(cv.__version__)

def rad(deg):#quick finction to convert rads to degrees
    return (deg * math.pi) / 180


def deg(rad):#quick finction to convert degrees to rads
    return (rad * 180) / math.pi



MIN_CONSECUTIVE_SAME_ANGLE_COUNT = 1    # Number of concurrent angle readings that must be same before returning tape angle
blueHSV = 103#100 or 95 depending on type of tape
deltaHSV = 11
cols = int(672)
rows = int(496)
horizontalFOV=rad(59)#FOV
verticalFOV=rad(43.85)#FOV
cameraHeightIn=7#height of camera from ground
cameraToWheelOffsetIn=abs(cameraHeightIn/math.tan(verticalFOV))#distance form the camera's closest viewto the wheels
crossTolerance = 0.60 #percentage of horizontal screen to scan for cross
showImg=True#decides if the frame is shown on the screen or not
camera = PiCamera()


# Wait until tape angle reported is the same for MIN_CONSECUTIVE_SAME_ANGLE_COUNT consecutive loops before returning angle
#    waitTime(1)
mask = take_picture(2.0,1.05)
#    show_img(mask)
    #--------------------------------------\/Image Measurment\/--------------------------------------
nonzero = np.nonzero(mask)


#find angle by lookingfor the lowest vertical pointand taking the coresponding horizontal index
#location = nonzero[1][np.where(nonzero[0]==(max(nonzero[0])))]
#print(cols)
#print(location.mean())
#phi = (deg(horizontalFOV)/2)*(location.mean()-cols/2)/(cols/2)
##=====================Angle Output=======================
#if phi>=0:
#    tapeAngle = int(phi+0.5)#0.5 for more accurate integer rounding
#else:
#    tapeAngle = int(phi-0.5)#0.5 for more accurate integer rounding
#
#print(is_cross(mask))
cv.imshow('frame', mask)
cv.waitKey()
