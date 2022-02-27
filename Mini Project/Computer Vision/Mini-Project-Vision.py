####################################################################### 
# NAME:     Johnathan Evans
# CLASS:    EENG-350
# TITLE:        Mini-Project
# FUNCTION:     Find a marker on the camera and return what quadrant it is in
# RUNNING:   Use Pi, press go
# RESOURCE:     https://docs.opencv.org/4.1.0/d6/d00/tutorial_py_root.html
# PURPOSE:   openCV resource
######################################################################


import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
print(cv.__version__)
# imports 

width=640
height=368
# let the camera calibrate and make a white ballance
camera = PiCamera(resolution=(640, 368), framerate=30)
rawCapture = PiRGBArray(camera)
# Set ISO to the desired value
camera.iso = 100
# Wait for the automatic gain control to settle
time.sleep(2)
# Now fix the values
#camera.shutter_speed = camera.exposure_speed
#camera.exposure_mode = 'off'
#g = camera.awb_gains
#camera.awb_mode = 'off'
#camera.awb_gains = g


# Callibrate color
input('Hold monochrome marker in front of camera and press enter')

camera.capture(rawCapture, format="bgr")
image = rawCapture.array
image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
image = cv.blur(image,(10,10))#blur to smooth
px = image[int(height/2),int(width/2)]
print(px)
lower_mask = np.array([px[0]-10,0,0])
upper_mask = np.array([px[0]+10,255,255])


#print(colorMedian)

#cv.imshow('', image)
#cv.waitKey(0)

camera.close()
#-----------------------------------
# prepare video capture
cap = cv.VideoCapture(0)
#currentWB = cap.get(cv.CAP_PROP_WHITE_BALANCE_BLUE_U)
# test camera function
if not cap.isOpened():
    print("Cannot open camera")
    exit()
# run viideo with cap open
while True:
    #cap.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, currentWB)
    # Capture frame-by-frame
    ret, img = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
   
    #Image operations below
    #resize image for easier processing
    width = int(img.shape[1] * 1/2)
    height = int(img.shape[0] * 1/2)
    imgtuple = (width,height)
    img = cv.resize(img, imgtuple)
    # transform to hsv for color analysis
    imghsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    
    

    mask = cv.inRange(imghsv, lower_mask, upper_mask)

    #make mask
    #mask = cv.cvtColor(mask, cv.COLOR_HSV2BGR)
    #below code is to clean the image

    kernel = np.ones((5,5),np.uint8)#create kernel

    mask = cv.erode(mask,kernel,iterations=1)#dialate to fill in gap
    mask = cv.erode(mask,kernel,iterations=1)#dialate to fill in gap
    mask = cv.blur(mask,(5,5))#blur to smooth
    mask = cv.blur(mask,(5,5))#blur to smooth
    #find the average location
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        print('No markers found')
    else:
    # ============================= TODO =============================
    # view location to find quardinate
    # ================================================================
        #find angle and return quadrant (-1 for no color found)
        locx = nonzero[1].mean()
        locy = nonzero[0].mean()
        ret_quad = -1
        if locx>width/2 and locy<height/2:
            ret_quad = 0
        elif locx<width/2 and locy<height/2:
            ret_quad = 1
        elif locx<width/2 and locy>height/2:
            ret_quad = 2
        elif locx>width/2 and locy>height/2:
            ret_quad = 3
        else:
            ret_quad = -1
        #====================================DAVID HERE IS YOUR RETURN
        print(ret_quad)
        #==================================== -1 IS FOR NO DETECTED COLOR
    
        
        
    
    # Display the resulting frame
    cv.imshow('frame', mask)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()    
