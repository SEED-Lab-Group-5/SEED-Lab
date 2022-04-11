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
    return (deg*math.pi)/180
def deg(rad):#quick finction to convert degrees to rads
    return (rad*180)/math.pi


blueHSV=110#100 or 95 dependingon type of tape
deltaHSV=10
cols = int(672)
rows = int(496)
horizontalFOV=rad(55.5)#FOV
verticalFOV=rad(44.1)#FOV
cameraHeightIn=7#height of camera from ground
desiredPerspectiveRangeIn=72
perspectiveRows = rows
perspectiveCols = cols
cameraToWheelOffsetIn=abs(cameraHeightIn/math.tan(verticalFOV/2))#distance form the camera's closest viewto the wheels
resizecols=abs(2*cameraToWheelOffsetIn*math.tan(horizontalFOV/2))#phisical viewing with at the corners of the camera
showImg=True#decides if the frame is shown on the screen or not

#init videocapture
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()


# transforms the image to see from a bird's eye view
def perspectiveShift(shapeImg):
    #print("resizing...")
    rows = shapeImg.shape[0]
    cols = shapeImg.shape[1]
    perspectiveResizeY=int((rows/2)*(1+2*math.atan(cameraHeightIn/desiredPerspectiveRangeIn)/verticalFOV))#the location to shift from for bird's eye view
    perspectiveResizeXLeft=int((cols/2)*(1-2*math.atan((resizecols/2)/desiredPerspectiveRangeIn)/horizontalFOV))#^
    perspectiveResizeXRight=int((cols/2)*(1+2*math.atan((resizecols/2)/desiredPerspectiveRangeIn)/horizontalFOV))#^
    finalCols = perspectiveResizeXRight-perspectiveResizeXLeft
    #finalRows = rows-perspectiveResizeY
    finalRows = int(finalCols*((desiredPerspectiveRangeIn-cameraToWheelOffsetIn)/resizecols))
    #print("Streatching points:")
    #print(str(perspectiveResizeXLeft)+","+str(perspectiveResizeY)+" to 0,0")
    #print(str(perspectiveResizeXRight)+","+str(perspectiveResizeY)+" to "+str(cols)+",0")
    pts1 = np.float32([[0,rows],[cols,rows],[perspectiveResizeXRight,perspectiveResizeY],[perspectiveResizeXLeft,perspectiveResizeY]])
    pts2 = np.float32([[0,rows],[cols,rows],[cols,0],[0,0]])
    
    M = cv.getPerspectiveTransform(pts1,pts2)
    dst = cv.warpPerspective(shapeImg,M,(cols,rows))
    perspectiveRows = finalRows*2
    perspectiveCols = finalCols*2
    ret = cv.resize(dst,(perspectiveCols,perspectiveRows))
    
    return ret

#takes a picture
def take_picture():
    camera = PiCamera(resolution=(cols, rows), framerate=30)
    rawCapture = PiRGBArray(camera)
    # allow the camera to warmup
    time.sleep(0.1)
    # grab an image from the camera
    print("Capturing")
    try:
        camera.capture(rawCapture, format="bgr")
        return rawCapture.array
    except:
        print("Failed")

#halfs the image size for processing (may not use)
def size_down(img):
    print("Sizing Down...")
    cols = int(img.shape[1] * 1/2)
    rows = int(img.shape[0] * 1/2)
    imgtuple = (cols,rows)
    img = cv.resize(img, imgtuple)
    return img

#outputs a mask of the bird's eye view 
def birds_eye_(img):
    #resize for quicker processing  
    imgtuple = (cols,rows)
    img = cv.resize(img, imgtuple)

    #=========================================\/Image Processing Block\/=========================================
    #--------------------------------------\/Image Masking\/--------------------------------------
    imghsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    lower = np.array([blueHSV-deltaHSV,50,50])
    upper = np.array([blueHSV+deltaHSV,255,255])
    mask = cv.inRange(imghsv, lower, upper)
    #--------------------------------------/\Image Masking/\--------------------------------------


    #--------------------------------------\/Image Modification\/--------------------------------------
    out=perspectiveShift(mask)
    #--------------------------------------/\Image Modification/\--------------------------------------


    #--------------------------------------\/Image Cleaning\/--------------------------------------

    #--------------------------------------/\Image Cleaning/\--------------------------------------
    #=========================================/\Image Processing Block/\=========================================
    return out


#will return the length of a line the robot is aligned with and at the base of
def measure_line(img):
    mask = birds_eye_(img)
    
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        print('No markers found')#tape not found flag = -126
    else:
        #find the length of a straight line at its base to the end
        topOfLine=min(nonzero[0])/2
        #since this perspective has pixels proportional to the physical distances, a simple constant can be used to determine the distance 
        lengthToLine = int((desiredPerspectiveRangeIn-cameraToWheelOffsetIn)*((perspectiveRows-topOfLine)/perspectiveRows)+cameraToWheelOffsetIn+0.5)
        #=====================Distance Output=======================
        print(str(lengthToLine)+" in")
        #print(str(topOfLine)+" of "+str(perspectiveRows))
    
    #--------------------------------------/\Image Measurment/\--------------------------------------

    return mask

#will return the distance to the start of a line the robot is aligned with
def measure_distance_to_start(img):
    mask = birds_eye_(img)
    
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        print('No markers found')#tape not found flag = -126
    else:
        #find the length of a straight line at its base to the end
        bottomOfLine=max(nonzero[0])/2
        #since this perspective has pixels proportional to the physical distances, a simple constant can be used to determine the distance 
        lengthToLine = int((desiredPerspectiveRangeIn-cameraToWheelOffsetIn)*((perspectiveRows-bottomOfLine)/perspectiveRows)+cameraToWheelOffsetIn+0.5)
        #=====================Distance Output=======================
        print(str(lengthToLine)+" in")
        #print(str(bottomOfLine)+" of "+str(perspectiveRows))
    
    #--------------------------------------/\Image Measurment/\--------------------------------------

    return mask
 
#returns a masked wide view for angle measurment
def wide_angle_(img):
    
    #resize for quicker processing  
    imgtuple = (cols,rows)
    img = cv.resize(img, imgtuple)

    #=========================================\/Image Processing Block\/=========================================
    #--------------------------------------\/Image Masking\/--------------------------------------
    imghsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    lower = np.array([blueHSV-deltaHSV,50,50])
    upper = np.array([blueHSV+deltaHSV,255,255])
    mask = cv.inRange(imghsv, lower, upper)
    #--------------------------------------/\Image Masking/\--------------------------------------


    #--------------------------------------\/Image Modification\/--------------------------------------
    #--------------------------------------/\Image Modification/\--------------------------------------


    #--------------------------------------\/Image Cleaning\/--------------------------------------
    kernel = np.ones((5,5),np.uint8)#create kernel
    mask = cv.erode(mask,kernel,iterations=2)#dialate to fill in gap
    mask = cv.blur(mask,(5,5))#blur to smooth
    mask = cv.blur(mask,(5,5))#blur to smooth
    out = mask[int(rows/2):(rows-1), 0:(cols-1)]#crop out the top half
    #--------------------------------------/\Image Cleaning/\--------------------------------------
    #=========================================/\Image Processing Block/\=========================================
    return out


#locates and points to a tape that may be outside of the screen
def measure_angle(img):
    mask = wide_angle_(img)
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        print('No markers found')#tape not found flag = -126
    else:
        #find angle
        location = nonzero[1].mean()
        phi = (deg(horizontalFOV)/2)*(location-cols/2)/(cols/2)
        #=====================Angle Output=======================
        if phi>=0:
            print(int(phi+0.5))#0.5 for more accurate integer rounding
        else:
            print(int(phi-0.5))#0.5 for more accurate integer rounding
    #--------------------------------------/\Image Measurment/\--------------------------------------
    return mask

#returns the angle to the lowest point of blue tape in the image
def measure_angle_to_start(img):
    mask = wide_angle_(img)
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        print('No markers found')#tape not found flag = -126
    else:
        
        #find angle by lookingfor the lowest vertical pointand taking the coresponding horizontal index
        location = nonzero[1][np.where(nonzero[0]==(max(nonzero[0])))]
        #print(cols)
        #print(location.mean())
        phi = (deg(horizontalFOV)/2)*(location.mean()-cols/2)/(cols/2)
        #=====================Angle Output=======================
        if phi>=0:
            print(int(phi+0.5))#0.5 for more accurate integer rounding
        else:
            print(int(phi-0.5))#0.5 for more accurate integer rounding
    #--------------------------------------/\Image Measurment/\--------------------------------------
    return mask
    


def show_img(img2show):
    cv.imshow('frame', img2show)
    

# The Finite state Loop - primarily for testing

while True:
    state = "angle"
    
    # Capture frame-by-frame
    ret, img = cap.read()
    imgRGB = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    #--------------------------------------\/Finite State\/--------------------------------------
    if   state == "angle":
        out=measure_angle_to_start(img)
    else:
        print("ERROR, enter angle bird or pass")
        
    #--------------------------------------/\Finite State/\--------------------------------------  
    
    
    # Display the resulting frame
    if showImg:
        cv.imshow('frame', out)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()    

    
