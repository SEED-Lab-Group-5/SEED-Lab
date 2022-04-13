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
import smbus


#import matplotlib.pyplot as plt
print(cv.__version__)

def rad(deg):#quick finction to convert rads to degrees
    return (deg * math.pi) / 180


def deg(rad):#quick finction to convert degrees to rads
    return (rad * 180) / math.pi

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

# Flags and transmission codes
rotateComplete = False            # Indicates if robot is done rotating 
ROTATE_COMPLETE_SET = -127  # Transmitted from Arduino when flag is set

tapeFound = False             # Indicates if tape was found in the field of view
TAPE_NOT_FOUND_SET = -126       # Transmitted to Arduino when flag is set

atStart = False               # Indicates if robot has stopped moving forward
MOTION_COMPLETE_SET = -125    # Transmitted from Arduino when flag is set

startStateMachine = False
START_STATE_MACHINE_SET = -124


currentImg = None
currentMask = None
blueHSV = 95#100 or 95 dependingon type of tape
deltaHSV = 10
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


def waitTime(secondsToWait):
    startTime = time.time()
    while time.time() < startTime + secondsToWait:
        pass
    return
    
        


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
def birds_eye_(mask):
    #resize for quicker processing  
    imgtuple = (cols,rows)
    mask = cv.resize(mask, imgtuple)

    #=========================================\/Image Processing Block\/=========================================
    #--------------------------------------\/Image Masking\/--------------------------------------
    
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
    waitTime(1)
    mask = birds_eye_(img)
    
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        print('No markers found')#tape not found flag = -126
        return TAPE_NOT_FOUND_SET
    else:
        #find the length of a straight line at its base to the end
        topOfLine=min(nonzero[0])/2
        #since this perspective has pixels proportional to the physical distances, a simple constant can be used to determine the distance 
        lengthToLine = int((desiredPerspectiveRangeIn-cameraToWheelOffsetIn)*((perspectiveRows-topOfLine)/perspectiveRows)+cameraToWheelOffsetIn+0.5)
        #=====================Distance Output=======================
        print(str(lengthToLine)+" in")
        #print(str(topOfLine)+" of "+str(perspectiveRows))
    
    #--------------------------------------/\Image Measurment/\--------------------------------------

    currentMask = mask
    return lengthToLine


#will return the distance to the start of a line the robot is aligned with
def measure_distance_to_start(img):
    waitTime(1)
    mask = birds_eye_(img)
    
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        lengthToLine = TAPE_NOT_FOUND_SET
        print('No markers found')#tape not found flag = -126
    else:
        #find the length of a straight line at its base to the end
        bottomOfLine=max(nonzero[0])/2
        # since this perspective has pixels proportional to the physical distances, a simple constant can be used to determine the distance 
        lengthToLine = int((desiredPerspectiveRangeIn-cameraToWheelOffsetIn)*((perspectiveRows-bottomOfLine)/perspectiveRows)+cameraToWheelOffsetIn+0.5)
        #=====================Distance Output=======================
        print(str(lengthToLine)+" in")
#        print(str(bottomOfLine)+" of "+str(perspectiveRows))
    
    #--------------------------------------/\Image Measurment/\--------------------------------------

    currentMask = mask
    return lengthToLine
 
 
#returns a masked wide view for angle measurment
def wide_angle_(mask):
    
    #resize for quicker processing  
    imgtuple = (cols,rows)
    mask = cv.resize(mask, imgtuple)

    #=========================================\/Image Processing Block\/=========================================
    #--------------------------------------\/Image Masking\/--------------------------------------
   
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
    waitTime(1)
    mask = wide_angle_(img)
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        tapeAngle = -126
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
    currentMask = mask
    return tapeAngle


#returns the angle to the lowest point of blue tape in the image
def measure_angle_to_start(img):
    waitTime(1)
    mask = wide_angle_(img)
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        tapeAngle = -126 #tape not found flag = -126
    else:
        
        #find angle by lookingfor the lowest vertical pointand taking the coresponding horizontal index
        location = nonzero[1][np.where(nonzero[0]==(max(nonzero[0])))]
        #print(cols)
        #print(location.mean())
        phi = (deg(horizontalFOV)/2)*(location.mean()-cols/2)/(cols/2)
        #=====================Angle Output=======================
        if phi>=0:
            tapeAngle = int(phi+0.5)#0.5 for more accurate integer rounding
        else:
            tapeAngle = int(phi-0.5)#0.5 for more accurate integer rounding
    #--------------------------------------/\Image Measurment/\--------------------------------------
    currentMask = mask
    return tapeAngle


def show_img(img2show):
    cv.imshow('frame', img2show)
    

 


##################
# States
##################

# This is the start state. No code takes place here.
def state_start():
    print("state_start")
    return state_FOV_rotate


# This state primarily takes place on the arduino where the robot is rotated half its FOV clockwise. The flag
# rotateComplete will be set to -127 and sent to the Pi when the robot has finished rotating
def state_FOV_rotate():
    print("state_FOV_rotate")
    waitForRotation()    
    return state_find_tape    # Move to next state
    
    
# Find the angle to a line of tape in the camera's FOV
def state_find_tape():
    print("state_find_tape")
    
    
#    tapeAngle = TAPE_NOT_FOUND_SET    # NOTE: Placeholder - Use to test if tape was NOT found
#    tapeAngle = -22                     # NOTE: Placeholder - Use to test if tape WAS found
    
    # Use the camera to measure the angle to the tape
    tapeAngle = measure_angle(currentImg)
    
    # Implement find tape code here    
    if(tapeAngle == TAPE_NOT_FOUND_SET):
        writeData(TAPE_NOT_FOUND_SET)
        print("\tTape Not Found")
        return state_FOV_rotate
    else:
        writeData(tapeAngle)
        print("\ttapeAngle =", tapeAngle)
        return state_turn_to_start


# Rotate the robot to be in line with the start of the tape path
def state_turn_to_start():
    print("state_turn_to_start")
    waitForRotation()    
    return state_calc_dist_to_start


# Calculate the distance from the robot to the start of the tape path
def state_calc_dist_to_start():
    print("state_calc_dist_to_start") 

#    distToStart = 22  # NOTE: Placeholder
    
    distToStart = measure_distance_to_start(currentImg)
    print("\tDistance To Start =", distToStart)
    if distToStart == TAPE_NOT_FOUND_SET:
        return state_calc_dist_to_start
    # Send the distance to start to the Arduino
    writeData(distToStart)
    
    return state_drive_to_start
# Drive the robot to the start of the tape path
def state_drive_to_start():
    print("state_drive_to_start")
    waitForMotion()  
    return state_calc_path_angle


# Calculate the angle the robot needs to turn to to be in line with the tape path
def state_calc_path_angle():
    print("state_calc_path_angle")
    
#    angleToEnd = 10  # NOTE: Placeholder
    
    # Use the camera to measure the angle to the tape
    angleToEnd = measure_angle(currentImg)
    print("\tAngle To End =", angleToEnd)
       
    # Send the angle to end to the Arduino
    writeData(angleToEnd)
    
    return state_turn_inline_to_path


# Turn the robot to be in line with the tape path
def state_turn_inline_to_path():
    print("state_turn_inline_to_path")
    waitForRotation()
    return state_calc_dist_to_end


# Calculate the distance from the robot to the end of the tape path
def state_calc_dist_to_end():
      
#    print("state_calc_dist_to_end")
        

#    distToEnd = 36  # NOTE: Placeholder  
    distToEnd = measure_line(currentImg)
    if distToEnd == TAPE_NOT_FOUND_SET:
        return state_calc_dist_to_end
    print("\tDistance To End =", distToEnd)
    
    # Send the distance to start to the Arduino
    writeData(distToEnd)
    
    return state_drive_to_end


# Drive to the end of the tape path
def state_drive_to_end():
    print("state_drive_to_end")
    waitForMotion()
    return state_stop


# The end of the tape was reached. Exit the state machine
def state_stop():
    return None


def readData():
    data = bus.read_byte(address)
    if (data > 127):
        data = 256 - data
        data *= -1
    return data


def writeData(value):
    bus.write_byte(address, value)
    return -1


# Wait until the rotationComplete flag is transmitted from the Arduino (Robot has stopped rotating)
def waitForRotation():
    rotateComplete = False
    while rotateComplete != ROTATE_COMPLETE_SET:        
        rotateComplete = readData()    # Read from the I2C line and see if rotateComplete flag was set and transmitted       
        print("\trotateComplete =", rotateComplete)
        waitTime(1)  # Wait 1 second to reduce polling
        
# Wait until the motionComplete flag is transmitted from the Arduino (Robot has stopped driving forward)
def waitForMotion():
    motionComplete = False
    while motionComplete != MOTION_COMPLETE_SET:        
        motionComplete = readData()    # Read from the I2C line and see if motionComplete flag was set and transmitted       
        print("\tmotionComplete =", motionComplete)
        waitTime(1)    # Wait 1 second to reduce polling



# initalization
state = state_start # initial state

writeData(START_STATE_MACHINE_SET)

#init videocapture
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()


# The Finite state Loop
while state is not None: # Run until state is None   
    # Capture frame-by-frame
    ret, currentImg = cap.read()
    currentImg = cv.cvtColor(currentImg, cv.COLOR_BGR2RGB)
    imghsv = cv.cvtColor(currentImg, cv.COLOR_RGB2HSV)
    lower = np.array([blueHSV-deltaHSV,50,50])
    upper = np.array([blueHSV+deltaHSV,255,255])
    currentImg = cv.inRange(imghsv, lower, upper)
    
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # Our operations on the frame come here
    #--------------------------------------\/Finite State\/--------------------------------------
    new_state = state() # launch state machine
    state = new_state # update the next state
        
    #--------------------------------------/\Finite State/\--------------------------------------    
    
    # Display the resulting frame
    if showImg:        
        cv.imshow('frame', currentImg)
    if cv.waitKey(1) == ord('q'):
        break
    
print("Done with state machine")
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()   
