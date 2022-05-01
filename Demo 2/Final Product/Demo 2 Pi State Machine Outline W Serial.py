

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


# Set serial address and baud rate
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(1)   # Wait a moment to finalize the connection

# Flags and transmission codes

MOTION_COMPLETE_SET = -127  # Transmitted from Arduino when flag is set
START_STATE_MACHINE_SET = -126
MOTION_TYPE_ROTATE = -125
MOTION_TYPE_FORWARD = -124
TAPE_NOT_FOUND_SET = -123

motionComplete = False        # Indicates if robot is done rotating 
tapeFound = False             # Indicates if tape was found in the field of view
atStart = False               # Indicates if robot has stopped moving forward
startStateMachine = False


MIN_CONSECUTIVE_SAME_ANGLE_COUNT = 1    # Number of concurrent angle readings that must be same before returning tape angle


currentImg = None
dispImg = None
blueHSV = 103#100 or 95 depending on type of tape
deltaHSV = 7
cols = int(672)
rows = int(496)
horizontalFOV=rad(55.5)#FOV
verticalFOV=rad(45)#FOV
cameraHeightIn=6#height of camera from ground
desiredPerspectiveRangeIn=48
perspectiveRows = rows
perspectiveCols = cols
cameraToWheelOffsetIn=abs(cameraHeightIn/math.tan(verticalFOV/2))#distance form the camera's closest viewto the wheels
resizecols=abs(2*cameraToWheelOffsetIn*math.tan(horizontalFOV/2))#physical viewing with at the corners of the camera
showImg=False#decides if the frame is shown on the screen or not
camera = PiCamera(resolution=(cols, rows), framerate=30)


def saveImg(name,img):
    if (showImg):
        cv.imshow(name,img)
        cv.waitKey(0)

def waitTime(secondsToWait):
    startTime = time.time()
    while time.time() < startTime + secondsToWait:
        pass
    return False

#################################################################
# Camera States
#################################################################

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
    
    rawCapture = PiRGBArray(camera)
    # allow the camera to warmup
    time.sleep(0.1)
    # grab an image from the camera
    print("Capturing")
    try:
        camera.capture(rawCapture, format="bgr")
        img = rawCapture.array
    except:
        print("Failed")
        return None
        
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    imghsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
    lower = np.array([blueHSV-deltaHSV,50,50])
    upper = np.array([blueHSV+deltaHSV,255,255])
    mask = cv.inRange(imghsv, lower, upper)
    
    return mask #set a global variable for functions to reference 


#halfs the image size for processing (may not use)
def size_down(img):
    print("Sizing Down...")
    cols = int(img.shape[1] * 1/2)
    rows = int(img.shape[0] * 1/2)
    imgtuple = (cols,rows)
    img = cv.resize(img, imgtuple)
    return img


#outputs a mask of the bird's eye view 
def birds_eye_():
    #resize for quicker processing
    mask = take_picture()
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
    if tapeAngle!=TAPE_NOT_FOUND_SET:    
        saveImg('angle',mask)
    return tapeAngle


#will return the length of a line the robot is aligned with and at the base of
def measure_line():
#    waitTime(1)
    mask = birds_eye_()
    
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
#        print('No markers found')
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

    return lengthToLine


#will return the distance to the start of a line the robot is aligned with
def measure_distance_to_start():
#    waitTime(1)
    mask = birds_eye_()
#    cv.imshow('frame',mask)
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

    return lengthToLine
 
 
#returns a masked wide view for angle measurment
def wide_angle_():
    
    #resize for quicker processing
    mask = take_picture()
    imgtuple = (cols,rows)
    mask = cv.resize(mask, imgtuple)
    kernel = np.ones((5,5),np.uint8)#create kernel
    mask = cv.erode(mask,kernel,iterations=2)#dialate to fill in gap
    mask = cv.blur(mask,(5,5))#blur to smooth
    mask = cv.blur(mask,(5,5))#blur to smooth

    #=========================================\/Image Processing Block\/=========================================
    #--------------------------------------\/Image Masking\/--------------------------------------
   
    #--------------------------------------/\Image Masking/\--------------------------------------


    #--------------------------------------\/Image Modification\/--------------------------------------
    out = mask[int((rows/1.8)):(rows-1), 0:(cols-1)]#crop out the top half
    #--------------------------------------/\Image Modification/\--------------------------------------


    #--------------------------------------\/Image Cleaning\/--------------------------------------
    
    #--------------------------------------/\Image Cleaning/\--------------------------------------
    #=========================================/\Image Processing Block/\=========================================
    return out


#returns the angle to the lowest point of blue tape in the image
def measure_angle_to_start():
#    waitTime(1)
    mask = wide_angle_()
    #--------------------------------------\/Image Measurment\/--------------------------------------
    nonzero = np.nonzero(mask)
    if len(nonzero[0])==0:
        tapeAngle = TAPE_NOT_FOUND_SET #tape not found flag = -126
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
    return tapeAngle


def show_img(img2show):
    cv.imshow('frame', img2show)
    
#################################################################
# Data functions
#################################################################
def readData():
    
    data = 0
    # While there are still bytes to be read from the buffer
    while (ser.in_waiting > 0):
        # Read line from buffer and decode using utf-8 
        data = ser.readline().decode('utf-8').rstrip('\n')
        try:
            data = int(data)
        except:
            data = 0      
        print(data);
    return data


def writeData(motionType, motionMagnitude):
#    
    ser.write((str(motionType)+" "+str(motionMagnitude)+'\n').encode('utf-8'))
    ser.reset_output_buffer()
#    time.sleep(2)
    return -1


# Wait until the rotationComplete flag is transmitted from the Arduino (Robot has stopped rotating)
def waitForMotion():
    motionComplete = 0
    while motionComplete != MOTION_COMPLETE_SET:
        if ser.in_waiting > 0:
            motionComplete = readData()    # Read from the Serial line and see if motionComplete flag was set and transmitted       


#################################################################
# States
#################################################################

# This is the start state. No code takes place here.
def state_start():
    print("state_start")
    return state_FOV_rotate


# This state primarily takes place on the arduino where the robot is rotated half its FOV clockwise. The flag
# rotateComplete will be set to -127 and sent to the Pi when the robot has finished rotating
def state_FOV_rotate():
    waitTime(0.5)
    print("state_FOV_rotate")
    writeData(MOTION_TYPE_ROTATE, 30)
    waitForMotion()    
    return state_turn_to_start    # Move to next state


# Rotate the robot to be in line with the start of the tape path
def state_turn_to_start():
    print("state_turn_to_start")
    
    # Use the camera to measure the angle to the tape
    tapeAngle = measure_angle()
#    tapeAngle = TAPE_NOT_FOUND_SET    # NOTE: Placeholder - Use to test if tape was NOT found
#    tapeAngle = 10                     # NOTE: Placeholder - Use to test if tape WAS found
    
    # Implement find tape code here    
    if(tapeAngle == TAPE_NOT_FOUND_SET):        
        print("\tTape Not Found")
        return state_FOV_rotate
    else:
        print("\ttapeAngle =", tapeAngle)
        writeData(MOTION_TYPE_ROTATE, tapeAngle)
        waitForMotion()        
        return state_drive_to_start()    


# Calculate the distance from the robot to the start of the tape path then drive
def state_drive_to_start():
    print("state_drive_to_start()") 

    distToStart = 24  # NOTE: Placeholder   
#    distToStart = measure_distance_to_start()
    
    
    if distToStart == TAPE_NOT_FOUND_SET:
        print("\tTape Not Found")
        return state_drive_to_start
    
    else:
        print("\tDistance To Start =", distToStart)
        writeData(MOTION_TYPE_FORWARD, distToStart)    
        waitForMotion()  
        return state_turn_inline_to_path

    


# Calculate the angle the robot needs to turn to to be in line with the tape path
def state_turn_inline_to_path():
    print("state_turn_inline_to_path")
    
    # Use the camera to measure the angle to the tape
    angleToEnd = 6  # NOTE: Placeholder
#    angleToEnd = measure_angle()

    if angleToEnd == TAPE_NOT_FOUND_SET:
        print("\tTape Not Found")
        return state_turn_inline_to_path
    
    else:
        print("\tAngle To End =", angleToEnd)
        writeData(MOTION_TYPE_ROTATE, angleToEnd)    
        waitForMotion()  
        return state_drive_to_end


# Calculate the distance from the robot to the end of the tape path
def state_drive_to_end():
      
    print("state_drive_to_end")     

    distToEnd = 36  # NOTE: Placeholder  
#    distToEnd = measure_line()
    
    if distToEnd == TAPE_NOT_FOUND_SET:
        return state_drive_to_end
    
    else: 
        print("\tDistance To End =", distToEnd)
        writeData(MOTION_TYPE_FORWARD, distToEnd)    
        waitForMotion()  
        return state_stop
    

# The end of the tape was reached. Exit the state machine
def state_stop():
    ser.close()
    return None



# initalization
state = state_start # initial state


#init videocapture
cap = cv.VideoCapture(0)
if not cap.isOpened():
#    print("Cannot open camera")
    exit()


# The Finite state Loop
while state is not None: # Run until state is None   
    # Capture frame-by-frame
    
    # if frame is read correctly ret is True
    #currentImg = take_picture()
    
    # Our operations on the frame come here
    #waitTime(0.5)
    #--------------------------------------\/Finite State\/--------------------------------------
    #waitTime(0.2)
    new_state = state() # launch state machine
    state = new_state # update the next state
        
    #--------------------------------------/\Finite State/\--------------------------------------    
    cap.release()

    # Display the resulting frame
    
    
print("Done with state machine")
# When everything done, release the capture
cv.destroyAllWindows()





###this is for testing cv
#while True:
#    ret, img = cap.read()
#    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
#    imghsv = cv.cvtColor(img, cv.COLOR_RGB2HSV)
#    lower = np.array([blueHSV-deltaHSV,50,50])
#    upper = np.array([blueHSV+deltaHSV,255,255])
#    mask = cv.inRange(imghsv, lower, upper)
#    kernel = np.ones((5,5),np.uint8)#create kernel
#    mask = cv.erode(mask,kernel,iterations=3)#dialate to fill in gap
#    mask = cv.blur(mask,(5,5))#blur to smooth
#    mask = cv.blur(mask,(5,5))#blur to smooth
#    currentImg = mask
#    #print(measure_angle(mask))
#    #show = mask[int(rows/2):(rows-1), 0:(cols-1)]#crop out the top half
#    #print(measure_distance_to_start(mask))
#    #show = perspectiveShift(mask)
#    if showImg:        
#        cv.imshow('frame', currentImg)
#    if cv.waitKey(1) == ord('q'):
#        break





