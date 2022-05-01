

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
###print(cv.__version__)

def rad(deg):#quick finction to convert rads to degrees
    return (deg * math.pi) / 180


def deg(rad):#quick finction to convert degrees to rads
    return (rad * 180) / math.pi


# Set serial address and baud rate
ser = serial.Serial('/dev/ttyACM0', 57600)
time.sleep(1)   # Wait a moment to finalize the connection

# Flags and transmission codes

MOTION_COMPLETE_SET = -127  # Transmitted from Arduino when flag is set
TAPE_NOT_FOUND_SET = -123

motionComplete = False        # Indicates if robot is done rotating 
tapeFound = False             # Indicates if tape was found in the field of view
atStart = False               # Indicates if robot has stopped moving forward
startStateMachine = False


MIN_CONSECUTIVE_SAME_ANGLE_COUNT = 1    # Number of concurrent angle readings that must be same before returning tape angle
blueHSV = 103#100 or 95 depending on type of tape
deltaHSV = 11
cols = int(672)
rows = int(496)
horizontalFOV=rad(59)#FOV
verticalFOV=rad(43.85)#FOV
cameraHeightIn=7#height of camera from ground
cameraToWheelOffsetIn=abs(cameraHeightIn/math.tan(verticalFOV))#distance form the camera's closest viewto the wheels
crossTolerance = 0.90 #percentage of horizontal screen to scan for cross
showImg=True#decides if the frame is shown on the screen or not
camera = PiCamera()

totalPathRotation = 0;          # Tracks the total rotation the robot has made over the path
ROTATION_BEFORE_CROSS = 450     # Angle the robot must traverse before it is know it is near the end of the tape
FORWARD_STEP_DISTANCE = 10      # The distance the robot should drive forward after each turn
crossFound = False              # Flag to denote whether the cross at the end of the path was found
VIEW_SHIFT_ANGLE = 30            # The angle the robot should turn in the update angle state if the tape is not found
ANGLE_FUDGE_DELTA = 0           # The minimum angle the robot should attempt to turn to in the update angle state
ANGLE_AFTER_START = -45         # The angle the robot should rotate to after moving to the start

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
#
#measure_angle()
#returns angle from a small sliver of screen
#TODO: measure_start_angle()
#      returns angle from a longer range of view


#takes pic and masks
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


#alters the image size to isolate the floor in fromt of the robot
def crop_out_noise(img,frameSizeY,frameSizeX):
    crop = img[int(rows-frameSizeY):int(rows*(0.93)), int(cols/2-frameSizeX/2):int(cols/2+frameSizeX/2)]
    #0.95 is to eliminate the strip of rgb error on the open cv
    return crop

    


#locates and points to a tape that may be outside of the screen
def measure_angle():
    consecutiveSameAngleCount = 0
    prevTapeAngle = TAPE_NOT_FOUND_SET
    
    # Wait until tape angle reported is the same for MIN_CONSECUTIVE_SAME_ANGLE_COUNT consecutive loops before returning angle
    while (consecutiveSameAngleCount < MIN_CONSECUTIVE_SAME_ANGLE_COUNT):
        mask = take_picture(3.0,1.05)#Crop values for the picture
#        cv.imshow('frame',mask)
        #--------------------------------------\/Image Measurment\/--------------------------------------
        nonzero = np.nonzero(mask)
        if len(nonzero[0])==0:
            tapeAngle = TAPE_NOT_FOUND_SET
            print('No markers found')#tape not found flag
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
        print(tapeAngle)
        if tapeAngle == prevTapeAngle:
            consecutiveSameAngleCount += 1
        
        prevTapeAngle = tapeAngle
            
    return tapeAngle

def measure_start_angle():
    consecutiveSameAngleCount = 0
    prevTapeAngle = TAPE_NOT_FOUND_SET
    
    # Wait until tape angle reported is the same for MIN_CONSECUTIVE_SAME_ANGLE_COUNT consecutive loops before returning angle
    while (consecutiveSameAngleCount < MIN_CONSECUTIVE_SAME_ANGLE_COUNT):
#    waitTime(1)
        mask = take_picture(1.4,1.05)
    #    show_img(mask)
        #--------------------------------------\/Image Measurment\/--------------------------------------
        nonzero = np.nonzero(mask)
        if len(nonzero[0])==0:
            print("nonzero")
            tapeAngle = TAPE_NOT_FOUND_SET #tape not found flag
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
        tapeAngle = TAPE_NOT_FOUND_SET #tape not found flag 
    else:
        maxTolerance = max(nonzero[1])
        minTolerance = min(nonzero[1])
        #print(maxTolerance)
        #print(minTolerance)
        #print(crossTolerance*frameSizeX)
        if (maxTolerance-minTolerance>(crossTolerance*frameSizeX)):
            return True
    return False
    
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
        #print(data);
    return data


# Send the next controller values to the Arduino over serial
def writeMotionData(newAngle, newDistance):   
    ser.write((str(newAngle)+" "+str(newDistance)+'\n').encode('utf-8'))
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
    writeMotionData(30, 0)
    waitForMotion()    
    return state_turn_to_start    # Move to next state




# Rotate the robot to be in line with the start of the tape path
def state_turn_to_start():
    print("state_turn_to_start")
    
    # Use the camera to measure the angle to the tape
    tapeAngle = measure_start_angle()
#    tapeAngle = TAPE_NOT_FOUND_SET    # NOTE: Placeholder - Use to test if tape was NOT found
#    tapeAngle = 10                     # NOTE: Placeholder - Use to test if tape WAS found
    
    # Implement find tape code here    
    if(tapeAngle == TAPE_NOT_FOUND_SET):        
        print("\tTape Not Found")
        return state_FOV_rotate
    else:
        print("\ttapeAngle =", tapeAngle)
        writeMotionData(tapeAngle, 0)
        waitForMotion()        
        return state_drive_to_start()    




# Calculate the distance from the robot to the start of the tape path then drive
def state_drive_to_start():
    print("state_drive_to_start()") 

    # Move the constant distance to start 
    distToStart = 18  # The sqrt of two times 12 inches    
    writeMotionData(0, distToStart)    
    waitForMotion()  
    return state_turn_CCW

    


# Rotate the robot counterclockwise a constant angle to realign the camera FOV with the path
def state_turn_CCW():
    print("state_turn_CCW")
    
    # move a constant angle to align with start
    writeMotionData(ANGLE_AFTER_START, 0)    
    waitForMotion()
    return state_update_angle


# Calculate the angle to the next segment of tape the robot sees in front of it
def state_update_angle():
#    TODO: if angle measured is above a fudge delta, turn angle then move constant distance
#    if below delta, dont turn and move straight
#----Try without fudge first
#    if not found, turn right some constant
#    must detect cross and move on

    print("state_update_angle")     

#    nextAngle = 5  # NOTE: Placeholder  
    nextAngle = measure_angle()
    print("\tNext Angle =", nextAngle)

    # If the tape was not found, turn right some constant
    if nextAngle == TAPE_NOT_FOUND_SET:
        writeMotionData(VIEW_SHIFT_ANGLE, 0)    
        waitForMotion()
        return state_update_angle

    # If angle measured is above a fudge delta, turn angle then move constant distance
    elif abs(nextAngle) >= ANGLE_FUDGE_DELTA:
        # If the robot has turned enough to warrant checking for the cross, move to check for cross state
        global totalPathRotation
        if totalPathRotation >= ROTATION_BEFORE_CROSS:            
            writeMotionData(nextAngle, 0)    
            waitForMotion()
            print(totalPathRotation)  
            return state_check_for_cross
        else:    
            writeMotionData(nextAngle, 0)    
            waitForMotion()
            totalPathRotation += nextAngle      # Add the angle turned to the counter NOTE: This may need to be tweeked to interact with the angle fudge factor properly
            print(totalPathRotation)  
            return state_drive_forward
        
    # If angle measured  is below delta, dont turn. Just move straight.
    elif abs(nextAngle) < ANGLE_FUDGE_DELTA:
##        totalPathRotation += nextAngle          # Add the angle turned to the counter NOTE: This may need to be tweeked to interact with the angle fudge factor properly
        return state_drive_forward
        
        


# Check to see if the cross was detected
def state_check_for_cross():
    print("state_check_for_cross")
    # Function to check for cross goes here
    
    img = take_picture(2.0,1.0)
    if (is_cross(img)):
            crossFound = True    # NOTE: Placeholder. Implement function to set this flag
    return state_drive_forward




# Drive forward a constant distance   
def state_drive_forward():
    print("state_drive_forward")
    if not crossFound:
        writeMotionData(0, FORWARD_STEP_DISTANCE)
        waitForMotion()
        return state_update_angle
    else:
        writeMotionData(0, FORWARD_STEP_DISTANCE)   # May need to change the last move forward
        waitForMotion()
        return state_stop


    

# The end of the tape was reached. Exit the state machine
def state_stop():
    print("Done with state machine")
    ser.close()
    return None



# initalization
#state = state_update_angle # initial state

state = state_start # initial state


#init videocapture
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
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
  
    
# When everything done, release the capture
cv.destroyAllWindows()








