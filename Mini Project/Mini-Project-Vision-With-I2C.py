########################################################################## 
# NAME:     Johnathan Evans (Primary Author),   Modified by David Long
# CLASS:    EENG-350
# TITLE:    Mini-Project
# FUNCTION: Find a marker on the camera and return what quadrant it is in
# RUNNING:  Use Pi, press go
# RESOURCE: https://docs.opencv.org/4.1.0/d6/d00/tutorial_py_root.html
# PURPOSE:  openCV resource
##########################################################################

# Imports 
import smbus
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
print(cv.__version__)

# Create new bus object
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

# Send value to Arduino
def writeArduino(value):
    # Write passed value to I2C address
    bus.write_byte(address, value)
    return -1

#################################
# Code required for LCD interface
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#################################

# Print message to LCD screen indicating what angle the motor is being told to move to
def sendToLCD(quadrant):
    # Clear the LCD screen and reset the color to blue
    lcd.clear()
    lcd.color = [100, 0, 0]

    # Set LCD message
    if quadrant == 0:
        lcd.message = "SETPOINT: 0 Rad"
        
    elif quadrant == 1:
        lcd.message = "SETPOINT: PI/2"

    elif quadrant == 2:
        lcd.message = "SETPOINT: PI"

    elif quadrant == 3:
        lcd.message = "SETPOINT: 3PI/2"

    else:
        lcd.message = "MARKER NOT FOUND"

def main():

    width = 640
    height = 368

    # Let the camera calibrate and make a white balance
    camera = PiCamera(resolution = (width, height), framerate = 30)
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
    input('Hold monochrome marker in front of camera and press enter to begin callibration')

    camera.capture(rawCapture, format = "bgr")
    image = rawCapture.array
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    image = cv.blur(image, (10, 10))  # Blur to smooth
    px = image[int(height / 2),int(width / 2)]
    print(px)
    lower_mask = np.array([px[0] - 10, 0, 0])
    upper_mask = np.array([px[0] + 10, 255, 255])


    #print(colorMedian)

    #cv.imshow('', image)
    #cv.waitKey(0)

    camera.close()
    #-----------------------------------
    # Prepare video capture
    cap = cv.VideoCapture(0)
    #currentWB = cap.get(cv.CAP_PROP_WHITE_BALANCE_BLUE_U)

    # Test camera function
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
        
    # Run video with cap open
    while True:
        #cap.set(cv.CAP_PROP_WHITE_BALANCE_BLUE_U, currentWB)
        
        # Capture frame-by-frame
        ret, img = cap.read()
        
        # If frame is read correctly ret is True
        if not ret:
            print("Cannot receive frame (stream end?). Exiting ...")
            break
       
        # Image operations below
        # Resize image for easier processing
        width = int(img.shape[1] * 1/2)
        height = int(img.shape[0] * 1/2)
        imgtuple = (width,height)
        img = cv.resize(img, imgtuple)
        
        # Transform to hsv for color analysis
        imghsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        mask = cv.inRange(imghsv, lower_mask, upper_mask)

        # Make mask
        #mask = cv.cvtColor(mask, cv.COLOR_HSV2BGR)
        
        # Below code is to clean the image
        kernel = np.ones((5, 5), np.uint8)  # create kernel

        mask = cv.erode(mask, kernel, iterations = 1)   # Dialate to fill in gap
        mask = cv.erode(mask, kernel, iterations = 1)   # Dialate to fill in gap
        mask = cv.blur(mask,(5, 5))                     # Blur to smooth
        mask = cv.blur(mask,(5, 5))                     # Blur to smooth
        
        # Find the average location
        nonzero = np.nonzero(mask)
        if len(nonzero[0]) == 0:
            print('No markers found')

        # ============================= TODO =============================
        # view location to find quadrant
        # ================================================================    
        else:
            # Find angle and return quadrant (-1 for no color found)
            locx = nonzero[1].mean()
            locy = nonzero[0].mean()
            ret_quad = -1
            if locx > width / 2 and locy < height / 2:
                ret_quad = 1
            elif locx < width / 2 and locy < height / 2:
                ret_quad = 0
            elif locx < width / 2 and locy > height / 2:
                ret_quad = 3
            elif locx > width / 2 and locy > height / 2:
                ret_quad = 2
            else:
                ret_quad = -1
            #====================================DAVID HERE IS YOUR RETURN

##            ret_quad = int(input("Enter Quadrant: "))    
            writeArduino(ret_quad)        
            sendToLCD(ret_quad)
            print(ret_quad)
            
            #==================================== -1 IS FOR NO DETECTED COLOR
            
        # Display the resulting frame
        cv.imshow('frame', mask)
        if cv.waitKey(1) == ord('q'):
            break
        
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()


# Run main function   
if __name__ == "__main__":
    main()
