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
import serial
import cv2 as cv
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
print(cv.__version__)

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
        lcd.message = "SETPOINT: PI/2 Rad"

    elif quadrant == 2:
        lcd.message = "SETPOINT: PI Rad"

    elif quadrant == 3:
        lcd.message = "SETPOINT: 3PI/2 Rad"

    else:
        lcd.message = "MARKER NOT FOUND"
        
    

def main():
    # Set serial address and baud rate
    ser = serial.Serial('/dev/ttyACM0', 9600)
    time.sleep(3)   # Wait a moment to finalize the connection

    WIDTH = 640
    HEIGHT = 368

    # Let the camera calibrate and make a white balance
    camera = PiCamera(resolution = (WIDTH, HEIGHT), framerate = 30)
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
            print("Can't receive frame (stream end?). Exiting ...")
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

        #make mask
        #mask = cv.cvtColor(mask, cv.COLOR_HSV2BGR)
        
        # Below code is to clean the image
        kernel = np.ones((5, 5), np.uint8)# create kernel

        mask = cv.erode(mask, kernel, iterations = 1)   # Dialate to fill in gap
        mask = cv.erode(mask, kernel, iterations = 1)   # Dialate to fill in gap
        mask = cv.blur(mask,(5, 5))                  # Blur to smooth
        mask = cv.blur(mask,(5, 5))                  # Blur to smooth
        
        # Find the average location
        nonzero = np.nonzero(mask)
        if len(nonzero[0]) == 0:
            print('No markers found')
            
        else:
        # ============================= TODO =============================
        # view location to find quadrant
        # ================================================================
            # Find angle and return quadrant (-1 for no color found)
            locx = nonzero[1].mean()
            locy = nonzero[0].mean()
            ret_quad = -1
            if locx > width / 2 and locy < height / 2:
                ret_quad = 0
            elif locx < width / 2 and locy < height / 2:
                ret_quad = 1
            elif locx < width / 2 and locy > height / 2:
                ret_quad = 2
            elif locx > width / 2 and locy > height / 2:
                ret_quad = 3
            else:
                ret_quad = -1
            #====================================DAVID HERE IS YOUR RETURN
                
            ser.write(ret_quad.encode())
            sendToLCD(ret_quad)
            print(ret_quad)
            time.sleep(100) # Wait some time before looking for the marker again
            
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

