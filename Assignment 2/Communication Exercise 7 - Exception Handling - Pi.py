#############################################################################################
# NAME:     David Long
# CLASS:    EENG-350
# TITLE:    Communication Exercise 7 - Exception Handling - Pi
#
# FUNCTION: This is a modified version of Communication Exercise 5
#           Asks the user to input a string. This string is then converted to a list of
#           integers and sends it from a Raspberry Pi to an attached Arduino. The Arduino
#           then reverses the list and sends it back to the Pi where the list is converted
#           back into characters and printed to the screen. If the SDA pin is disconnected
#           during runtime, an exception will be raised and a message will be displayed on
#           an attached LCD screen.
#
# HARDWARE: Pin Connection Guide
#           Pi:            Arduino:
#           GPIO2 (SDA) -> A4
#           GPIO3 (SCL) -> A5
#           GND         -> GND
#
#           Connect LCD display to Raspberry Pi header pins
#
# EXECUTE:  Follow the connection guide above to link the Raspberry Pi and Arduino together
#           for I2C. Next, upload the "Communication_Exercise_5and7-Arduino.ino" code to the
#           Arduino. Finally, run this program and follow the prompts in the Python Shell.
#############################################################################################

import smbus
import time

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

bus = smbus.SMBus(1)

# Set the address to use for I2C communication (Also setup in Arduino Program)
address = 0x04

def writeBlock(blockToSend):
    # Send the list of ASCII codes of the passed character to the Arduino over I2C using
    # the passed address as a block of integers to register 0.
    bus.write_i2c_block_data(address, 0, blockToSend)
    return -1

def readBlock(sentBlock):
    # Read the value at the passed offset from the Arduino
    receivedBlock = bus.read_i2c_block_data(address, 0, len(sentBlock))
    return receivedBlock

operationComplete = False   # Flag to tell when communication was succesful
# Repeat the main loop until communication has been succesfully established with the Arduino
while not operationComplete:
    try:
        lcd.color = [25, 100, 0]
        # Get a string to send to the Arduino from the user
        stringToSend = input("Enter a string to send to the Arduino:\n")

        # Convert the string to a list of the ASCII codes for each individual character
        strToIntList = []
        for char in stringToSend:
            strToIntList.append(ord(char))
        print("\nYour string input has been converted to a list of ASCII codes:")
        print(strToIntList)

        # Send the block of integers to the Arduino
        writeBlock(strToIntList)
        print("Which was sent to the Arduino")

        # Receive the reversed block of integers from the Arduino
        reversedBlock = readBlock(strToIntList)

        # Convert the reversed block of integers back to a string of ASCII characters
        reversedBlockToStr = ""
        for code in reversedBlock:
            reversedBlockToStr += chr(code)
        print("\nThe reverse of your string input was received from the Arduino. It is:")
        print(reversedBlockToStr)

        # Signal that the operation was completed succesfully and the program can exit
        operationComplete = True
        lcd.clear()                 # Clear the error message on the LCD if it exists
        lcd.color = [25, 100, 0]    # Reset the LCD color

    # If I2C communication is lost
    except OSError:    
        # Print the message to the screen and LCD display
        print("\nI2C Error\n")
        lcd.message = "I2C Error"

















    
