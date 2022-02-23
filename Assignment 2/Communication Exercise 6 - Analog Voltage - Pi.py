#############################################################################################
# NAME:     David Long
# CLASS:    EENG-350
# TITLE:    Communication Exercise 6 - Analog Voltage - Pi
#
# FUNCTION: A connected Arduino reads the voltage across a potentiometer and sends this data
#           to the Raspberry Pi running this program. That data is then displayed on an
#           attached LCD display
#
# HARDWARE: Pin Connection Guide
#           Pi:            Arduino:
#           GPIO2 (SDA) -> A4
#           GPIO3 (SCL) -> A5
#           GND         -> GND
#
#           Potentiometer Setup (Connected to Arduino)
#           A0  -> Center Lead of Pot
#           GND -> Outer Left Lead of Pot
#           5V  -> Outer Right Lead of Pot
#
#           Connect LCD display to Raspberry Pi header pins
#
# EXECUTE:  Follow the connection guide above to link the Raspberry Pi and Arduino together
#           for I2C. Next, upload the "Communication_Exercise_5-Arduino.ino" code to the
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

lcd.color = [100, 0, 0]
bus = smbus.SMBus(1)

# Set the address to use for I2C communication (Also setup in Arduino Program)
address = 0x04

def readVoltage():
    # Read the current voltage across the potentiometer from the Arduino
    # This value is given by a block of 4 bytes ranging from 0-255
    voltageByteBlock = bus.read_i2c_block_data(address, 0, 4)

    # Convert 4-byte block to single integer equivalent ranging from 0-1023
    sampledVoltage = 0
    for byte in voltageByteBlock:
        if byte != 0:
            sampledVoltage = sampledVoltage + byte + 1
        
    if sampledVoltage / 256 > 0:    # no need to subtract 1 if only 4th byte is non-zero
        sampledVoltage = sampledVoltage - 1     # Subtract 1 to make max value 1023

    # Convert integer voltage to float equivalent ranging from 0-5V
    sampledVoltage = round(sampledVoltage * (5.0 / 1023.0), 2)
    return sampledVoltage

# Allow user to stop and start sampling the voltage until they press Q to quit the program
repeatInput = input("\nPress enter to start sampling the potentiometer voltage or 'Q' to quit:")
if repeatInput.upper() != 'Q':
        print("To stop sampling, press Ctrl+C.")
        
while repeatInput.upper() != 'Q':    
    try:
        # Continue reading the voltage and printing it to the screen and LCD display
        while True:          
            currentVoltage = readVoltage()
            lcd.clear()
            lcd.message = str(currentVoltage) + "V"
            print(currentVoltage, "V")
            
            
    # Terminate the infinite while loop without ending the program
    except KeyboardInterrupt:
        pass    

    repeatInput = input("Press enter to sample the potentiometer voltage or 'Q' to quit: ")
    if repeatInput.upper() != 'Q':
        print("To stop sampling, press Ctrl+C.")
    
    
    


