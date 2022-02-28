#############################################################################################
# NAME:     David Long
# CLASS:    EENG-350
# TITLE:    Communication Exercise 2 - Advanced I2C - Pi
#
# FUNCTION: Asks the user for a number and an offset of 0 or 1. These values are sent
#           to the Arduino. If the offset sent was 0, the Arduino stores the number
#           to a different variable than if the offset sent was 1. The user is then asked
#           for an offset to read from. This value is sent from the Pi to the Arduino, and
#           the value cooresponding to that offset is sent back to the Pi and modified as
#           follows:
#           - If read offset = 0, add 5 to the saved value
#           - If read offset = 1, add 10 to the saved value
#
# HARDWARE: Pin Connection Guide
#           Raspberry Pi | Arduino
#           GPIO2 (SDA) -> A4
#           GPIO3 (SCL) -> A5
#           GND         -> GND
#
# Execute:  Follow the connection guide above to link the Raspberry Pi and Arduino together
#           for I2C. Next, upload the "Communication_Exercise_2and3-Arduino.ino" code to the
#           Arduino. Finally, run this program and follow the prompts in the Python Shell.
#############################################################################################

import smbus
import time

bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeArduino(value, offset):
    # Write passed value to I2C address to passed register offset 
    bus.write_byte_data(address, offset, value)
    return -1

def readArduino(offset):
    # Read the value at the passed offset from the Arduino
    readVal = bus.read_byte_data(address, offset)
    
    # If the user chose to read from register 0
    if offset == 0:
        modReadVal = readVal + 5
        print("\nThe value", readVal, "was read from register 0.")
        print("Since you chose to read from register 0, 5 was added to the read value.")

    # If the user chose to read from register 1
    elif offset == 1:
        modReadVal = readVal + 10
        print("\nThe value", readVal, "was read from register 1.")
        print("Since you chose to read from register 1, 10 was added to the read value.")

    return modReadVal

# Collect user input and convert to integer
writeVal = int(input("Enter an integer to send to the Arduino: "))
writeOffset = int(input("Enter a register offset to writing to (0 or 1): "))

# Write value to chosen register offset
writeArduino(writeVal, writeOffset)
time.sleep(1) # sleep one second

# Ask user for offset to read from
readOffset = int(input("Enter the register offset you would like to read from: "))

# Read the value at that offset from the Arduino and print the modified value
modReadVal = readArduino(readOffset)
print("The modified read value is", modReadVal)










