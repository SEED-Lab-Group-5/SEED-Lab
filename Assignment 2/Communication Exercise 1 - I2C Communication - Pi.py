#############################################################################################
# NAME:     David Long
# CLASS:    EENG-350
# TITLE:    Communication Exercise 1 - I2C Communication - Pi
#
# FUNCTION: Gets a integer value as user input then sends it to an attached Arduino. The
#           Arduino then adds 5 to this number and sends it back to the Raspberry Pi.
#
# HARDWARE: Pin Connection Guide
#           Raspberry Pi | Arduino
#           GPIO2 (SDA) -> A4
#           GPIO3 (SCL) -> A5
#           GND         -> GND
#
# EXECUTE:  Follow the connection guide above to link the Raspberry Pi and Arduino together
#           for I2C. Next, upload the "Communication_Exercise_1-Arduino.ino" code to the
#           Arduino. Finally, run this program and follow the prompts in the Python Shell.
#############################################################################################
import smbus
import time
# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    # bus.write_byte(address, value)
    bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 0)
    return number

while True:
    var = int(input("Enter an integer to send to the Arduino: "))
    if not var:
        continue

    writeNumber(var)
    print("RPI: Hi Arduino, I sent you ", var)
    # sleep one second
    time.sleep(1)

    number = readNumber()
    print("Arduino: Hey RPI, I received the number and added 5 to it: ", number)
    print()
