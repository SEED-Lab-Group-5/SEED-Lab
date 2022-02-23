#############################################################################################
# NAME:     David Long
# CLASS:    EENG-350
# TITLE:    Communication Exercise 8 - Serial Communication - Pi
#
# FUNCTION: Gets a integer value as user input then sends it to an attached Arduino over
#           serial. The Arduino then adds 5 to this number and sends it back to the Raspberry
#           Pi.
#
# HARDWARE: Connect the Arduino to the Raspberry Pi using a USB-A to USB-B cable
#
# EXECUTE:  Follow the connection guide above to link the Raspberry Pi and Arduino together
#           for serial. Ensure the Arduino serial monitor is not open. Next, upload the
#           "Communication_Exercise_8-Arduino.ino" code to the Arduino. Finally, run this
#           program and follow the prompts in the Python Shell.
#############################################################################################
import serial
import time

# Set serial address and baud rate
ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(3)   # Wait a moment to finalize the connection

# Function for reading serial from Arduino
def readFromArduino():
    # While there are still bytes to be read from the buffer
    while (ser.in_waiting > 0):
        try:
            # Read line from buffer and decode using utf-8 
            recievedNum = ser.readline().decode('utf-8').rstrip()
            print(recievedNum)
        except:
            print("Communication Error")    

# Ask user for integer then encode it and send over serial
value = input("Enter an integer to send to the Arduino: ")
ser.write(value.encode())

# Wait for Arduino to set up a response
time.sleep(2)
readFromArduino()
