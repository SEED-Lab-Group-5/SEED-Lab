import serial
import time

# Set serial address and baud rate
ser = serial.Serial('/dev/ttyACM0', 4800)
time.sleep(3)   # Wait a moment to finalize the connection



#################################################################
# Data functions
#################################################################
def readData():
    
    data = 0
    # While there are still bytes to be read from the buffer
    if (ser.in_waiting > 0):
        # Read line from buffer and decode using utf-8 
        data = (ser.readline().decode('utf-8'))
#        print(data)
#    ser.reset_input_buffer()
    return data


def writeData(motionType, motionMagnitude):
#    
    ser.write((str(motionType)+" "+str(motionMagnitude)).encode('utf-8'))
    ser.reset_output_buffer()
#    ser.write(str(motionMagnitude).encode('utf-8'))
    time.sleep(2)
    return -1

MOTION_COMPLETE_SET = -127  # Transmitted from Arduino when flag is set
START_STATE_MACHINE_SET = -126
MOTION_TYPE_ROTATE = -125
MOTION_TYPE_FORWARD = -124
TAPE_NOT_FOUND_SET = -123

motionType = MOTION_TYPE_ROTATE
motionMagnitude = 22
while True:
    input("Press Enter to Retrieve Data")
    flag = readData()
    print(flag)
    
    input("Press Enter to Send Data")    
    writeData(motionType, motionMagnitude)
    print(readData());
    