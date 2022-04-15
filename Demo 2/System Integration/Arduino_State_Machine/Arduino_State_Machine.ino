#include <Control.h>

#include <Control.h>

#include <Control.h>

#include <Control.h>

#include <Control.h>

#include <Control.h>

//////////////////////////////////////////////////////////////////////// 
// NAME:
// CLASS:    EENG-350
// GROUP:    5
// TITLE:    Demo 2
// FUNCTION: Write what your code does here
// HARDWARE: Any hardware connections you must make to your device
// SOFTWARE: Any software that must be installed to the device
// EXECUTE:  Execution instructions for your program
// RESOURCE: Link to any resource you used
// PURPOSE:  What the resource was used for
// RESOURCE: Link to any resource you used
// PURPOSE:  What the resource was used for
//////////////////////////////////////////////////////////////////////



#include "Control.h"
#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include <Wire.h>


#define SLAVE_ADDRESS 0x04

// Define data types needed for finite state machine
typedef enum {START, FOV_ROTATE, FIND_TAPE, TURN_TO_START, CALC_DIST_TO_START, DRIVE_TO_START, CALC_PATH_ANGLE, TURN_INLINE_TO_PATH, CALC_DIST_TO_END, DRIVE_TO_END, STOP} currentState_t;

// Flags and transmission codes // TODO too many flags for movement.
bool rotateComplete = false;          //!< Indicates if robot is done rotating
bool motionComplete = false;        //!< Indicates if robot has stopped moving forward
bool tapeNotFound = false;          //!< Indicates if tape was not found in the field of view
#define ROTATE_COMPLETE_SET (-127)  //!< Transmitted to Pi when flag is set
#define TAPE_NOT_FOUND_SET (-126)   //!< Transmitted from Pi when flag is set
#define MOTION_COMPLETE_SET (-125)  //!< Transmitted to Pi when flag is set
#define START_STATE_MACHINE_SET (-124)    //!< Indicates the Pi is ready and the state machine should start

// I2C
int data = 0;                       //!< Value to store data received over I2C
bool flagSent = false;              //!< Indicates if a flag was sent to the Pi
bool dataReceived = false;          //!< Indicates if data was read from the Pi

// Functions
void receiveData(int byteCount);
void sendData();

// Object
Control control; //!< magic

/**
 * Does initial setup
 */
void setup() {
    // Begin serial communication
    Serial.begin(9600); // start serial for output

    // Initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // Define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
    Serial.println("Ready!");

    // Get initial values of currentTime and startTime
    control.startControl();

}
//////////////////////////
// Finite State Machine //
//////////////////////////
int angleToStart    = TAPE_NOT_FOUND_SET;   // The angle to the start of the line
int distanceToStart = TAPE_NOT_FOUND_SET;   // The distance to the start of the line
int angleToEnd      = TAPE_NOT_FOUND_SET;   // The angle to the end of the line
int distanceToEnd   = TAPE_NOT_FOUND_SET;   // The distance to the end of the line

static currentState_t currentState = START;
void loop() {
    switch (currentState) {

        // The START state runs at the start of the program, no code takes place
        case START:
            if (dataReceived) {
                // If the Pi indicated no tape was found in the screen, go back to FOV_ROTATE state
                if (data == START_STATE_MACHINE_SET) {
                    // Move to next state
                    currentState = FOV_ROTATE;
                }
                dataReceived = false;   // Reset dataReceived flag
            }
            break;


        // FOV_ROTATE State: Rotate the robot clockwise by slightly over half the field of view (30 degrees). The flag
        //                   rotateComplete will be set to -127 and sent to the Pi when the robot has finished rotating
        case FOV_ROTATE:
            // Set rotateComplete flag true after robot finishes rotating            

            if (!rotateComplete) {
                rotateComplete = control.drive(30.0, 0);
            }

            // If the Arduino finished sending data to the Pi, reset the flag and encoders and move to the next state
            if (flagSent && rotateComplete) {
                flagSent = false;
                rotateComplete = false;
                currentState = FIND_TAPE;
            }
            
            break;


        // FIND_TAPE State: Wait for the Pi to find the angle to a line of tape in the camera's FOV and transmit it. The flag value
        //                  TAPE_NOT_FOUND_SET will be sent by the Pi if no tape is found. Otherwise the value sent will be the angle.
        case FIND_TAPE:
            // If the Arduino received data from the Pi
            if (dataReceived) {
                angleToStart = data;
                // If the Pi indicated no tape was found in the screen, go back to FOV_ROTATE state
                if (angleToStart == TAPE_NOT_FOUND_SET) {
//                    Serial.println("Tape Not Found");
                    currentState = FOV_ROTATE;
                    
                // If the Pi indicated that it found tape, store the angle the Arduino received and move to TURN_TO_START state
                } else {                    
//                    Serial.print("angleToStart = ");
//                    Serial.println(angleToStart);
                    currentState = TURN_TO_START;
                }
                dataReceived = false;   // Reset dataReceived flag
            }
            break;


        // TURN_TO_START State: Rotate the robot to be in line with the start of the tape path. Send the flag code ROTATE_COMPLETE_SET to the Pi
        //                      when the robot is done rotating
        case TURN_TO_START:
        
            rotateComplete = control.drive(angleToStart, 0); // Set flag true after robot finishes rotating
                      
            // If the Arduino finished sending data to the Pi, reset the flag and encoders and move to the next state
            if (flagSent) {
                flagSent = false;  
                rotateComplete = false;
//                end();
                currentState = CALC_DIST_TO_START;    
            }
            break;


        // CALC_DIST_TO_START State: The Pi calculates the distance from the robot to the start of the tape path and transmits this value to the Arduino
        case CALC_DIST_TO_START:        
            // If the Arduino received data from the Pi
            if (dataReceived) {
                distanceToStart = data;                  
                
//                Serial.print("distanceToStart = ");
//                Serial.println(distanceToStart);               
                
                // Reset flag
                dataReceived = false;

                // Move to next state
                currentState = DRIVE_TO_START;
            }
            break;


        // DRIVE_TO_START State: Drive the robot to the start of the tape path. Transmit the flag code MOTION_COMPLETE_SET to the Pi when the robot has
        //                       finished driving forward.
        case DRIVE_TO_START:        
         
            motionComplete = control.drive(0, distanceToStart); // Set flag true after robot finishes moving forward
            
            // If the Arduino finished sending data to the Pi, reset the flag and encoders and move to the next state
            if (flagSent) {
                flagSent = false;   
//                end();
                motionComplete = false;
                currentState = CALC_PATH_ANGLE;            
            }
            break;          


        // CALC_PATH_ANGLE State: The Pi calculate the angle the robot needs to turn to to be in line with the tape path. This value is then sent to the Arduino
        case CALC_PATH_ANGLE:
            // If the Arduino received data from the Pi
            if (dataReceived) {
                angleToEnd = data;
//                Serial.print("angleToEnd = ");
//                Serial.println(angleToEnd);
                
                // Reset flag
                dataReceived = false;
                
                // Move to next state
                currentState = TURN_INLINE_TO_PATH;
            }
            break;   


        // TURN_INLINE_TO_PATH State: Turn the robot to be in line with the tape path. Transmit the flag code ROTATE_COMPLETE_SET to the Pi
        //                            when the robot is done rotating
        case TURN_INLINE_TO_PATH:

            rotateComplete = control.drive(angleToEnd, 0); // Set flag true after robot finishes rotating
            
            // If the Arduino finished sending data to the Pi, reset the flag and encoders and move to the next state
            if (flagSent) {
                flagSent = false;   
                rotateComplete = false;
//                end();
                currentState = CALC_DIST_TO_END;
            }
            break;


        // CALC_DIST_TO_END State: The Pi calculate the distance from the robot to the end of the tape path. This value is then sent to the Arduino
        case CALC_DIST_TO_END:
            // If the Arduino received data from the Pi
            if (dataReceived) {
                distanceToEnd = data;
//                Serial.print("distanceToEnd = ");
//                Serial.println(distanceToEnd);

                // Reset flag
                dataReceived = false;
                
                // Move to next state
                currentState = DRIVE_TO_END;
            }
            break;


        // DRIVE_TO_END State: Drive to the end of the tape path. Transmit the flag code MOTION_COMPLETE_SET to the Pi when the robot has
        //                     finished driving forward.
        case DRIVE_TO_END:
        
            motionComplete = control.drive(0, distanceToEnd); // Set flag true after robot finishes moving forward

            // If the Arduino finished sending data to the Pi, reset the flag and encoders and move to the next state
            if (flagSent) {
                flagSent = false;  
//                end();
                currentState = STOP;
            }
            break;


        // STOP State: The end of the tape was reached. Exit the state machine
        case STOP:
            control.drive(0,0);
            rotateComplete = false;
            motionComplete = false;
            Serial.println("STOP");
            break;
    }   
}

/**
 * Callback for received data (From mini project)
 * @param byteCount
 */
// callback for received data
void receiveData(int byteCount){
    dataReceived = true;                // Indicate that the Arduino received data from the Pi
//    Serial.println("Data Received");
  
    // While there are still bytes to read, read them and store the most recent to number
    while(Wire.available()) {
        data = Wire.read();   
        
        // Convert to Two's compliment representation
        if (data > 127) {
            data = 256 - data;
            data *= -1; 
        }
//        Serial.print("Data: ");
//        Serial.println(data);
    }
} // End receiveData

// callback for sending data
void sendData(){
   
//    Serial.println("Data Requested");

    // If a write request was received from the Pi
    switch (currentState) { 
        case FOV_ROTATE:
        case TURN_TO_START:
        case TURN_INLINE_TO_PATH:
            // Wait until robot has finished rotating
            if (rotateComplete) {
                Wire.write(ROTATE_COMPLETE_SET);
                flagSent = true;
                Serial.println("Rotate FLAG Sent");
            }
            else {
                Wire.write(0);
            }            
            break;

        case DRIVE_TO_START:
        case DRIVE_TO_END:
            // Wait until robot has finished driving forward
            if (motionComplete) {
                Wire.write(MOTION_COMPLETE_SET);
                flagSent = true;
                Serial.println("motion FLAG Sent");
            }
            break;
        case START:
            break;
        case FIND_TAPE:
            break;
        case CALC_DIST_TO_START:
            break;
        case CALC_PATH_ANGLE:
            break;
        case CALC_DIST_TO_END:
            break;
        case STOP:
            break;
    }
    delay(100); //TODO remove delay and replace with millis
} // End sendData
