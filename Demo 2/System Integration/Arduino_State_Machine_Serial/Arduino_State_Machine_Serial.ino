

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

#include <Control.h>
#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"


// Define data types needed for finite state machine
typedef enum {START, DRIVE, WAIT, SEND_STATUS} currentState_t;

// Flags and transmission codes
bool motionComplete = false;        //!< Indicates if robot has stopped moving/rotating
int  MOTION_COMPLETE_SET = -1000;  //!< Transmitted to Pi when flag is set
int START_STATE_MACHINE_SET = -2000;    //!< Indicates the Pi is ready and the state machine should start
int MOTION_TYPE_ROTATE = -3000;
int MOTION_TYPE_FORWARD = -4000;

// Serial
String dataString;                  //!< Value to store data received over serial
bool dataReceived = false;          //!< Indicates if data was read from the Pi
int motionType = MOTION_TYPE_ROTATE;
int motionMagnitude = 0;
int magnitude = 0;

// Object
Control control; //!< magic

/**
 * Does initial setup
 */
void setup() {
    // Begin serial communication
    Serial.begin(19200); // start serial for output

    // Get initial values of currentTime and startTime
    control.startControl();

    pinMode(LED_BUILTIN, OUTPUT);
     
}

//////////////////////////
// Finite State Machine //
//////////////////////////

static currentState_t currentState = WAIT;
void loop() {
//    control.drive(30, 0);
    switch (currentState) {

//        // The START state runs at the start of the program, no code takes place
//        case START:
//            digitalWrite(LED_BUILTIN, HIGH);
//            if (dataReceived) {
//                // If the Pi indicated no tape was found in the screen, go back to FOV_ROTATE state
//                if (motionType == START_STATE_MACHINE_SET) {
//                    // Move to next state
//                    currentState = WAIT;
//                }
//                dataReceived = false;   // Reset dataReceived flag
//            }
//            break;
            
        // WAIT State: Wait until data is recieved from Pi
        case WAIT:
//            digitalWrite(LED_BUILTIN, LOW);
            motionComplete = false;
            if (dataReceived) {
                currentState = DRIVE;
                dataReceived = false;
            }            
            break;
            

        // DRIVE State: Rotate the robot by a given angle or drive forward a given distance motionComplete 
        //              will be set to -127 and sent to the Pi when the robot has moving
        case DRIVE:  
            if(!motionComplete) {
                motionComplete = control.drive(30, 0);    
            }    
            if (motionComplete) {   
                currentState = SEND_STATUS;
                
            }
//            delay(100);
         
            break;

        // SEND_STATUS State: Tell the Pi that motion is complete
        case SEND_STATUS:
            Serial.println(MOTION_COMPLETE_SET);            
            currentState = WAIT;
    } 
//    if (!motionComplete) { 
//        if (motionType == MOTION_TYPE_ROTATE) {
//            motionComplete = control.drive(magnitude,0);
//        }
//        else if (motionType == MOTION_TYPE_FORWARD) {
//            motionComplete = control.drive(0, magnitude);
//        }      
//    }
}

void drivePlz(int motionType, int motionMagnitude) {
            
    if (motionType == MOTION_TYPE_ROTATE) {   
        motionComplete = (30.0, 0);
//        motionComplete = control.drive(0.0, motionMagnitude);  
    }
    else if (motionType == MOTION_TYPE_FORWARD) {                
        motionComplete = control.drive(0.0, motionMagnitude);
    }
}


void serialEvent(){
    if (Serial.available() > 0) {
        motionType = Serial.readStringUntil('\t').toInt();
        motionMagnitude = Serial.readStringUntil('\n').toInt();
        
        dataReceived = true;    // Flag to indicate if a serial read operation has occured
    }
    // Wait until buffer is empty
//    Serial.flush();
}
