

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
#define MOTION_COMPLETE_SET (-127)  //!< Transmitted to Pi when flag is set
#define START_STATE_MACHINE_SET (-126)    //!< Indicates the Pi is ready and the state machine should start
#define MOTION_TYPE_ROTATE (-125)
#define MOTION_TYPE_FORWARD (-124)

// Serial
String dataString;                  //!< Value to store data received over serial
bool dataReceived = false;          //!< Indicates if data was read from the Pi
int motionType = MOTION_TYPE_ROTATE;
int motionMagnitude = 0;


// Object
Control control; //!< magic

/**
 * Does initial setup
 */
void setup() {
    // Begin serial communication
    Serial.begin(9600); // start serial for output

    // Get initial values of currentTime and startTime
    control.startControl();

    pinMode(13, OUTPUT);
}

//////////////////////////
// Finite State Machine //
//////////////////////////

static currentState_t currentState = START;
void loop() {
    switch (currentState) {

        // The START state runs at the start of the program, no code takes place
        case START:
            if (dataReceived) {
                // If the Pi indicated no tape was found in the screen, go back to FOV_ROTATE state
                if (motionType == START_STATE_MACHINE_SET) {
                    // Move to next state
                    currentState = WAIT;
                }
                dataReceived = false;   // Reset dataReceived flag
            }
            break;
            
        // WAIT State: Wait until data is recieved from Pi
        case WAIT:
//            digitalWrite(13, HIGH);
            if (dataReceived) {
                currentState = DRIVE;
                dataReceived = false;
            }
            break;
            

        // DRIVE State: Rotate the robot by a given angle or drive forward a given distance motionComplete 
        //              will be set to -127 and sent to the Pi when the robot has moving
        case DRIVE:   
            digitalWrite(13, HIGH);         
            if (motionType == MOTION_TYPE_ROTATE) {
                motionComplete = control.drive(motionMagnitude, 0.0);
            }
            else if (motionType == MOTION_TYPE_FORWARD) {
                motionComplete = control.drive(0.0, motionMagnitude);
            }
            if (motionComplete) {
                currentState = SEND_STATUS;
            }            
            break;


        // SEND_STATUS State: Tell the Pi that motion is complete
        case SEND_STATUS:
            Serial.println(MOTION_COMPLETE_SET);
            currentState = WAIT;
    }   
}

void serialEvent(){
    if (Serial.available() > 0) {
        motionType = Serial.readStringUntil('\n').toInt();
        motionMagnitude = Serial.readStringUntil('\n').toInt();
        dataReceived = true;    // Flag to indicate if a serial read operation has occured
    }
    // Wait until buffer is empty
    Serial.flush();
}
