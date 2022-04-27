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
int  MOTION_COMPLETE_SET = -127;    //!< Transmitted to Pi when flag is set
int START_STATE_MACHINE_SET = -126; //!< Indicates the Pi is ready and the state machine should start
int MOTION_TYPE_ROTATE = -125;
int MOTION_TYPE_FORWARD = -124;

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
    Serial.begin(9600); // start serial for output
    dumpData(); 
}

//////////////////////////
// Finite State Machine //
//////////////////////////

static currentState_t currentState = WAIT;
void loop() {
    switch (currentState) {        
            
        // WAIT State: Wait until data is recieved from Pi
        case WAIT:
            motionComplete = false;
            if (dataReceived) {
                currentState = DRIVE;
                dataReceived = false;
            }            
            break;
            

        // DRIVE State: Rotate the robot by a given angle or drive forward a given distance motionComplete 
        //              will be set to -127 and sent to the Pi when the robot has moving
        case DRIVE:  
            if (motionType == MOTION_TYPE_ROTATE) {
                digitalWrite(LED_BUILTIN, HIGH);
                motionComplete = control.drive(motionMagnitude, 0);
            }
            else if (motionType == MOTION_TYPE_FORWARD) {
                motionComplete = control.drive(0, motionMagnitude);
            }  
            if (motionComplete) {   
                motionComplete = false;
                currentState = SEND_STATUS;              
            }    
            break;

        // SEND_STATUS State: Tell the Pi that motion is complete
        case SEND_STATUS:
            writeData(MOTION_COMPLETE_SET);            
            currentState = WAIT;
    } 
}

void dumpData() {
    String dataDumpster = "";
    while (Serial.available() > 0) {
        dataDumpster = Serial.read();
    }
}


void writeData(int dataToWrite) {
    dumpData();
    Serial.print(String(dataToWrite) + "\n");
}

void serialEvent(){
    if (Serial.available() > 0) {
        motionType = Serial.readStringUntil(' ').toInt();
        motionMagnitude = Serial.readStringUntil('\n').toInt();
        
        dataReceived = true;    // Flag to indicate if a serial read operation has occured
    }
    // Wait until buffer is empty
    Serial.flush();
}
