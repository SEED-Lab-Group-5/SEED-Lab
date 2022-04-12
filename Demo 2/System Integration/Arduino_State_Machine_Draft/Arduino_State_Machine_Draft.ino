//////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     David Long
// CLASS:    EENG-350
//////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>

#define SLAVE_ADDRESS 0x04

// Define data types needed for finite state machine
typedef enum {START, FOV_ROTATE, FIND_TAPE, TURN_TO_START, CALC_DIST_TO_START, DRIVE_TO_START, CALC_PATH_ANGLE, TURN_INLINE_TO_PATH, CALC_DIST_TO_END, DRIVE_TO_END, STOP} currentState_t;

int data = 0;

// State Flags and transmission codes
bool rotateComplete = false;      // Indicates if robot is done rotating 
#define ROTATE_COMPLETE_SET -127  // Transmitted to Pi when flag is set

bool tapeNotFound = false;        // Indicates if tape was not found in the field of view
#define TAPE_NOT_FOUND_SET -126   // Transmitted from Pi when flag is set

bool motionComplete = false;      // Indicates if robot has stopped moving forward
#define MOTION_COMPLETE_SET -125  // Transmitted to Pi when flag is set

bool flagSent = false;            // Indicates if a flag was sent to the Pi
bool dataReceived = false;        // Indicates if data was read from the Pi


void setup() {
    Serial.begin(115200); // start serial for output
  
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);  
    Serial.println("Ready!");
}
static currentState_t currentState = START;


void loop() {   
    int angleToStart;    // The angle to the start of the line 
    int distanceToStart; // The distance to the start of the line
    int angleToEnd;      // The angle to the end of the line
    int distanceToEnd;   // The distance to the end of the line
    
    
    switch (currentState) {

        // The START state runs at the start of the program, no code takes place
        case START:
            Serial.println("START State");            
            currentState = FOV_ROTATE;
            break;


        // FOV_ROTATE State: Rotate the robot clockwise by slightly over half the field of view (30 degrees) set a flag when rotate is complete
        case FOV_ROTATE:
            rotateComplete = true;  //TODO: set flag true after robot finished rotating

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = FIND_TAPE; 
                flagSent = false;              
            }         
            break;


        case FIND_TAPE:
//            Serial.println("FIND_TAPE State");
            // If the Arduino received data from the Pi
            if (dataReceived) {
                // If the Pi indicated no tape was found in the screen, go back to FOV_ROTATE state
                if (data == TAPE_NOT_FOUND_SET) {
                    Serial.println("Tape Not Found");
                    currentState = FOV_ROTATE;
                // If the Pi indicated that it found tape, store the angle the Arduino received and move to TURN_TO_START state
                } else {
                    angleToStart = data;
                    Serial.print("angleToStart = ");
                    Serial.println(angleToStart);
                    currentState = TURN_TO_START;
                }
                dataReceived = false;   // Reset dataReceived flag
            }
            break;


        case TURN_TO_START:
//            Serial.println("TURN_TO_START State");
            
            rotateComplete = true;  //TODO: set flag true after robot finished rotating

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = CALC_DIST_TO_START; 
                flagSent = false;              
            }
            break;

        case CALC_DIST_TO_START:
//            Serial.println("CALC_DIST_TO_START State");
            // If the Arduino received data from the Pi
            if (dataReceived) {
                distanceToStart = data;
                Serial.print("distanceToStart = ");
                Serial.println(distanceToStart);
                currentState = DRIVE_TO_START;
                dataReceived = false;   // Reset dataReceived flag
            }
            break;


        case DRIVE_TO_START:
//            Serial.println("DRIVE_TO_START State");
            motionComplete = true;  //TODO: set flag true after robot finished moving forward

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = CALC_PATH_ANGLE;
                flagSent = false;              
            }
            break;          


        case CALC_PATH_ANGLE:
//            Serial.println("CALC_PATH_ANGLE State");

            // If the Arduino received data from the Pi
            if (dataReceived) {
                angleToEnd = data;
                Serial.print("angleToEnd = ");
                Serial.println(angleToEnd);
                currentState = TURN_INLINE_TO_PATH;
                dataReceived = false;   // Reset dataReceived flag
            }
            break;   
            

        case TURN_INLINE_TO_PATH:
//            Serial.println("TURN_INLINE_TO_PATH State");/

            rotateComplete = true;  //TODO: set flag true after robot finished rotating

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = CALC_DIST_TO_END;
                flagSent = false;              
            }
            break;

        case CALC_DIST_TO_END:
//            Serial.println("CALC_DIST_TO_END State");

            // If the Arduino received data from the Pi
            if (dataReceived) {
                distanceToEnd = data;
                Serial.print("distanceToEnd = ");
                Serial.println(distanceToEnd);
                currentState = DRIVE_TO_END;
                dataReceived = false;   // Reset dataReceived flag
            }
            break;

        case DRIVE_TO_END:
//            Serial.println("DRIVE_TO_END State");

            motionComplete = true;  //TODO: set flag true after robot finished moving forward

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = STOP;
                flagSent = false;              
            }
            break;

        case STOP:
            break;
    }   
}

// callback for received data
void receiveData(int byteCount){
    dataReceived = true;    // Indicate that the Arduino received data from the Pi
    Serial.println("Data Received");
  
    // While there are still bytes to read, read them and store the most recent to number
    while(Wire.available()) {
        data = Wire.read();   
        
        // Convert to Two's compliment representation
        if (data > 127) {
            data = 256 - data;
            data *= -1; 
        }
        Serial.print("Data: ");
        Serial.println(data);
    }
}


// callback for sending data
void sendData(){
   
    Serial.println("Data Requested");

    // If a write request was received from the Pi
    switch (currentState) {
        case FOV_ROTATE:
        case TURN_TO_START:
        case TURN_INLINE_TO_PATH:
            // Wait until robot has finished rotating
            if (rotateComplete) {
                Wire.write(ROTATE_COMPLETE_SET);
                flagSent = true;
                Serial.println("FLAG Sent");
            }
            break;     
            
        case DRIVE_TO_START: 
        case DRIVE_TO_END:
            // Wait until robot has finished driving forward
            if (motionComplete) {
                Wire.write(MOTION_COMPLETE_SET);
                flagSent = true;
                Serial.println("FLAG Sent");
            }
            break; 
            
       
            
    

    }
    
    
    delay(100);
}
