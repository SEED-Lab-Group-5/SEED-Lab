//#include "DualMC33926MotorShield.h"
#define ENCODER_OPTIMIZE_INTERRUPTS         // Highly Optimized interrupt in background 
#include "Encoder.h"
#include "math.h"

//DualMC33926MotorShield md;

// Increase this value to increase the speed of the motor
int motor1Speed = 40;


Encoder myEnc(3, 2);

const byte ENC_PIN_TWO = 2;     // Encoder input 2
const byte ENC_PIN_ONE = 3;     // Encoder input 1
const byte MOTOR_ENABLE = 4;    // Disables motor outputs when LOW, Toggling resets fault condition
const byte M1VOLT_SIGN = 7;     // Motor 1 direction. LOW = Forward, HIGH = Reverse
const byte M2VOLT_SIGN = 8;     // Motor 2 direction. LOW = Forward, HIGH = Reverse
const byte M1PWM = 9;           // Controls whether PWM voltage is applied to motor 1 or not
const byte M2PWM = 10;          // Controls whether PWM voltage is applied to motor 2 or not
const byte FAULT_STATUS = 12;   // Status flag indicator (LOW indicates fault)

volatile int encCount = 0;      // Initial value of encoder

long encPosition;       ////////NOT USED, Do we need to remove this?//////////

const float RAD_PER_COUNT = (2 * PI) / 3200;    // 0.0019634954 rad/count
float currentAngle;
byte currentQuadrant;

bool angleRead;          // Flag to indicate if a serial read operation has occured
int desiredAngle = 0;   // Angle to move the motor to (Value between 0-3 to be multiplied by PI/2 to find angle to move motor to)    ///// May need to set this to 0 for final implementation, not sure yet ///// 

const float FLOAT_DELTA = 0.05; // How close a float must be to a desired value to be considered equal (actualValue +- FLOAT_DELTA = desiredValue)
const int BRAKE_TIME = 50;      // Time in ms to give motor to stop after being told to do so       ////// May not need this //////

void setup() {  
    // Begin serial communication
    Serial.begin(115200);

    // Pin configuration
    pinMode(ENC_PIN_ONE, INPUT_PULLUP);
    pinMode(ENC_PIN_TWO, INPUT_PULLUP);
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(M1VOLT_SIGN, OUTPUT);                       
    pinMode(M2VOLT_SIGN, OUTPUT);                     
    pinMode(M1PWM, OUTPUT);                 
    pinMode(M2PWM, OUTPUT);                 
    pinMode(FAULT_STATUS, INPUT);                 
}

void loop() {

    // Enable motor
    digitalWrite(MOTOR_ENABLE, HIGH);  
    
    // Read current encoder value and caluclate motor angle.
    encCount = myEnc.read();
    currentAngle = accountForRotations(encCount * RAD_PER_COUNT);
    Serial.println(currentAngle);
    
    // Starting 0 radians (red tape right)
    // If data was received
//    if (angleRead) {
        // Set motor direction to CCW
        digitalWrite(M1VOLT_SIGN, LOW);

        // Move motor to desired angle
        switch (desiredAngle) {
            
            // Motor should go to 0*PI
            case 0:                
                while(currentAngle > (0.0 + FLOAT_DELTA)) {   // While motor 1 is not at 0 radians
                    analogWrite(M1PWM, motor1Speed);         // Start rotating motor 1
                    encCount = myEnc.read();                     
                    currentAngle = accountForRotations(encCount * RAD_PER_COUNT);
                    ////
                    Serial.println(currentAngle);
                    ////                  
                }                
                analogWrite(M1PWM, 0);    // Stop motor
                delay(BRAKE_TIME);        // Give motor a moment to stop        ////// May not need this //////
//                ////
                Serial.println("Break");
//                desiredAngle = 5;
//                ////
                break;
                
            // Motor should go to PI/2
            case 1:                
                while(currentAngle < ((PI / 2) - FLOAT_DELTA) || currentAngle > ((PI / 2) + FLOAT_DELTA)) {     // While motor 1 is not at PI/2 radians
                    analogWrite(M1PWM, motor1Speed);                                                            // Start rotating motor 1
                    encCount = myEnc.read();
                    currentAngle = accountForRotations(encCount * RAD_PER_COUNT);
                }
                analogWrite(M1PWM, 0);    // Stop motor
                delay(BRAKE_TIME);        // Give motor a moment to stop        ////// May not need this //////
                break;
                
            // Motor should go to PI
            case 2:
                while(currentAngle < (PI - FLOAT_DELTA) || currentAngle > (PI + FLOAT_DELTA)) {     // While motor 1 is not at PI radians
                    analogWrite(M1PWM, motor1Speed);                                                // Start rotating motor 1
                    encCount = myEnc.read();
                    currentAngle = accountForRotations(encCount * RAD_PER_COUNT);
                }
                analogWrite(M1PWM, 0);    // Stop motor
                delay(BRAKE_TIME);        // Give motor a moment to stop        ////// May not need this //////
                break;
                
            // Motor should go to 3PI/2
            case 3: 
                while(currentAngle < ((3 * PI/2) - FLOAT_DELTA) || currentAngle > ((3 * PI/2) + FLOAT_DELTA)) {     // While motor 1 is not at 3PI/2 radians
                    analogWrite(M1PWM, motor1Speed);                                                                // Start rotating motor 1
                    encCount = myEnc.read();
                    currentAngle = accountForRotations(encCount * RAD_PER_COUNT);
                }
                analogWrite(M1PWM, 0);    // Stop motor
                delay(BRAKE_TIME);        // Give motor a moment to stop        ////// May not need this //////
                break;

            // If desiredAngle is not 0-3, do nothing
            default:                
                break;
        }
//        angleRead = false;   // Reset data flag
//    }
}

// Adjust current angle to be between 0 and 2PI radians by accounting for multiple rotations
float accountForRotations(float currentAngle) {
    if (currentAngle > 2 * PI ) {
        int rotations = int(currentAngle / (2 * PI));
        currentAngle -= rotations * 2 * PI;
    }
    return currentAngle;  
}


// The Pi is attempting to write to the Arduino over serial
//void serialEvent() {    
//    // If there is data in the serial buffer
//    if (Serial.available() > 0) {
//        desiredAngle = Serial.read();
//        angleRead = true;    // Flag to indicate if a serial read operation has occured
//    }
//    // Wait until buffer is empty
//    Serial.flush();
//}