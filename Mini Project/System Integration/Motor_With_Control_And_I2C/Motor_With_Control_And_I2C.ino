//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     Trevor Olsen (Primary Author) Modified by David Long 
// CLASS:	 EENG-350
// GROUP:    5
// TITLE:    Motor Control Mini-Project
//
// FUNCTION: The Function is to control a motor to turn to a given point in a circle pi/2 radians apart, and 
//           implement a control algorithm to resist changes if external force is applied to the wheel.
//
// HARDWARE: Components:
//           Arduino Uno + MC33926 motor driver shield 
//           Motor: 50:1 Metal Gearmotor 37Dx70L mm 12V with 64 CPR Encoder (Spur Pinion)
//           Battery: 7.2V capable of 3.2A 
//           Voltage Regulator: HCW-M635
//
//           Arduino and Motor Controller Connection Guide:
//              - Power Arduino through USB
//              - Power MC33926 through Input phoenix connector from output of voltage regulator
//              - Power in to the voltage regulator from the battery 
//              - Connect M1 output phoenix connector to the black and red pins on the motor 
//              - Connect 5V+(Blue) and GND(Green) to the center two pins of the motor cable to power the encoder
//              - Connect Arduino pins 2 and 3 (both interrupt pins) to white and yellow pins on the motor cable as the encoder pins 
//          
//          I2C Connection Guide: 
//          Rasberryi Pi | Arduino
//          GPIO2 (SDA) -> A4
//          GPIO3 (SCL) -> A5
//          GND         -> GND
//
// RUNNING:  Execution instructions for your program
// RESOURCE: Link to any resource you used
// PURPOSE:  What the resource was used for
// RESOURCE: Link to any resource you used
// PURPOSE:  What the resource was used for
//////////////////////////////////////////////////////////////////////


//#include "DualMC33926MotorShield.h"
#define ENCODER_OPTIMIZE_INTERRUPTS         // Highly Optimized interrupt in background 
#define SLAVE_ADDRESS 0x04
#include "Encoder.h"
#include "Arduino.h"
#include "math.h"
#include <Wire.h>

// Increase this value to increase the speed of the motor
int motor1Speed = 40;
const float KP = 1.5722, KI = 0.0773, KD = 0;
unsigned long Ts = 0, Tc = 0;
float error, pastError = 0;

float accountForRotations(float currentAngle);
int control(float current, float desired);
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

const float RAD_PER_COUNT = (2 * PI) / 3200;    // 0.0019634954 rad/count
float currentAngle;
byte currentQuadrant;

bool angleRead;                 // Flag to indicate if a serial read operation has occured
int desiredAngleCoeff = 0;      // Angle to move the motor to (Value between 0-3 to be multiplied by PI/2 to find angle to move motor to)

const float FLOAT_DELTA = 0.01; // How close a float must be to a desired value to be considered equal (actualValue +- FLOAT_DELTA = desiredValue)
const int BRAKE_TIME = 50;      // Time in ms to give motor to stop after being told to do so     

const unsigned long SAMPLE_TIME = 1000;  // Number of milliseconds between data receive from Pi
unsigned long prevSampleTime = 0;        // Time the last data receive occured

void setup() {  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
    
  // Begin serial communication
  Serial.begin(9600);

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
    
  // Read current encoder value and calculate motor angle.
  encCount = myEnc.read();
  Tc = millis(); // Update current time
  
  if(encCount > 3200) encCount -= 3200;
  currentAngle = encCount * RAD_PER_COUNT;
      
  while (currentAngle < (desiredAngleCoeff * (PI / 2) - FLOAT_DELTA) || currentAngle > (desiredAngleCoeff * (PI / 2) + FLOAT_DELTA)) {     // While motor 1 is not at desired angle

    encCount = myEnc.read();
    Tc = millis(); // Update current time
    
    if(encCount > 3200) encCount -= 3200;
    currentAngle = encCount * RAD_PER_COUNT;
    
    motor1Speed = control(currentAngle, float(desiredAngleCoeff) * float((PI / 2)) );

    if(motor1Speed < 0) { // If the controller says it needs to go backwards
      digitalWrite(M1VOLT_SIGN, HIGH); // Set motor direction to CW
      motor1Speed = abs(motor1Speed);  // restore a positive sign
    }
    else {
      digitalWrite(M1VOLT_SIGN, LOW);  // Set motor direction to CCW
    }
    // Set the motor speed
    analogWrite(M1PWM, motor1Speed);   // Start rotating motor 1
  }
}

int control(float current, float desired) {
  float I = 0; 
  float D = 0;
    
  int cOutput, maxOutput = 255;   // or 400 if using the motorShield header
  
  error = current - desired;      // Update error
  Serial.println(error);

  if (Ts > 0) {
    // Calculate D
    D = (error - pastError) / float(Ts);
    pastError = error;
  } 
  else {
    D = 0;
  }

  // Calculate I
  I += float(Ts) * error;

  // Calculate controller output
  cOutput = KP * error + KI * I + KD * D;

  // If calculated controller output is higher than the maximum output,
  // set output to the maximum output with matching sign
  if (abs(cOutput) > maxOutput) {
    if (cOutput < 0) {
      cOutput = -1 * maxOutput;
    }
    else {
      cOutput = maxOutput;
    }
  }

  // Set Ts
  Ts = millis() - Tc;
  Tc = millis();

  //TODO scale output to [-255, 255]
  cOutput = cOutput * 5;
  Serial.print("cOutput: ");
  Serial.println(cOutput);
  return cOutput;
}

// Adjust current angle to be between 0 and 2PI radians by accounting for multiple rotations
float accountForRotations(float currentAngle) {
  if (currentAngle > 2 * PI ) {
    int rotations = int(currentAngle / (2 * PI));
    currentAngle -= rotations * 2 * PI;
  }
  return currentAngle;  
}

// callback for received data
void receiveData(int byteCount) { 
  // While there are still bytes to read, read them and store the most recent to number
  while (Wire.available()) {
    int nextAngleCoeff = Wire.read(); 
    unsigned long currSampleTime = millis();
    // If time since previous sample exceeds SAMPLE_TIME receive a new input
    if (currSampleTime - prevSampleTime > SAMPLE_TIME) {
      desiredAngleCoeff = nextAngleCoeff;
      angleRead = true;
      prevSampleTime = currSampleTime;
      Serial.print("Desired Angle: ");       
      Serial.println(desiredAngleCoeff);
    }
  }  
}
