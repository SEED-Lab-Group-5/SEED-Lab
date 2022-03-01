//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     Alex Curtis (Primary Author)
// CLASS:    EENG-350
// GROUP:    5
// TITLE:    Motor Control Mini-Project - Step Response Experiment
//
// FUNCTION: This program runs a step response experiment on a motor and outputs the results over serial.
//           Sends a motor speed command of 0, then TARGET_SPEED after 1 second
// SOFTWARE: Requires matlab file "ArduinoFindTf.mlx" To run the experiment, upload this program to the Arduino,
//           then start the Matlab script.
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
// RUNNING:  Both libraries below must be installed before running this program. Use matlab to start.
// RESOURCE: https://github.com/pololu/dual-mc33926-motor-shield
// PURPOSE:  This provides the library "DualMC33926MotorShield.h" which simplifies motor control considerably.
// RESOURCE: https://www.pjrc.com/teensy/td_libs_Encoder.html
// PURPOSE:  This is the "Encoder.h" library used to read the encoders on the motor to determine position.
//////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <Encoder.h>


// Default pins used in DualMotorShield
/*DualMC33926MotorShield::DualMC33926MotorShield()
{
  //Pin map
  _nD2 = 4;
  _M1DIR = 7;
  _M1PWM = 9;
  _M2DIR = 8;
  _M2PWM = 10;
  _nSF = 12;
  _M1FB = A0;
  _M2FB = A1;
}*/

// Encoder parameters
const float CPR = 50.0*64.0;        // Total encoder counts per revolution (CPR) of motor shaft
#define ENC_PIN_A 3             // Encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define ENC_PIN_B 2             // Encoder output B (white wire)
Encoder motorEnc(ENC_PIN_A,ENC_PIN_B); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
long initialPosition = 0;           // initial position of motor (counts)
volatile long position = 0;         // Previous position reading (counts)
volatile long newPosition = 0;      // Current position reading (counts)

// Experiment parameters
DualMC33926MotorShield motor;
const long SAMPLE_RATE = 5;       // Sample rate in ms
const int TARGET_SPEED = 400;     // Desired speed. If using DualMC33926MotorShield.h: max speed = 400
bool motorSet = false;
unsigned long now = 0;          // Elapsed time in ms
unsigned long start = 0;        // Experiment start time
volatile float angPosition = 0;     // position relative to initial position (radians)
volatile float angVelocity = 0;     // Current angular velocity (rad/s)

bool commandReceived = false; // flag for new serial input
bool runExperiment = false;

String InputString = ""; // a string to hold incoming data

void setup() {
  Serial.begin(115200);
  InputString.reserve(200);      // reserve 200 bytes for the inputString
  motor.init();                       // Initialize motor
  motor.setM1Speed(0);         // Set motor speed to 0
  Serial.println("Ready!");           // Tell Matlab that Arduino is ready
}

void loop() {
  // Change behavior based on serial input
  if (commandReceived) {
    switch (InputString.charAt(0)) {
    case 'S':
      runExperiment = true;
      start = millis();
      break;
    case 'E':
      runExperiment = false;
      break;
    }
    commandReceived = false;
  }

  if(runExperiment) {
    // At 1 second, set the motor to target speed
    if(millis()-start >= 1000 && !motorSet) {
      motor.setM1Speed(TARGET_SPEED);
      motorSet = true;
    }

    // Get new data every SAMPLE_TIME ms
    if (millis()-start >= now + SAMPLE_RATE) {

      // Adjust elapsed time
      now += SAMPLE_RATE;

      // Read encoders
      newPosition = motorEnc.read();

      // Find current angular motor position relative to initial position
      angPosition = float( newPosition - initialPosition) * float( (2.0 * PI) / float(CPR));

      // Find current angular velocity in rad/s: (x2 - x1) / ∆t
      angVelocity = ( (float((newPosition - initialPosition) - (position - initialPosition))*float( (2.0 * PI)/CPR) )*float(1000) ) / float(SAMPLE_RATE);

      // If elapsed time is between 1s and 2s
      if(millis()-start >= 1000 && millis()-start <= 2000) {
        // Print elapsed time, target speed, angular velocity
        Serial.print(millis()-start); // Prints elapsed time in ms
        Serial.print("\t");
        Serial.print((TARGET_SPEED) * 51 / 80); // Same conversion used in the motor library for analogWrite()
        Serial.print("\t");
        Serial.print(angVelocity,7);
        Serial.print("\t");
        Serial.print(angPosition,7);
        Serial.println("");
      }
      // Update position with the new position
      position = newPosition;

    }

    // After 2s, turn off the motor and tell Matlab the experiment is done.
    if(millis()-start > 2000) {
      Serial.println("Finished");
      motor.setM1Speed(0);
      motorEnc.write(0);  // Reset encoders
    }
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    InputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      commandReceived = true;
    }
  }
}
