//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     Alex Curtis (Primary Author)
// CLASS:	 EENG-350
// GROUP:    5
// TITLE:    Steering Control - Step Response Experiment
//
// FUNCTION: This program runs a step response experiment on a motor and outputs the results over serial.
// 		     Sends a motor speed command of 0, then TARGET_SPEED after 1 second
// SOFTWARE: Requires matlab file "ArduinoFindTf.mlx" To run the experiment, upload this program to the Arduino,
// 			 then start the Matlab script.
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
// RUNNING:  Both libraries below must be installed before running this program.
// RESOURCE: https://github.com/pololu/dual-mc33926-motor-shield
// PURPOSE:  This provides the library "DualMC33926MotorShield.h" which simplifies motor control considerably.
// RESOURCE: https://www.pjrc.com/teensy/td_libs_Encoder.html
// PURPOSE:  This is the "Encoder.h" library used to read the encoders on the motor to determine position.
//////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <DualMC33926MotorShield.h>
#include <Encoder.h>
// This is a test
// TODO change motor pins if needed
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
// TODO assign encoder pins
#define ENC_R_PIN_A 2       			// Right encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define ENC_R_PIN_B 5       			// Right encoder output B (white wire)
#define ENC_L_PIN_A 3					// Left encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define ENC_L_PIN_B 6					// Left encoder output B (white wire)
Encoder motorEncR(ENC_R_PIN_A, ENC_R_PIN_B); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
Encoder motorEncL(ENC_L_PIN_A, ENC_L_PIN_B);
volatile long newPositionR = 0, newPositionL = 0;     	// Current position reading (counts)

// Experiment parameters
DualMC33926MotorShield motors;
const long SAMPLE_RATE = 5;     	// Sample rate in ms
const int TARGET_SPEED = 400;   	// Desired speed. If using DualMC33926MotorShield.h: max speed = 400
bool motorsSet = false;
unsigned long now = 0;    			// Elapsed time in ms
unsigned long start = 0;  			// Experiment start time
volatile float angPosR = 0, angPosL = 0;     		// position relative to initial position (radians)
volatile float newAngPosR = 0, newAngPosL = 0;     // current position relative to initial position (radians)
volatile float angVelR = 0, angVelL = 0;     // Current angular velocity (rad/s)

bool commandReceived = false; 	// flag for new serial input
bool runExperiment = false;

String InputString = ""; // a string to hold incoming data

void setup() {
	Serial.begin(115200);
	InputString.reserve(200);      // reserve 200 bytes for the inputString
	motors.init();                       // Initialize motor
	motors.setSpeeds(0, 0);         // Set motor A and B speeds to 0
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
		if(millis()-start >= 1000 && !motorsSet) {
			motors.setSpeeds(TARGET_SPEED, TARGET_SPEED);
			motorsSet = true;
		}

		// Get new data every SAMPLE_TIME ms
		if (millis()-start >= now + SAMPLE_RATE) {

			// Adjust elapsed time
			now += SAMPLE_RATE;

			// Read new encoder counts from both motors
			newPositionR = -motorEncR.read(); // One needs to be negative! CCW looking into the wheel is positive
			newPositionL = motorEncL.read();  // Left is positive

			// Find angular motor positions
			newAngPosR = float(newPositionR) * float((2.0 * PI) / float(CPR));
			newAngPosL = float(newPositionL) * float((2.0 * PI) / float(CPR));

			// Find current angular velocities in rad/s: (x2 - x1) / ∆t
			angVelR = ((newAngPosR - angPosR) * float(1000) ) / float(SAMPLE_RATE);
			angVelL = ((newAngPosL - angPosL) * float(1000) ) / float(SAMPLE_RATE);

			// If elapsed time is between 1s and 2s
			if(millis()-start >= 1000 && millis()-start <= 2000) {
				// Print elapsed time, target speed, and angular velocity for each motor
				Serial.print(millis()-start); // elapsed time in ms
				Serial.print("\t");
				Serial.print(TARGET_SPEED); // [-400,400]
				Serial.print("\t");
				Serial.print(angVelR, 7); // Right motor velocity
				Serial.print("\t");
				Serial.print(angPosR, 7); // Right motor position
				Serial.print("\t");
				Serial.print(angVelL, 7); // Left motor velocity
				Serial.print("\t");
				Serial.print(angPosL, 7); // Left motor position
				Serial.println("");
			}

			// Save positions for next loop
			angPosR = newAngPosR;
			angPosL = newAngPosL;

		}

		// After 2s, turn off the motors and tell Matlab the experiment is done.
		if(millis()-start > 2000) {
			Serial.println("Finished");
			motors.setSpeeds(0,0);
			motorEncR.write(0);  // Reset encoders
			motorEncL.write(0);
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