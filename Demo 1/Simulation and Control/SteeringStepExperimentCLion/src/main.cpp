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

// Default pins used in DualMotorShield
/*DualMC33926MotorShield::DualMC33926MotorShield()
 * test
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

// Instead of having a lot of duplicate lines, I decided to store left and right variables together.
// Arduino doesn't have std::pair, so I made my own that works with any type T.
template<typename T>
struct Pair {
	T L; T R;
	// I did some serious operator overloading to do math on both elements at once.
	Pair operator+(const T & a) const {			return Pair<T>( {T(L)+a, T(R)+a} );	};
	Pair operator+(const Pair<T> & a) const {	return Pair<T>( {T(L)+a.L, T(R)+a.R} ); };
	Pair operator-(const T & a) const { 		return Pair<T>( {T(L)-a, T(R)-a} ); };
	Pair operator-(const Pair<T> & a) const {	return Pair<T>( {T(L)-a.L, T(R)-a.R} ); };
	Pair operator*(const T & a) const { 		return Pair<T>( {T(L)*a, T(R)*a} ); };
	Pair operator*(const Pair<T> & a) const {	return Pair<T>( {T(L)*a.L, T(R)*a.R} ); };
	Pair operator/(const T & a) const {			return Pair<T>( {T(L)/a, T(R)/a} ); };
	Pair operator/(const Pair<T> & a) const {	return Pair<T>( {T(L)/a.L, T(R)/a.R} );	};
};

// Encoder parameters
const float CPR = 50.0*64.0;	// Total encoder counts per revolution (CPR) of motor shaft
// Castor wheel is the back of the robot // TODO which end of the robot do we want to be the front??
#define ENC_R_WHITE 2 			// Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5  		// Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3			// Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6			// Left motor encoder output A (yellow wire)

// From mini project, pin1 on the Encoder object needs to connect to the white wire on the motor encoder
// for CCW to be positive (facing the wheel)
// TODO I assume the castor wheel is the back
Encoder motorEncR(ENC_R_WHITE, ENC_R_YELLOW); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
Encoder motorEncL(ENC_L_WHITE, ENC_L_YELLOW);
Pair<long> newPosition;     	// Current position reading (counts)


// Experiment parameters
DualMC33926MotorShield motors; // Motor 2 is the right wheel
//TODO since the wheels need to spin in opposite directions to move forward,
// should we invert the power terminals on one of the motors instead of inverting a lot of code?
const long SAMPLE_RATE = 5;     	// Sample rate in ms
const int MAX_SPEED = 400;   	// Desired speed. If using DualMC33926MotorShield.h: max speed = 400
bool motorsSet = false;
unsigned long now = 0;    			// Elapsed time in ms
unsigned long start = 0;  			// Experiment start time
Pair<float> angPos, newAngPos, angVel;
//volatile float angPos.R = 0, angPos.L = 0;     		// position relative to initial position (radians)
//volatile float newAngPos.R = 0, newAngPos.L = 0;     // current position relative to initial position (radians)
//volatile float angVel.R = 0, angVel.L = 0;     		// Current angular velocity (rad/s)

// Steering
float motorDif, motorSum; // Parameters for steering
Pair<int> targetSpeed;
// Given the sum and difference [0,1], set the speed command of each motor
void setMotorValues(float commandDifference, float commandSum);

bool commandReceived = false; 	// flag for new serial input
bool run1, run2;				// flags to indicate which experiment to run
String InputString = ""; 		// a string to hold incoming data

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
			case '1':
				run1 = true;
				run2 = false;
				start = millis();
				break;
			case '2':
				run1 = false;
				run2 = true;
				start = millis();
			case 'E':
				run1 = false;
				run2 = false;
				break;

		}
		commandReceived = false;
	}

	if(run1) {
		// At 1 second, set the motor to target speed
		if(millis()-start >= 1000 && !motorsSet) {
			setMotorValues(0, 1);
			// The right motor needs to rotate in the opposite direction compared to the left motor.
			// instead of inverting the power supply on the right motor, we just need to negate the value we set.
			motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
			motorsSet = true;
		}

		// Get new data every SAMPLE_TIME ms
		if (millis()-start >= now + SAMPLE_RATE) {

			// Adjust elapsed time
			now += SAMPLE_RATE;

			// Read new encoder counts from both motors
			// One needs to be negative! CCW looking into the wheel is positive
			// Left wheel spinning CCW and right wheel spinning CW = forward
			// Right encoder counts need to have the opposite sign
			// TODO should the right wheel have its power supply inverted also?

			newPosition = {motorEncL.read(), -motorEncR.read()};

			// Convert encoder counts to radians
			newAngPos = Pair<float>({float(newPosition.L),float(newPosition.R)}) * float((2.0 * PI) / CPR);

			// Find current angular velocities in rad/s: (x2 - x1) / ∆t
			angVel = ((newAngPos - angPos) * float(1000) ) / float(SAMPLE_RATE);

			// If elapsed time is between 1s and 2s
			if(millis()-start >= 1000 && millis()-start <= 2000) {
				// Print elapsed time, target speed, and angular velocity for each motor
				Serial.print(millis()-start); // elapsed time in ms
				Serial.print("\t");
				Serial.print(MAX_SPEED); // [-400,400] //TODO print target values instead
				Serial.print("\t");
				Serial.print(angVel.R, 7); // Right motor velocity
				Serial.print("\t");
				Serial.print(angPos.R, 7); // Right motor position
				Serial.print("\t");
				Serial.print(angVel.L, 7); // Left motor velocity
				Serial.print("\t");
				Serial.print(angPos.L, 7); // Left motor position
				Serial.println("");
			}

			// Save positions for next loop
			angPos = newAngPos;

		}

		// After 2s, turn off the motors and tell Matlab the experiment is done.
		if(millis()-start > 2000) {
			Serial.println("Finished");
			motors.setSpeeds(0,0);
			motorEncR.write(0);  // Reset encoders
			motorEncL.write(0);
		}
	}
	// TODO implement a rotation experiment
}

void setMotorValues(float commandDifference, float commandSum) {
	Pair<float> target = {0,0};

	// TODO do the math on the handout to get Va,1 and Va,2 from the sum and difference
	// commandSum: A decimal value between -1 and 1 describing the proportional voltage sum applied to left and right motors
	// (Voltage of left motor + voltage of right motor = commandSum. sum = -1 = full reverse, sum = 1 = full forward)
	// commandDifference: A decimal value between -1 and 1 describing the proportional difference in voltages applied to left nd right motors (controls rotation, higher value =

	target.R = (commandSum+commandDifference)/float(2.0);
	target.L = (commandSum-commandDifference)/float(2.0);

	// Scale the ratios to map a max of 1 to a max of 400
	target = target * float(400);

	// Set the global targetSpeed variable with the
	targetSpeed = {int(target.L),int(target.R)};
	//targetSpeed = {targetL,targetR};
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