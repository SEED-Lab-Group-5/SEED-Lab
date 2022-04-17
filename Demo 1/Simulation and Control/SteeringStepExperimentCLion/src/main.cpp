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
//           Motor: 50:1 Metal Gear motor 37Dx70L mm 12V with 64 CPR Encoder (Spur Pinion)
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
//          Raspberry Pi | Arduino
//          GPIO2 (SDA) -> A4
//          GPIO3 (SCL) -> A5
//          GND         -> GND
// test
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
const float CPR = 50.0*64.0;	//!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
#define ENC_R_WHITE 2 			//!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5  		//!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3			//!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6			//!< Left motor encoder output A (yellow wire)

// From mini project, pin1 on the Encoder object needs to connect to the white wire on the motor encoder
// for CCW to be positive (facing the wheel)
// 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
Encoder motorEncR(ENC_R_WHITE, ENC_R_YELLOW); //!< Right motor encoder
Encoder motorEncL(ENC_L_WHITE, ENC_L_YELLOW); //!< Left motor encoder

// Experiment parameters
DualMC33926MotorShield motors; 	//!< Motor 2 is the right wheel
const long SAMPLE_RATE = 10;     //!< Period to wait before measuring and sending new data
const int MAX_SPEED = 400;   	//!< Maximum scaled PWM (if using DualMC33926MotorShield.h, max = 400)
bool motorsSet = false;			//!< Flag indicating if the motors were set to target speeds
unsigned long now = 0;    		//!< Experiment elapsed time (ms)
unsigned long start = 0, start2 = 0;  		//!< Experiment start time (ms)
Pair<float> angPos;				//!< PREVIOUS wheel positions (radians)
Pair<float> newAngPos;			//!< CURRENT wheel positions (radians)
Pair<long> newPosition;     	//!< CURRENT wheel encoder readings (counts)
Pair<float> angVel;				//!< Wheel angular velocities (rad/s)
unsigned long oldTime = 0;		//test
unsigned long currentTime = 0;
float rho_dot = 0;
float phi_dot = 0;
volatile float distanceLeft = 0;
volatile float distanceRight = 0;
const float WHEEL_RADIUS = 2.935;              	// Radius of wheel in inches
const float WHEELBASE = 13.625;                  //Wheelbase measurement in inches
int nLeft = 1;
int nRight = 1;

// Steering
float motorDif, motorSum; 		//!< Parameters for steering
Pair<int> targetSpeed;			//!< Scaled PWM values given to motors.setSpeeds() ranging from -400 to 400
/**
 * setMotorValues() sets the target motor PWM speeds based on commandDifference and commandSum
 * @param commandDifference [in] ∆Va = the desired difference of targetSpeed.L and targetSpeed.R between 0 and 1
 * @param commandSum [in] |Va| = the desired sum of targetSpeed.L and targetSpeed.R between 0 and 1
 */
void setMotorValues(float commandDifference, float commandSum);

bool commandReceived = false; 	//!< flag for new serial input
bool run1 = false, run2 = false;				//!< flags to indicate which experiment to run
String InputString = ""; 		//!< a string to hold incoming data

void setup() {

	Serial.begin(115200);
	InputString.reserve(200);      // reserve 200 bytes for the inputString
	motors.init();                       // Initialize motor
	motors.setSpeeds(0, 0);         // Set motor A and B speeds to 0
	Serial.println("Start!");           // Tell Matlab that Arduino is ready
}

void loop() {
	// Change behavior based on serial input from Matlab
	if (commandReceived) {
		switch (InputString.charAt(0)) {
			case '1': // Run experiment 1 (full forward)
				run1 = true;
				run2 = false;
				motorsSet = false;
				break;
			case '2': // Run experiment 2 (full rotation)
				run1 = false;
				run2 = true;
				motorsSet = false;
				start2 = millis();
				break;
			case 'E': // End all experiments
				run1 = false;
				run2 = false;
				motorsSet = false;
				break;
		}
		commandReceived = false;
	}

	// Experiment 1
	if(run1) {
		// At 1 second, set the motor to target speed
		if(millis() - start >= 1000 && !motorsSet) {
			setMotorValues(0, 400);
			// The right motor needs to rotate in the opposite direction compared to the left motor.
			// instead of inverting the power supply on the right motor, we just need to negate the value we set.
			motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
			motorsSet = true;
		}

		// Get new data every SAMPLE_TIME ms
		if (millis() - start >= now + SAMPLE_RATE) {

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

			/*if(abs(newPosition.R) > CPR){
				nRight = 1+int(floorf(abs(newPosition.R)/(CPR)));
			}else if(abs(newPosition.L) > CPR){
				nLeft = 1+int(floorf(abs(newPosition.L)/(CPR)));
			}*/

			// Find current angular velocities in rad/s: (x2 - x1) / ∆t
			angVel = ((newAngPos - angPos) * float(1000) ) / float(millis()-oldTime);

			rho_dot = WHEEL_RADIUS*(angVel.L + angVel.R)*float(0.5);
			phi_dot = (WHEEL_RADIUS*(angVel.L - angVel.R))/WHEELBASE;

			/*distanceRight = nRight*2*PI*pow(WHEEL_RADIUS,2);
			distanceLeft = nLeft*2*PI*pow(WHEEL_RADIUS,2);*/

			// If elapsed time is between 1s and 2s
			if(millis() - start >= 1000 && millis() - start <= 5000) {
				// Print elapsed time, target speed, and angular velocity for each motor
				Serial.print(millis() - start); // elapsed time in ms
				Serial.print("\t");
				Serial.print(newPosition.L);
				Serial.print("\t");
				Serial.print(newPosition.R);
				Serial.print("\t");
				Serial.print(rho_dot, 4); // forward velocity
				Serial.print("\t");
				Serial.print(phi_dot, 4); // Rotational velocity
				Serial.println("");
			}

			// Save positions for next loop
			angPos = newAngPos;
			oldTime = millis();

		}

		// After 2s, turn off the motors and tell Matlab the experiment is done.
		if(millis() - start > 5000 && run1) {
			Serial.println("Finished1");
			motors.setSpeeds(0,0);
			motorEncR.write(0);  // Reset encoders
			motorEncL.write(0);
			run1 = false;
			//Serial.println("Next!");           // Tell Matlab that Arduino is ready
			//now = 0;
		}
	}
	// Experiment 2
	else if(run2) {

		// At 1 second, set the motor to target speed
		if(millis()-start2 >= 1000 && !motorsSet) {
			setMotorValues(400, 0);
			// The right motor needs to rotate in the opposite direction compared to the left motor.
			// instead of inverting the power supply on the right motor, we just need to negate the value we set.
			motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
			motorsSet = true;
		}

		// Get new data every SAMPLE_TIME ms
		if (millis() - start2 >= now + SAMPLE_RATE) {

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

//			if(abs(newPosition.R) > CPR){
//				nRight = 1+int(floorf(abs(newPosition.R)/(CPR)));
//			}else if(abs(newPosition.L) > CPR){
//				nLeft = 1+int(floorf(abs(newPosition.L)/(CPR)));
//			}

			// Find current angular velocities in rad/s: (x2 - x1) / ∆t
			angVel = ((newAngPos - angPos) * float(1000) ) / float(millis()-oldTime);

			rho_dot = WHEEL_RADIUS*(angVel.L + angVel.R)*float(0.5);
			phi_dot = (WHEEL_RADIUS*(angVel.L - angVel.R))/WHEELBASE;

//			distanceRight = nRight*2*PI*pow(WHEEL_RADIUS,2);
//			distanceLeft = nLeft*2*PI*pow(WHEEL_RADIUS,2);

			// If elapsed time is between 1s and 2s
			if(millis() - start2 >= 1000 && millis() - start2 <= 5000) {
				// Print elapsed time, target speed, and angular velocity for each motor
				Serial.print(millis() - start2); // elapsed time in ms
				Serial.print("\t");
				Serial.print(newPosition.L);
				Serial.print("\t");
				Serial.print(newPosition.R);
				Serial.print("\t");
				Serial.print(rho_dot, 4); // forward velocity
				Serial.print("\t");
				Serial.print(phi_dot, 4); // Rotational velocity
				Serial.println("");
			}

			// Save positions for next loop
			angPos = newAngPos;
			oldTime = millis();

		}
		// After 2s, turn off the motors and tell Matlab the experiment is done.
		if(millis()-start2 > 5000 && run2) {
			run2 = false;
			Serial.println("Finished2");
			motors.setSpeeds(0,0);
			motorEncR.write(0);  // Reset encoders
			motorEncL.write(0);
			// Serial.println("Next!");           // Tell Matlab that Arduino is ready
			//now = 0;
			//start2 = millis();
		}
	}

} // End Loop()

void setMotorValues(float commandDifference, float commandSum) {
	Pair<float> target = {0,0}; //!< Motor PWM outputs

	// TODO do the math on the handout to get Va,1 and Va,2 from the sum and difference
	// commandSum: A decimal value between -1 and 1 describing the proportional voltage sum applied to left and right motors
	// (Voltage of left motor + voltage of right motor = commandSum. sum = -1 = full reverse, sum = 1 = full forward)
	// commandDifference: A decimal value between -1 and 1 describing the proportional difference in voltages applied to left nd right motors (controls rotation, higher value =

	target.R = (commandSum-commandDifference)/float(2.0);
	target.L = (commandSum+commandDifference)/float(2.0);

	// Make sure the speeds are within [-400, 400]
	if(target.R > MAX_SPEED) target.R = MAX_SPEED;
	if(target.L > MAX_SPEED) target.L = MAX_SPEED;
	if(target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if(target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	targetSpeed = {int(target.L),int(target.R)};
	//targetSpeed = {int(400),int(400)};
	// targetSpeed = {,targetR};
}

//!< Reads data sent over serial from Matlab
void serialEvent() {
	while (Serial.available()) {
		char inChar = (char)Serial.read(); // get the new byte:
		float test = (int)Serial.read();
		InputString += inChar; // add it to the inputString:
		// if the incoming character is a newline, set commandReceived = true
		// so the main loop can do something about it.
		if (inChar == '\n') {
			commandReceived = true;
		}
	}
}

