// Alex Curtis
// Final Controller for Demo 1

//TODO fix the localization. Angle is very inaccurate

#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

// TARGETS
float rho = 0, targetRho = 0; 		//!< current and target distances in inches
float phi = 0, targetPhi = 360; 	//!< current and target angles in radians

// Instead of creating dedicated left and right variables, I made a Pair type that has a left and right element
// Operator overloading allows me to perform math operations on both elements at the same time (like multiplying both by a scalar)
template<typename T>
struct Pair {
	T L; T R;
	Pair operator+(const T & a) const {			return Pair<T>( {T(L)+a, T(R)+a} );	};
	Pair operator+(const Pair<T> & a) const {	return Pair<T>( {T(L)+a.L, T(R)+a.R} ); };
	Pair operator-(const T & a) const { 		return Pair<T>( {T(L)-a, T(R)-a} ); };
	Pair operator-(const Pair<T> & a) const {	return Pair<T>( {T(L)-a.L, T(R)-a.R} ); };
	Pair operator*(const T & a) const { 		return Pair<T>( {T(L)*a, T(R)*a} ); };
	Pair operator*(const Pair<T> & a) const {	return Pair<T>( {T(L)*a.L, T(R)*a.R} ); };
	Pair operator/(const T & a) const {			return Pair<T>( {T(L)/a, T(R)/a} ); };
	Pair operator/(const Pair<T> & a) const {	return Pair<T>( {T(L)/a.L, T(R)/a.R} );	};
};

// Controller Parameters
const float KP_RHO = 35.638836, KI_RHO = 17.532965, KD_RHO = 0.000000; 	//!< Rho controller constants
const float KP_PHI = 238.778998, KI_PHI = 124.273019, KD_PHI = 0.000000; 	//!< Phi controller constants

const long CONTROL_SAMPLE_RATE = 5;                     //!< Controller sample rate in ms
float error, pastErrorRho = 0, pastErrorPhi = 0;        //!< Variables used in calculating control output
float I_rho = 0, I_phi = 0;								//!< Integral controller accumulations
unsigned long currentTime = 0, startTime = 0;           //!< For creating a discrete time controller

/**
 * Sets motorSum based on error
 * @param current current forward counts
 * @param desired desired forward counts
 * @param KP Kp value
 * @param KI Ki value
 * @param KD Kd value
 * @return Va,L + Va,R (motorSum)
 */
float controlRho(float current, float desired, float KP, float KI, float KD);

/**
 * Sets motorDif based on error
 * @param current current angular counts
 * @param desired desired angular counts
 * @param KP Kp value
 * @param KI Ki value
 * @param KD Kd value
 * @return ???Va (motorDif)
 */
float controlPhi(float current, float desired, float KP, float KI, float KD);

// Numeric Constants and Conversions
const float CPR = 50.0*64.0;							//!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
const float RADIUS = 2.9375;              		//!< Measured radius of wheels in inches
const float BASE = 13.65; // WAS 13.8                 			//!< Distance between center of wheels in inches
const float RAD_CONVERSION = float(2.0*PI)/CPR;			//!< Scalar to convert counts to radians
const int MAX_SPEED = 400;   							//!< Maximum scaled PWM (max motor speed = 400)
const int MIN_SPEED = 84;								//!< Minimum scaled PWM
#define ENC_R_WHITE 2 									//!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5  								//!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3									//!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6									//!< Left motor encoder output A (yellow wire)

// Motor Encoder Objects
Encoder EncR(ENC_R_WHITE, ENC_R_YELLOW); //!< Right motor encoder
Encoder EncL(ENC_L_WHITE, ENC_L_YELLOW); //!< Left motor encoder

// Motor Shield Object
DualMC33926MotorShield motors; 	//!< Motor 2 is the right wheel

Pair<long> counts;     	//!< Left and right encoder readings (counts)
bool firstRho = true;			//!< Flag for accurately determining forward counts after rotating
float rhoOffset = 0;			//!< Contains initial forward counts after rotating
float motorDif, motorSum; 		//!< Parameters for speed control. motorDif [-400,400] and motorSum [-400, 400]
bool rotating = true;			//!< Flag indicating the robot is currently turning
Pair<int> targetSpeed;			//!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400

/**
 * setMotorValues() Determines Va,L and Va,R based on dif and sum
 * @param dif ???Va = the target difference between Va,L and Va,R
 * @param sum Va = the target sum of Va,L and Va,R
 */
void setMotorValues(float dif, float sum);
/**
 * Turns on the LED if the target positions were reached
 */
void showStatus();

void setup() {
	pinMode(LED_BUILTIN, OUTPUT);

	// Begin serial communication
	Serial.begin(9600);

	// Get initial values of currentTime and startTime
	currentTime = millis();
	startTime = millis();

	// Initialize the motor object
	motors.init();

	// Set left and right motor speeds to 0
	motors.setSpeeds(0, 0);
}

void loop() {

	// Update encoder counts
	counts = {EncL.read(), -EncR.read()};
	// Find current robot positions
	phi = (RADIUS * RAD_CONVERSION * float(counts.L - counts.R)) / BASE;
	rho = RADIUS * RAD_CONVERSION * float(counts.L + counts.R) * float(0.5);

	// Update motorDif and motorSum with control() every CONTROL_SAMPLE_RATE ms
	if (millis()-startTime >= currentTime + CONTROL_SAMPLE_RATE) {
		// Determine next time to update motorDif and motorSum
		currentTime += CONTROL_SAMPLE_RATE;
		// Calculate ???Va
		motorDif = controlPhi(phi,targetPhi*float(PI)/float(180),KP_PHI,KI_PHI,KD_PHI);

		rotating = abs(motorDif) >= 20;
		if(rotating) Serial.print("Rotating\t");

		/*
		 * Do initial rotation
		 * If rotation needs to be adjusted: stop moving forward
		 */
		// When the robot finishes rotating, start moving forward
		if(!rotating) {
			if(firstRho) { // When the robot finishes the first rotation, set the initial forward counts
				rhoOffset = RADIUS * RAD_CONVERSION * float(counts.L + counts.R) * float(0.5);
				firstRho = false;
			}
			// Calculate Va
			motorSum = controlRho(rho-rhoOffset,targetRho,KP_RHO,KI_RHO,KD_RHO);
		}

	}

	if(rotating) motorSum = 0;
	else motorDif = 0;

	// Determine Va,L and Va,R based on motorDif and motorSum
	setMotorValues(motorDif,motorSum);
	// Set the motors to the new speeds
	motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
	showStatus();
}

// Need two controllers. One to use rho to set commandSum and one to use phi to set commandDifference
float controlRho(float current, float desired, const float KP, const float KI, const float KD) {
	float P = 0, D = 0, output = 0;
	// Calculate error
	error = desired - current;

	// Give I some help if the error changes sign
	if(error < 0 && I_rho > 0) I_rho = 0;
	if(error > 0 && I_rho < 0) I_rho = 0;

	// If the error is really small or really big, clear the accumulated I to prevent overshoot
	if(abs(error) <= 0.001 || abs(error) >= 5) I_rho = 0;

	// TODO this is new:
	else if(error >= 0.1 && I_rho < MIN_SPEED) I_rho = MIN_SPEED;
	else if(error <= -0.1 && I_rho > -MIN_SPEED) I_rho = -MIN_SPEED;

	// Calculate P component
	P = KP * error;
	// Calculate I component
	I_rho += KI * float(CONTROL_SAMPLE_RATE*0.01) * error; // I needs to accumulate faster when it needs to correct itself

	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorRho) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorRho = error;
		D *= KD;
	} else D = 0;
	// Calculate total controller output
	output = P + I_rho + D;
	// Make sure the output is within [-MAX_SPEED, MAX_SPEED]
	if(output > MAX_SPEED) output = MAX_SPEED;
	if(output < -MAX_SPEED) output = -MAX_SPEED;

	// Make sure the output is large enough if the error is significant enough.
	// The magnitude of each motor speed must be greater than ~40 for it to turn
	//if(error > 0 && output < MIN_SPEED) output = MIN_SPEED;
	//if(error < 0 && output > -MIN_SPEED) output = -MIN_SPEED;

	// Print current controller values for testing
	//Serial.print("\nrho: "); Serial.print(current);
	//Serial.print("\ttargetRho: "); Serial.print(desired);
	Serial.print("\tRho error: "); Serial.print(error,5);
	Serial.print("\tP: "); Serial.print(P);
	Serial.print("\tI: "); Serial.print(I_phi);
	//Serial.print("\tD: "); Serial.print(D);
	Serial.print("\tnewSum: "); Serial.print(output);

	// Return the updated motorSum
	return output;
}

float controlPhi(float current, float desired, const float KP, const float KI, const float KD) {

	float P = 0, D = 0, output = 0;

	// Calculate error
	error = desired - current;

	// Calculate P component
	P = KP * error;

	// TODO made some changes to the thresholds since 0.1 radians is significant
	// If the error is really small or really big, clear the accumulated I to prevent overshoot
	if(abs(error) <= 0.0001 || abs(error) >= 15*PI/180) I_phi = 0;

	// TODO this is new:
	else if(error >= 0.05 && I_phi < MIN_SPEED) I_phi = MIN_SPEED;
	else if(error <= -0.05 && I_phi > -MIN_SPEED) I_phi = -MIN_SPEED;

	// Give I some help if the error changes sign
	if(error < 0 && I_phi > 0) I_phi = 0;
	if(error > 0 && I_phi < 0) I_phi = 0;

	// Calculate I component
	I_phi += KI * float(CONTROL_SAMPLE_RATE*0.01) * error;

	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorPhi) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorPhi = error;
		D *= KD;
	} else D = 0;

	// Calculate total controller output
	output = P + I_phi + D;

	// Make sure the output is within [-MAX_SPEED, MAX_SPEED]
	if(output > MAX_SPEED) output = MAX_SPEED;
	if(output < -MAX_SPEED) output = -MAX_SPEED;

	// Make sure the output is large enough. Each motor speed must be greater than ~40 for the motor to move
	//if(error > 0 && output < MIN_SPEED) output = MIN_SPEED;
	//if(error < 0 && output > -MIN_SPEED) output = -MIN_SPEED;

	// Print current values for testing
//	Serial.print("\nphi: "); Serial.print(current);
//	Serial.print("\ttargetPhi: "); Serial.print(desired);
	Serial.print("\tPhi error: "); Serial.print(error*(180/PI),5);
	Serial.print("\tP: "); Serial.print(P);
	Serial.print("\tI: "); Serial.print(I_phi);
//	Serial.print("\tD: "); Serial.print(D);
	Serial.print("\tnewDif: "); Serial.println(output);

	// Return the updated motorDif
	return output;
}

void setMotorValues(float dif, float sum) {
	Pair<float> target = {0,0}; 		//!< Motor outputs

	target.R = (sum - dif) / float(2.0);
	target.L = (sum + dif) / float(2.0);
	// TODO should I offset the targets?
//	if(target.L < 0) target.L -= 1.58884;
//	if(target.L > 0) target.L += 1.58884;

	// Make sure the new speeds are within [-MAX_SPEED, MAX_SPEED]
	if(target.R > MAX_SPEED) target.R = MAX_SPEED;
	if(target.L > MAX_SPEED) target.L = MAX_SPEED;
	if(target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if(target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	// Update the global targetSpeed variable
	// Average Difference (L-R): 1.58884
	targetSpeed = {int(target.L),int(target.R)};
}

void showStatus() {
	if(targetSpeed.R < 20 && targetSpeed.L < 20) {
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
	} else {
		digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
	}
}