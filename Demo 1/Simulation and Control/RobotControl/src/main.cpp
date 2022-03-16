#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

// TARGETS
float rho = 0, targetRho = 0; 	//!< current and target distances in inches
float phi = 0, targetPhi = 3600; 	//!< current and target angles in radians

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

const float KP_RHO = 41.507628, KI_RHO = 0.000000, KD_RHO = 0.000000; //!< Rho controller constants
const float KP_PHI = 260.542014, KI_PHI = 0.000000, KD_PHI = 0.000000; //!< Phi controller constants
/**
 * Sets motorSum based on error
 * @param current current forward position
 * @param desired desired forward position
 * @param KP Kp value
 * @param KI Ki value
 * @param KD Kd value
 * @return controller output (commandSum)
 */
float controlRho(float current, float desired, float KP, float KI, float KD);
/**
 * Sets motorDif based on error
 * @param current current angular position
 * @param desired desired angular position
 * @param KP Kp value
 * @param KI Ki value
 * @param KD Kd value
 * @return ∆Va (motorDif)
 */
float controlPhi(float current, float desired, float KP, float KI, float KD);

// Control implementation
const long CONTROL_SAMPLE_RATE = 5;                     //!< Controller sample rate in ms
float error, pastErrorRho = 0, pastErrorPhi = 0;        //!< Variables used in calculating control output
float I_rho = 0, I_phi = 0;								//!< Integral controller accumulations
unsigned long currentTime = 0, startTime = 0;           //!< For creating a discrete time controller

// Numeric Constants and Conversions
const float CPR = 50.0*64.0;							//!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
const float WHEEL_RADIUS = 2.9375;              		//!< Measured radius of wheels in inches
const float WHEELBASE = 13.65;                 			//!< Distance between center of wheels in inches
const float RAD_CONVERSION = float(2.0*PI)/CPR;			//!< Scalar to convert counts to radians
const int MAX_SPEED = 400;   							//!< Maximum scaled PWM (max motor speed = 400)
#define ENC_R_WHITE 2 									//!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5  								//!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3									//!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6									//!< Left motor encoder output A (yellow wire)

// Motor Encoder Objects
Encoder motorEncR(ENC_R_WHITE, ENC_R_YELLOW); //!< Right motor encoder
Encoder motorEncL(ENC_L_WHITE, ENC_L_YELLOW); //!< Left motor encoder

// Motor Shield Object
DualMC33926MotorShield motors; 	//!< Motor 2 is the right wheel

Pair<long> newPosition;     	//!< Left and right encoder readings (counts)
bool firstRho = true;			//!< Flag for accurately determining forward position after rotating
float rhoOffset = 0;			//!< Contains initial forward position after rotating
float motorDif, motorSum; 		//!< Parameters for speed control. motorDif [0,400] and motorSum [-400, 400]
Pair<int> targetSpeed;			//!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400

/**
 * setMotorValues() sets the target motor speeds based on commandDifference and commandSum
 * @param commandDifference [in] ∆Va = the desired difference of targetSpeed.L and targetSpeed.R between 0 and 400
 * @param commandSum [in] Va = the desired sum of targetSpeed.L and targetSpeed.R between -400 and 400
 */
void setMotorValues(float commandDifference, float commandSum);

void setup() {
	// Begin serial communication
	Serial.begin(9600);

	// Get initial position and start time
	currentTime = millis();
	startTime = millis();

	// Initialize the motor
	motors.init();
	motors.setSpeeds(0, 0);         // Set motor A and B speeds to 0

}

void loop() {
	// Update encoder counts
	newPosition = {motorEncL.read(), -motorEncR.read()};
	// Find current robot positions
	phi = (WHEEL_RADIUS*RAD_CONVERSION*float(newPosition.L - newPosition.R))/WHEELBASE;
	rho = WHEEL_RADIUS*RAD_CONVERSION*float(newPosition.L + newPosition.R)*float(0.5); // Circumference = 2*PI*r
	// Update motorDif and motorSum with control() every CONTROL_SAMPLE_RATE ms
	if (millis()-startTime >= currentTime + CONTROL_SAMPLE_RATE) {
		// Adjust elapsed time
		currentTime += CONTROL_SAMPLE_RATE;
		// Calculate ∆Va
		motorDif = controlPhi(phi,targetPhi*float(PI)/float(180),KP_PHI,KI_PHI,KD_PHI);
		// only start moving forward when done turning
		if(abs(motorDif) < 20) {
			if(firstRho) { // to mitigate the initial encoder readings from turning
				rhoOffset = WHEEL_RADIUS*RAD_CONVERSION*float(newPosition.L + newPosition.R)*float(0.5);
				firstRho = false;
			}
			// Calculate Va
			motorSum = controlRho(rho-rhoOffset,targetRho,KP_RHO,KI_RHO,KD_RHO);
		}
	}
	// Determine target motor speeds based on motorDif and motorSum using setMotorValues()
	setMotorValues(motorDif,motorSum);
	// Set the motors to the new speeds
	motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
}

// Need two controllers. One to use rho to set commandSum and one to use phi to set commandDifference
float controlRho(float current, float desired, const float KP, const float KI, const float KD) {

	float P = 0, D = 0, output = 0;
	// Calculate error
	error = desired - current;
	// Calculate P component
	P = KP * error;
	// Calculate I component
	I_rho += KI * float(CONTROL_SAMPLE_RATE / 1000.0) * error;
	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorRho) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorRho = error;
		D *= KD;
	} else D = 0;
	// Calculate controller output
	output = P + I_rho + D;
	// Make sure the output is within [-400, 400]
	if(output > 400) output = 400;
	if(output < -400) output = -400;

	// Make sure the output is large enough for the motors to turn
	if(error > 0.5 && output < 80) output = 80;
	if(error < -0.5 && output > -80) output = -80;

	if(millis() < 20000) {
		// Print current values for testing
		Serial.print("\nrho: "); Serial.print(current);
		Serial.print("\ttargetRho: "); Serial.print(desired);
		Serial.print("\terror: "); Serial.print(error);
		Serial.print("\tP: "); Serial.print(P);
		Serial.print("\tI: "); Serial.print(I_phi);
		Serial.print("\tD: "); Serial.print(D);
		Serial.print("\tSum: "); Serial.println(output);
	}

	return output;
}

float controlPhi(float current, float desired, const float KP, const float KI, const float KD) {

	float P = 0, D = 0, output = 0;
	// Calculate error
	error = desired - current;
	// Calculate P component
	P = KP * error;
	// Calculate I component
	I_phi += KI * float(CONTROL_SAMPLE_RATE / 1000.0) * error;
	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorPhi) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorPhi = error;
		D *= KD;
	} else D = 0;
	// Calculate controller output
	output = P + I_phi + D;
	// Make sure the output is within [0, 400]
	if(output > 400) output = 400;
	if(output < -400) output = -400;

	if(error > 0.1 && output < 80) output = 80;
	if(error < -0.1 && output > -80) output = -80;

	if(millis() < 20000) {
		// Print current values for testing
		Serial.print("\nphi: "); Serial.print(current);
		Serial.print("\ttargetPhi: "); Serial.print(desired);
		Serial.print("\terror: "); Serial.print(error);
		Serial.print("\tP: "); Serial.print(P);
		Serial.print("\tI: "); Serial.print(I_phi);
		Serial.print("\tD: "); Serial.print(D);
		Serial.print("\tnewDif: "); Serial.println(output);
	}
	return output;
}

void setMotorValues(float commandDifference, float commandSum) {
	Pair<float> target = {0,0}; 		//!< Motor PWM outputs
	// commandSum: A value between -400 and 400 describing the voltage sum applied to left and right motors
	// (Voltage of left motor + voltage of right motor = commandSum. sum = -400 = full reverse, sum = 400 = full forward)
	// commandDifference: A decimal value between 0 and 400 describing the proportional difference in voltages applied to left nd right motors (controls rotation, higher value =

	target.R = (commandSum-commandDifference)/float(2.0);
	target.L = (commandSum+commandDifference)/float(2.0);

	// Make sure the speeds are within [-400, 400]
	if(target.R > MAX_SPEED) target.R = MAX_SPEED;
	if(target.L > MAX_SPEED) target.L = MAX_SPEED;
	if(target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if(target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	// Update the global targetSpeed variable
	targetSpeed = {int(target.L),int(target.R)};
}