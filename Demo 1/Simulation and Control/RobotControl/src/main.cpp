#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

// TARGETS
float rho_dot = 0, rho = 0, targetRho = 60; // target distance in inches
float phi_dot = 0, phi = 0, targetPhi = 0; // target angle in radians


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
// With rhodot and phidot divided by 800, the values just got wayy to big.
//const float KP_RHO = 63.939045, KI_RHO = 3.795643, KD_RHO = 4.845007;          // Calculated in Matlab (ArduinoFindTf.mlx)
//const float KP_RHO = 34.216994, KI_RHO = 1.093487, KD_RHO = 2.496114;
//const float KP_RHO = 17.108497, KI_RHO = 0.546743, KD_RHO = 1.248057;
//const float KP_RHO = 37.654344, KI_RHO = 1.203335, KD_RHO = 2.746867;
// const float KP_RHO = 20.532959, KI_RHO = 0.358939, KD_RHO = 1.386060;
const float KP_RHO = 41.507628, KI_RHO = 0.000000, KD_RHO = 0.000000;
//const float KP_PHI = 427.167887, KI_PHI = 24.544081, KD_PHI = 26.739142;          // Calculated in Matlab (ArduinoFindTf.mlx)
//const float KP_PHI = 213.583944, KI_PHI = 12.272041, KD_PHI = 13.369571;
//const float KP_PHI = 1232.854805, KI_PHI = 199.132425, KD_PHI = 91.461473;
//const float KP_PHI = 735.404277, KI_PHI = 51.895973, KD_PHI = 0.000000;
//const float KP_PHI = 286.817493, KI_PHI = 51.895973, KD_PHI = 0.000000;
//const float KP_PHI = 285.727771, KI_PHI = 0.000000, KD_PHI = 11.507431;
//const float KP_PHI = 567.842595, KI_PHI = 120.272041, KD_PHI = 0.000000;
//const float KP_PHI = 1000, KI_PHI = 200, KD_PHI = 0.000000;
const float KP_PHI = 260.542014, KI_PHI = 0.000000, KD_PHI = 0.000000;
/**
 * Sets motorSum based on error
 * @param current current position
 * @param desired desired position
 * @param KP Kp value
 * @param KI Ki value
 * @param KD Kd value
 * @return controller output
 */
float controlRho(float current, float desired, float KP, float KI, float KD);
/**
 * Sets motorDif based on error
 * @param current current position
 * @param desired desired position
 * @param KP Kp value
 * @param KI Ki value
 * @param KD Kd value
 * @return controller output
 */
float controlPhi(float current, float desired, float KP, float KI, float KD);

float error, pastErrorRho = 0, pastErrorPhi = 0;        //!< Variables used in calculating control output
float I_rho = 0, I_phi = 0;
unsigned long currentTime = 0, startTime = 0;           //!< For creating a discrete time controller
const int MAX_SPEED = 400;   							//!< Maximum scaled PWM (if using DualMC33926MotorShield.h, max = 400)
const long CONTROL_SAMPLE_RATE = 5;                     //!< Controller sample rate in ms
const long TURN_WAIT = 1000;
volatile float currentAngle = 0, desiredAngle = 0;      //!< position relative to initial position (radians)

const float CPR = 50.0*64.0;	//!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
const float RAD_CONVERSION = float(2.0*PI)/CPR;		//!< Scalar to convert counts to radians
#define ENC_R_WHITE 2 			//!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5  		//!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3			//!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6			//!< Left motor encoder output A (yellow wire)

// From mini project, pin1 on the Encoder object needs to connect to the white wire on the motor encoder
// for CCW to be positive (facing the wheel)
// 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
Encoder motorEncR(ENC_R_WHITE, ENC_R_YELLOW); //!< Right motor encoder
Encoder motorEncL(ENC_L_WHITE, ENC_L_YELLOW); //!< Left motor encoder

DualMC33926MotorShield motors; 	//!< Motor 2 is the right wheel
const long SAMPLE_RATE = 10;     //!< Period to wait before measuring and sending new data

//Pair<float> angPos;				//!< PREVIOUS wheel positions (radians)
//Pair<float> newAngPos;			//!< CURRENT wheel positions (radians)
Pair<long> newPosition;     	//!< CURRENT wheel encoder readings (counts)
//Pair<float> angVel;				//!< Wheel angular velocities (rad/s)
Pair<long> startRead = {0,0};
bool firstRho = true;
float rhoOffset = 0;


const float WHEEL_RADIUS = 2.9375;              //Radius of wheel in inches
const float WHEELBASE = 13.65;                 //Wheelbase measurement in inches

float motorDif, motorSum; 		//!< Parameters for steering. motorDif must be [0,800] and motorSum must be [-800, 800]
Pair<int> targetSpeed;			//!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400

/**
 * setMotorValues() sets the target motor speeds based on commandDifference and commandSum
 * @param commandDifference [in] ∆Va = the desired difference of targetSpeed.L and targetSpeed.R between 0 and 800
 * @param commandSum [in] Va = the desired sum of targetSpeed.L and targetSpeed.R between -800 and 800
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

	// Find current robot position
	// RAD_CONVERSION = float(2.0*PI)/CPR

	phi = (WHEEL_RADIUS*RAD_CONVERSION*float(newPosition.L - newPosition.R))/WHEELBASE;
	rho = WHEEL_RADIUS*RAD_CONVERSION*float(newPosition.L + newPosition.R)*float(0.5); // Circumference = 2*PI*r
	// Update motorDif and motorSum with control() every CONTROL_SAMPLE_RATE ms
	if (millis()-startTime >= currentTime + CONTROL_SAMPLE_RATE) {

		// Adjust elapsed time
		currentTime += CONTROL_SAMPLE_RATE;

		motorDif = controlPhi(phi,targetPhi*float(PI)/float(180),KP_PHI,KI_PHI,KD_PHI);
		// only start moving forward when done turning
		// millis() - startTime >= TURN_WAIT &&
		if(abs(motorDif) < 20) {
			if(firstRho) { // to mitigate the initial encoder readings from turning
				rhoOffset = WHEEL_RADIUS*RAD_CONVERSION*float(newPosition.L + newPosition.R)*float(0.5);
				firstRho = false;
			}
			motorSum = controlRho(rho-rhoOffset,targetRho,KP_RHO,KI_RHO,KD_RHO);
		}

	}

	// Determine target motor speeds based on motorDif and motorSum using setMotorValues()
	setMotorValues(motorDif,motorSum);

	// Set the motors to the new speeds
	motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
	//motors.setSpeeds(400,-400);
}

// Need two controllers. One to use rho to set commandSum and one to use phi to set commandDifference
float controlRho(float current, float desired, const float KP, const float KI, const float KD) {

	float P = 0, D = 0;
	float output = 0;

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

	// Make sure the output is within [-800, 800]
	if(output > 400) output = 400;
	if(output < -400) output = -400;

	if(error > 1 && output < 80) output = 80;
	if(error < -1 && output > -80) output = -80;



	if(millis() < 20000) {
		// Print current values for testing
		Serial.print("\nCurrentRho: "); Serial.print(current);
		Serial.print("\tDesired: "); Serial.print(desired);
		Serial.print("\terror: "); Serial.print(error);
		Serial.print("\tP: "); Serial.print(P);
		Serial.print("\tI: "); Serial.print(I_rho);
		Serial.print("\tD: "); Serial.print(D);
		Serial.print("\tnewSum: "); Serial.println(output);
	}

	return output;
}

float controlPhi(float current, float desired, const float KP, const float KI, const float KD) {

	float P = 0, D = 0;
	float output = 0;

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

	// Make sure the output is within [0, 800]
	if(output > 400) output = 400;
	if(output < -400) output = -400;

	if(error > 0.1 && output < 80) output = 80;
	if(error < -0.1 && output > -80) output = -80;


	if(millis() < 20000) {
		// Print current values for testing
		Serial.print("\nCurrentPhi: "); Serial.print(current);
		Serial.print("\tDesiredPhi: "); Serial.print(desired);
		Serial.print("\terror: "); Serial.print(error);
		Serial.print("\tP: "); Serial.print(P);
		Serial.print("\tI: "); Serial.print(I_phi);
		Serial.print("\tD: "); Serial.print(D);
		Serial.print("\tnewDif: "); Serial.println(output);
	}


	return output;
}

void setMotorValues(float commandDifference, float commandSum) {
	Pair<float> target = {0,0}; //!< Motor PWM outputs

	// commandSum: A value between -800 and 800 describing the voltage sum applied to left and right motors
	// (Voltage of left motor + voltage of right motor = commandSum. sum = -800 = full reverse, sum = 800 = full forward)
	// commandDifference: A decimal value between 0 and 800 describing the proportional difference in voltages applied to left nd right motors (controls rotation, higher value =

	target.R = (commandSum-commandDifference)/float(2.0);
	target.L = (commandSum+commandDifference)/float(2.0);

	// TODO remove this. Wheels are slipping so reducing the max speed should help
	//target = target * 0.5;

	// Make sure the speeds are within [-400, 400]
	if(target.R > MAX_SPEED) target.R = MAX_SPEED;
	if(target.L > MAX_SPEED) target.L = MAX_SPEED;
	if(target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if(target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	// Update the global targetSpeed variable
	targetSpeed = {int(target.L),int(target.R)};
}