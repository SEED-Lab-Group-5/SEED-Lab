#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include <Wire.h>

#define ENCODER_OPTIMIZE_INTERRUPTS

// TARGETS
float rho = 0, targetRho = 0;        //!< current and target distances in inches
float phi = 0, targetPhi = 0;        //!< current and target angles in radians

bool tapeFound = false;         //flag to know whether to search for the tape or not
bool runState();
void scanForTape();
void initialCameraRead();
void encoderReset();

// Instead of creating dedicated left and right variables, I made a Pair type that has a left and right element
// Operator overloading allows me to perform math operations on both elements at the same time (like multiplying both by a scalar)
template<typename T>
struct Pair {
	T L;
	T R;
	Pair operator+(const T &a) const { return Pair<T>({T(L) + a, T(R) + a}); };
	Pair operator+(const Pair<T> &a) const { return Pair<T>({T(L) + a.L, T(R) + a.R}); };
	Pair operator-(const T &a) const { return Pair<T>({T(L) - a, T(R) - a}); };
	Pair operator-(const Pair<T> &a) const { return Pair<T>({T(L) - a.L, T(R) - a.R}); };
	Pair operator*(const T &a) const { return Pair<T>({T(L) * a, T(R) * a}); };
	Pair operator*(const Pair<T> &a) const { return Pair<T>({T(L) * a.L, T(R) * a.R}); };
	Pair operator/(const T &a) const { return Pair<T>({T(L) / a, T(R) / a}); };
	Pair operator/(const Pair<T> &a) const { return Pair<T>({T(L) / a.L, T(R) / a.R}); };
};

// Controller Parameters
const float KP_RHO = 41.507628, KI_RHO = 2, KD_RHO = 0.000000;    //!< Rho controller constants
const float KP_PHI = 260.542014, KI_PHI = 5, KD_PHI = 0.000000;    //!< Phi controller constants

const long CONTROL_SAMPLE_RATE = 5;                     //!< Controller sample rate in ms
float error, pastErrorRho = 0, pastErrorPhi = 0;        //!< Variables used in calculating control output
float I_rho = 0, I_phi = 0;                                //!< Integral controller accumulations
unsigned long currentTime = 0, startTime = 0;           //!< For creating a discrete time controller

float controlRho(float current, float desired, float KP, float KI, float KD);
float controlPhi(float current, float desired, float KP, float KI, float KD);

// Numeric Constants and Conversions
const float CPR = 50.0 *64.0;                            //!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
const float RADIUS = 2.9375;                            //!< Measured radius of wheels in inches
const float BASE = 13.8;                                //!< Distance between center of wheels in inches
const float RAD_CONVERSION = float(2.0 * PI) / CPR;            //!< Scalar to convert counts to radians
const int MAX_SPEED = 400;                            //!< Maximum scaled PWM (max motor speed = 400)
const int MIN_SPEED = 84;                                //!< Minimum scaled PWM
#define ENC_R_WHITE 2                                    //!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5                                //!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3                                    //!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6                                    //!< Left motor encoder output A (yellow wire)

// Motor Encoder Objects
Encoder EncR(ENC_R_WHITE, ENC_R_YELLOW); //!< Right motor encoder
Encoder EncL(ENC_L_WHITE, ENC_L_YELLOW); //!< Left motor encoder

// Motor Shield Object
DualMC33926MotorShield motors;    	//!< Motor 2 is the right wheel

Pair<long> counts;                	//!< Left and right encoder readings (counts)
bool firstRho = true;            	//!< Flag for accurately determining forward counts after rotating
float rhoOffset = 0;            	//!< Contains initial forward counts after rotating
float motorDif, motorSum;        	//!< Parameters for speed control. motorDif [-400,400] and motorSum [-400, 400]
bool rotating = true;            	//!< Flag indicating the robot is currently turning
bool driving = true;            	//!< Flag indicating the robot is currently moving forward
Pair<int> targetSpeed;            	//!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400

/**
 * computeSpeeds() Determines Va,L and Va,R based on dif and sum
 * @param dif ∆Va = the target difference between Va,L and Va,R
 * @param sum Va = the target sum of Va,L and Va,R
 */
void computeSpeeds(float dif, float sum);

void setup() {

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
    //testing
    targetRho = Serial.available();

    //testing end
	scanForTape();
	runState();
}

void encoderReset(){
    EncR.write(0);  // Reset encoders
    EncL.write(0);
}
void getPositions() {
	// Update encoder counts
	counts = {EncL.read(), -EncR.read()};
	// Find current robot positions
	phi = (RADIUS * RAD_CONVERSION * float(counts.L - counts.R)) / BASE;
	rho = RADIUS * RAD_CONVERSION * float(counts.L + counts.R) * float(0.5);
}

void initialCameraRead() {
	//TODO read from camera whether tape is seen or not
    while (Serial.available() == 0){
        int camInput = Serial.read();
        if (camInput = 1){
            tapeFound = true;
        }else{
            tapeFound = false;
        }
    }
}

void scanForTape() {
	byte increment = 1;
	initialCameraRead();

	// Update encoder counts
	counts = {EncL.read(), -EncR.read()};
	// Find current robot positions
	phi = (RADIUS * float(counts.L - counts.R)) / BASE; //removed RAD_CONVERSION to have degree
	targetSpeed = {100, 100};

	while (!tapeFound && phi < 2 * PI) {
		while (phi < (increment * 20)) {
			motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
		}
		initialCameraRead();
		increment++;
	}
}

Pair<float> computeControllers() {
	// Update motorDif and motorSum with control() every CONTROL_SAMPLE_RATE ms
	if (millis() - startTime >= currentTime + CONTROL_SAMPLE_RATE) {
		// Determine next time to update motorDif and motorSum
		currentTime += CONTROL_SAMPLE_RATE;
		// Calculate ∆Va
		motorDif = controlPhi(phi, targetPhi * float(PI) / float(180), KP_PHI, KI_PHI, KD_PHI);

		rotating = abs(motorDif) >= 20;
		if (rotating) Serial.print("Rotating\t");

		/*
		 * Do initial rotation
		 * If rotation needs to be adjusted: stop moving forward
		 */
		// When the robot finishes rotating, start moving forward
		if (!rotating) {
			if (firstRho) { // When the robot finishes the first rotation, set the initial forward counts
				rhoOffset = RADIUS * RAD_CONVERSION * float(counts.L + counts.R) * float(0.5);
				firstRho = false;
			}
			// Calculate Va
			motorSum = controlRho(rho - rhoOffset, targetRho, KP_RHO, KI_RHO, KD_RHO);
		}
	}

	if (rotating) motorSum = 0;
	else motorDif = 0;

	return {motorDif,motorSum};
}

bool runState() {

	getPositions();
	Pair<float> controlOutput = {0,0};

	controlOutput = computeControllers();
	motorDif = controlOutput.L;
	motorSum = controlOutput.R;

	// Determine Va,L and Va,R based on motorDif and motorSum
	computeSpeeds(motorDif, motorSum);
	// Set the motors to the new speeds
	motors.setSpeeds(targetSpeed.L, -targetSpeed.R);

}

// Need two controllers. One to use rho to set commandSum and one to use phi to set commandDifference
float controlRho(float current, float desired, const float KP, const float KI, const float KD) {
	float P = 0, D = 0, output = 0;
	// Calculate error
	error = desired - current;

	// If the error is really small or really big, clear the accumulated I to prevent overshoot
	if (abs(error) <= 0.001 || abs(error) >= 5) I_rho = 0;

	// Give I some help if the error changes sign
	if (error < 0 && I_rho > 0) I_rho = 0;
	if (error > 0 && I_rho < 0) I_rho = 0;

	// Calculate P component
	P = KP * error;
	// Calculate I component
	I_rho += KI * float(CONTROL_SAMPLE_RATE) * error;

	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorRho) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorRho = error;
		D *= KD;
	} else D = 0;
	// Calculate total controller output
	output = P + I_rho + D;
	// Make sure the output is within [-MAX_SPEED, MAX_SPEED]
	if (output > MAX_SPEED) output = MAX_SPEED;
	if (output < -MAX_SPEED) output = -MAX_SPEED;

	// Return the updated motorSum
	return output;
}

float controlPhi(float current, float desired, const float KP, const float KI, const float KD) {

	float P = 0, D = 0, output = 0;

	// Calculate error
	error = desired - current;
	// Calculate P component
	P = KP * error;
	// If the error is really small or really big, clear the accumulated I to prevent overshoot
	if (abs(error) <= 0.001) I_phi = 0;

	// Give I some help if the error changes sign
	if (error < 0 && I_phi > 0) I_phi = 0;
	if (error > 0 && I_phi < 0) I_phi = 0;

	// Calculate I component
	I_phi += KI * float(CONTROL_SAMPLE_RATE) * error;

	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorPhi) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorPhi = error;
		D *= KD;
	} else D = 0;

	// Calculate total controller output
	output = P + I_phi + D;

	// Make sure the output is within [-MAX_SPEED, MAX_SPEED]
	if (output > MAX_SPEED) output = MAX_SPEED;
	if (output < -MAX_SPEED) output = -MAX_SPEED;

	// Return the updated motorDif
	return output;
}

void computeSpeeds(float dif, float sum) {
	Pair<float> target = {0, 0};        //!< Motor outputs

	target.R = (sum - dif) / float(2.0);
	target.L = (sum + dif) / float(2.0);

	if (target.L < 0) target.L += 2.487932;
	if (target.L > 0) target.L -= 2.487932;

	// Make sure the new speeds are within [-MAX_SPEED, MAX_SPEED]
	if (target.R > MAX_SPEED) target.R = MAX_SPEED;
	if (target.L > MAX_SPEED) target.L = MAX_SPEED;
	if (target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if (target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	// Update the global targetSpeed variable
	// Average Difference (L-R): 2.487932
	targetSpeed = {int(target.L), int(target.R)};
}