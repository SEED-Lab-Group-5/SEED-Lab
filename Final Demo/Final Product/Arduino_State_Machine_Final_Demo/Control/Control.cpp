//
// Created by Alex Curtis on 4/14/22.
//

#include "Control.h"
#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#define ENCODER_OPTIMIZE_INTERRUPTS

#define ENC_R_WHITE 2                                   //!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5                                  //!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3                                   //!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6                                  //!< Left motor encoder output A (yellow wire)


const Pair<float> MIN_SPEED = {81.879180,80.635212};    //!< Minimum scaled PWM //TODO implement this
// Pairs
Pair<int> targetSpeed;              //!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400
Pair<long> counts, pastCounts;                  //!< Left and right encoder readings (counts)

Encoder EncR(ENC_R_WHITE, ENC_R_YELLOW); //!< Right motor encoder
Encoder EncL(ENC_L_WHITE, ENC_L_YELLOW); //!< Left motor encoder
DualMC33926MotorShield motors;           //!< Motor 2 is the right wheel

Control::Control(){
	// Initialize the motor object
	motors.init();
}

bool Control::drive(float targetPhi, float targetRho) {
	// Instead of running forever or waiting for the error to hit zero and cutting it off,
	// I'll use millis() and a local variable to keep track of how long the motors are off
	// That way it wont go forever or get cut off too fast
	
	// Setup once
	startControl();

	// Update encoder counts
	counts = {EncL.read(), -EncR.read()};
	// Find current robot positions
	phi = (RADIUS*RAD_CONVERSION*float(counts.L - counts.R))/BASE;
	rho = RADIUS*RAD_CONVERSION*float(counts.L + counts.R)*float(0.5); // Circumference = 2*PI*r
	// Update motorDif and motorSum with control() every CONTROL_SAMPLE_RATE ms
	if (millis()-startTime >= currentTime + CONTROL_SAMPLE_RATE) {
		// Adjust elapsed time
		currentTime += CONTROL_SAMPLE_RATE;
		// Calculate ∆Va
		motorDif = controlPhi(phi,targetPhi*float(PI)/float(180));
		// only start moving forward when done turning
		if(abs(motorDif) < 20) {
			if(firstRho) { // to mitigate the initial encoder readings from turning
				rhoOffset = RADIUS*RAD_CONVERSION*float(counts.L + counts.R)*float(0.5);
				firstRho = false;
			}
			// Calculate Va
			motorSum = controlRho(rho-rhoOffset,targetRho);
		}
	}
	// Determine target motor speeds based on motorDif and motorSum using setMotorValues()
	setMotors(motorDif,motorSum);
	// Set the motors to the new speeds
	motors.setSpeeds(targetSpeed.L, -targetSpeed.R);

	
	if(isDone()) {
		stopControl(); // Clean up locals
		return true;
	}

	return false;
}


void Control::startControl() {
	// Guard clause
	if(driveStarted) return;
	else driveStarted = true;

	// Reset flags
	firstRho = true;

	// Reset driving variables
	rhoOffset = 0;
	motorDif = 0;
	motorSum = 0;
	error = 0, pastErrorRho = 0, pastErrorPhi = 0;
	I_rho = 0, I_phi = 0;

	// Save current time
	currentTime = millis();
	startTime = millis();



	// Set left and right motor speeds to 0
	motors.setSpeeds(0, 0);

}

/**
 * Reset controller when finished
 */
void Control::stopControl() {

	// Reset encoders
	EncR.readAndReset();
	EncL.readAndReset();

	// Set left and right motor speeds to 0
	motors.setSpeeds(0, 0);

} // End


/**
 * Read current encoder counts and calculate phi and rho
 */
void Control::getPositions() {
	// Update encoder counts
	counts = {EncL.read(), -EncR.read()};
	// Find current robot positions
	phi = (RADIUS * RAD_CONVERSION * float(counts.L - counts.R)) / BASE;
	rho = RADIUS * RAD_CONVERSION * float(counts.L + counts.R) * float(0.5);


}

float Control::controlRho(float current, float desired) {

	float P = 0, D = 0, output = 0;
	// Calculate error
	error = desired - current;
	// Calculate P component
	P = KP_RHO * error;
	// Calculate I component
	I_rho += KI_RHO * float(CONTROL_SAMPLE_RATE / 1000.0) * error;
	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorRho) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorRho = error;
		D *= KD_RHO;
	} else D = 0;
	// Calculate controller output
	output = P + I_rho + D;
	// Make sure the output is within [-400, 400]
	if(output > 400) output = 400;
	if(output < -400) output = -400;

	// Make sure the output is large enough for the motors to turn
	if(error > 0.5 && output < 80) output = 80;
	if(error < -0.5 && output > -80) output = -80;

	// Print current values for testing
//	Serial.print("\trho: "); Serial.print(current,5);
//	Serial.print("\ttargetRho: "); Serial.print(desired);
//	Serial.print("\terror: "); Serial.print(error,5);
//	Serial.print("\tP: "); Serial.print(P);
//	Serial.print("\tI: "); Serial.print(I_phi);
//	Serial.print("\tD: "); Serial.print(D);
//	Serial.print("\tSum: "); Serial.println(output);


	return output;
}

float Control::controlPhi(float current, float desired) {

	float P = 0, D = 0, output = 0;
	// Calculate error
	error = desired - current;
	// Calculate P component
	P = KP_PHI * error;
	// Calculate I component
	I_phi += KI_PHI * float(CONTROL_SAMPLE_RATE / 1000.0) * error;
	// Calculate D component
	if (currentTime > 0) {
		D = (error - pastErrorPhi) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorPhi = error;
		D *= KD_PHI;
	} else D = 0;
	// Calculate controller output
	output = P + I_phi + D;
	// Make sure the output is within [0, 400]
	if(output > 400) output = 400;
	if(output < -400) output = -400;

	if(error > 0.1 && output < 80) output = 80;
	if(error < -0.1 && output > -80) output = -80;

	// Print current values for testing
	//Serial.print("phi: "); Serial.print(current,5);
	//Serial.print("\ttargetPhi: "); Serial.print(desired);
	//Serial.print("\terror: "); Serial.print(error,5);
	//Serial.print("\tP: "); Serial.print(P);
	//Serial.print("\tI: "); Serial.print(I_phi);
	//Serial.print("\tD: "); Serial.print(D);
	//Serial.print("\tnewDif: "); Serial.println(output);

	return output;
}

void Control::setMotors(float diff, float sum) const {
	Pair<float> target = {0,0}; 		//!< Motor PWM outputs
	// sum: A value between -400 and 400 describing the voltage sum applied to left and right motors
	// (Voltage of left motor + voltage of right motor = sum. sum = -400 = full reverse, sum = 400 = full forward)
	// diff: A decimal value between 0 and 400 describing the proportional difference in voltages applied to left nd right motors (controls rotation, higher value =

	target.R = (sum - diff) / float(2.0);
	target.L = (sum + diff) / float(2.0);

	// Make sure the speeds are within [-400, 400]
	if(target.R > MAX_SPEED) target.R = MAX_SPEED;
	if(target.L > MAX_SPEED) target.L = MAX_SPEED;
	if(target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if(target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	// Update the targetSpeed variable
	targetSpeed = {int(target.L),int(target.R)};
}

bool Control::isDone() {
	bool done = false;
	// If the errors are low enough for long enough, return true

	// if current position is the same as the previous position

	// Every MIN_SETTLING_TIME milliseconds
	if (millis() - startTime >= lastTime + MIN_SETTLING_TIME) {

		lastTime += MIN_SETTLING_TIME;

		// If the encoders haven't changed
		if(counts.L == pastCounts.L && counts.R == pastCounts.R) {
			// The robot is done moving, or isn't moving fast enough for us to wait.
			done = true;
		}

		// Save position
		pastCounts = counts;
	}

	return done;
}

