//////////////////////////////////////////////////////////////////////// NAME:
// CLASS:	 EENG-350
// GROUP:    5
// TITLE:    Demo 2
// FUNCTION: Write what your code does here
// HARDWARE: Any hardware connections you must make to your device
// SOFTWARE: Any software that must be installed to the device
// EXECUTE:  Execution instructions for your program
// RESOURCE: Link to any resource you used
// PURPOSE:  What the resource was used for
// RESOURCE: Link to any resource you used
// PURPOSE:  What the resource was used for
//////////////////////////////////////////////////////////////////////



#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include <Wire.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#define SLAVE_ADDRESS 0x04

// Communication with PI
// TODO change this to work with David's new structure
void receiveData(int byteCount);
bool angleRead;
int desiredAngleCoeff = 2;              // Angle to move the motor to (Value between 0-3)
const unsigned long PI_SAMPLE_TIME = 5; // Number of milliseconds between data receive from Pi
unsigned long prevSampleTime = 0;       // Time the last data receive occurred

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



// Constants
const float CPR = 3200;                            			//!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
const float RADIUS = 2.9375;                            	//!< Measured radius of wheels in inches
const float BASE = 13.8;                                	//!< Distance between center of wheels in inches
const float RAD_CONVERSION = float(2.0 * PI) / CPR;         //!< Scalar to convert counts to radians
const long CONTROL_SAMPLE_RATE = 5;                     //!< Controller sample rate in ms
const int MAX_SPEED = 400;                            		//!< Maximum scaled PWM (max motor speed = 400)
const int MIN_SPEED = 84;                                	//!< Minimum scaled PWM
#define ENC_R_WHITE 2                                    	//!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5                                		//!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3                                    	//!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6                                    	//!< Left motor encoder output A (yellow wire)

// Controller globals
float rho = 0, targetRho = 0;        //!< current and target distances in inches
float phi = 0, targetPhi = 0;        //!< current and target angles in radians
float rhoOffset = 0;            	//!< Contains initial forward counts after rotating
float motorDif, motorSum;        	//!< Parameters for speed control. motorDif [-400,400] and motorSum [-400, 400]
float errorPhi, pastErrorRho = 0, pastErrorPhi = 0;        	//!< Variables used in calculating control output
float I_rho = 0, I_phi = 0;                                	//!< Integral controller accumulations
unsigned long currentTime = 0, startTime = 0;           	//!< For creating a discrete time controller

// Flags
bool tapeFound = false;         	//!<flag to know whether to search for the tape or not
bool firstRho = true;           	//!< Flag for accurately determining forward counts after rotating
bool rotating = true;            	//!< Flag indicating the robot is currently turning
bool encReset = false;

// Pairs
Pair<int> targetSpeed;            	//!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400
Pair<long> counts;                	//!< Left and right encoder readings (counts)

// Functions
void initialize();
void runState();
void getPositions();
bool initialCameraRead();
bool drive(float angle, float forward);
Pair<float> computeControllers();
void scanForTape();
float controlRho(float current, float desired, float KP, float KI, float KD);
float controlPhi(float current, float desired, float KP, float KI, float KD);

void encoderReset();

// Objects
Encoder EncR(ENC_R_WHITE, ENC_R_YELLOW); //!< Right motor encoder
Encoder EncL(ENC_L_WHITE, ENC_L_YELLOW); //!< Left motor encoder
DualMC33926MotorShield motors;    					//!< Motor 2 is the right wheel


void setMotors(float dif, float sum);

/**
 * Does initial setup
 */
void initialize() {
	// Begin serial communication
	Serial.begin(9600); // TODO remove when done testing

	// Initialize i2c as slave
	Wire.begin(SLAVE_ADDRESS);

	// Define callbacks for i2c communication
	Wire.onReceive(receiveData);

	// Get initial values of currentTime and startTime
	currentTime = millis();
	startTime = millis();

	// Initialize the motor object
	motors.init();

	// Set left and right motor speeds to 0
	motors.setSpeeds(0, 0);
}



// TODO remove loop
void setup() {
	initialize();
}

void loop() {

	scanForTape();
	runState();
}

void runState() {

	getPositions();

	Pair<float> controlOutput = {0,0};

	controlOutput = computeControllers();

	motorDif = controlOutput.L;
	motorSum = controlOutput.R;

	// Determine Va,L and Va,R based on motorDif and motorSum
	setMotors(motorDif, motorSum);
}

/**
 * Reset encoder counts
 */
void encoderReset() {
    EncR.write(0);
    EncL.write(0);
}

/**
 * Read current encoder counts and calculate phi and rho
 */
void getPositions() {
	// Update encoder counts
	counts = {EncL.read(), -EncR.read()};
	// Find current robot positions
	phi = (RADIUS * RAD_CONVERSION * float(counts.L - counts.R)) / BASE;
	rho = RADIUS * RAD_CONVERSION * float(counts.L + counts.R) * float(0.5);
}

/**
 * Check if the camera can see tape. If not, scanForTape()
 */
bool initialCameraRead() {
    // TODO read from camera whether tape is seen or not
    int newData; int oldData;

    while((Serial.available() == 0) && initialCameraRead()){
        newData = Serial.read();
    }

    return false;
}

/**
 * Rotate the robot until the start of the tape is found
 */
void scanForTape() {
	byte increment = 1;
	initialCameraRead();


	targetSpeed = {100, 100};

	while (!tapeFound && phi < (2 * PI)) {

		while (phi < ((increment * 30)*(PI/180)) ) {
            getPositions();
			motors.setSpeeds(targetSpeed.L, targetSpeed.R);
		}
		initialCameraRead();
		increment++;
	}
}

bool drive(float angle, float forward){
    targetPhi = angle;
    targetRho = forward;
    if(encReset){ ////TODO must be set to true in FSM before calling drive function
        encoderReset();
    }
    encReset = false;

    computeControllers();
    if (errorPhi < 1){
        return false;
    }
}

Pair<float> computeControllers() {
	// Controller Parameters
	const float KP_RHO = 41.507628, KI_RHO = 2, KD_RHO = 0.000000;    	//!< Rho controller constants
	const float KP_PHI = 260.542014, KI_PHI = 5, KD_PHI = 0.000000;    	//!< Phi controller constants

	if (millis() - startTime >= currentTime + CONTROL_SAMPLE_RATE) {

		// Determine next time to update motorDif and motorSum
		currentTime += CONTROL_SAMPLE_RATE;

		// Calculate ∆Va (motorDif)
		motorDif = controlPhi(phi, targetPhi * float(PI) / float(180), KP_PHI, KI_PHI, KD_PHI);
		rotating = abs(motorDif) >= 20;
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

	// If the robot is turning, stop moving forward
	if (rotating) motorSum = 0;
	else motorDif = 0;

	return {motorDif,motorSum};
}


float controlRho(float current, float desired, const float KP, const float KI, const float KD) {
	// TODO revert changes that made movement choppy. The camera should help correct the robot
	float P = 0, D = 0, output = 0;
	// Calculate error
	errorPhi = desired - current;
	// If the error is really small or really big, clear the accumulated I to prevent overshoot
	if (abs(errorPhi) <= 0.001 || abs(errorPhi) >= 5) I_rho = 0;

	// Give I some help if the error changes sign
	if (errorPhi < 0 && I_rho > 0) I_rho = 0;
	if (errorPhi > 0 && I_rho < 0) I_rho = 0;

	// Calculate P component
	P = KP * errorPhi;
	// Calculate I component
	I_rho += KI * float(CONTROL_SAMPLE_RATE) * errorPhi;

	// Calculate D component
	if (currentTime > 0) {
		D = (errorPhi - pastErrorRho) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorRho = errorPhi;
		D *= KD;
	} else D = 0;

	// Calculate total controller output
	output = P + I_rho + D;

	// Make sure the output is within [-MAX_SPEED, MAX_SPEED]
	if (output > MAX_SPEED) output = MAX_SPEED;
	if (output < -MAX_SPEED) output = -MAX_SPEED;

	return output;
}

float controlPhi(float current, float desired, const float KP, const float KI, const float KD) {
	float P = 0, D = 0, output = 0;
	// Calculate error
	errorPhi = desired - current;
	// Calculate P component
	P = KP * errorPhi;
	// If the error is really small or really big, clear the accumulated I to prevent overshoot
	if (abs(errorPhi) <= 0.001) I_phi = 0;

	// Give I some help if the error changes sign
	if (errorPhi < 0 && I_phi > 0) I_phi = 0;
	if (errorPhi > 0 && I_phi < 0) I_phi = 0;

	// Calculate I component
	I_phi += KI * float(CONTROL_SAMPLE_RATE) * errorPhi;

	// Calculate D component
	if (currentTime > 0) {
		D = (errorPhi - pastErrorPhi) / float(CONTROL_SAMPLE_RATE / 1000.0);
		pastErrorPhi = errorPhi;
		D *= KD;
	} else D = 0;

	// Calculate total controller output
	output = P + I_phi + D;

	// Make sure the output is within [-MAX_SPEED, MAX_SPEED]
	if (output > MAX_SPEED) output = MAX_SPEED;
	if (output < -MAX_SPEED) output = -MAX_SPEED;

	return output;
}

/**
 * setMotors() Determines Va,L and Va,R based on dif and sum
 * @param dif ∆Va = the target difference between Va,L and Va,R
 * @param sum Va = the target sum of Va,L and Va,R
 */
void setMotors(float dif, float sum) {
	Pair<float> target = {0, 0};        //!< Motor outputs

	target.R = (sum - dif) / float(2.0);
	target.L = (sum + dif) / float(2.0);

	// TODO verify this works as intended. if not, remove it.
	// Average Difference (L-R): 2.487932
	if (target.L < 0) target.L += 2.487932;
	if (target.L > 0) target.L -= 2.487932;

	// Make sure the new speeds are within [-MAX_SPEED, MAX_SPEED]
	if (target.R > MAX_SPEED) target.R = MAX_SPEED;
	if (target.L > MAX_SPEED) target.L = MAX_SPEED;
	if (target.R < -MAX_SPEED) target.R = -MAX_SPEED;
	if (target.L < -MAX_SPEED) target.L = -MAX_SPEED;

	// Update the global targetSpeed variable
	// TODO can we remove this?
	targetSpeed = {int(target.L), int(target.R)};

	// Set the left and right motors to the new speeds
	motors.setSpeeds(targetSpeed.L, -targetSpeed.R);
}

/**
 * Callback for received data (From mini project)
 * @param byteCount
 */
void receiveData(int byteCount) {

	// While there are still bytes to read, read them and store the most recent to number
	while (Wire.available()) {
		int nextAngleCoef = Wire.read();
		unsigned long currSampleTime = millis();
		// If time since previous sample exceeds SAMPLE_TIME receive a new input
		if (millis() - prevSampleTime > PI_SAMPLE_TIME) {
			// TODO set global targets to new target
			angleRead = true;
			desiredAngleCoeff = nextAngleCoef;

			prevSampleTime = currSampleTime;
		}
	}
}