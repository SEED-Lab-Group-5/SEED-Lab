//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     Alex Curtis (Primary Author)
// CLASS:	 EENG-350
// GROUP:    5
// TITLE:    Motor Control Mini-Project
//
// FUNCTION: This program is our final implementation of our controller.
//
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


#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#define SLAVE_ADDRESS 0x04

// ***** Control
const float KP = 88.1417, KI = 4.8396, KD = 0;          // Calculated in Matlab (ArduinoFindTf.mlx)
int control(float current, float desired);              // Returns new motor speed based on desired and current position
float P = 0, I = 0, D = 0, error, pastError = 0;        // Variables used in calculating control output
unsigned long currentTime = 0, startTime = 0;           // For creating a discrete time controller
const long CONTROL_SAMPLE_RATE = 5;                     // Controller sample rate in ms
int targetSpeed = 0;                                    // Controller output (int [-400, 400])
volatile float currentAngle = 0, desiredAngle = 0;      // position relative to initial position (radians)

// ***** Localization
#define ENC_PIN_A 3                     // Encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define ENC_PIN_B 2                     // Encoder output B (white wire)
Encoder motorEnc(ENC_PIN_A, ENC_PIN_B); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
const float CPR = 50.0 * 64.0;          // Total encoder counts per revolution (CPR) of motor shaft
long initialPosition = 0;               // Initial position of motor (counts)
volatile long position = 0;             // Previous position reading (counts)
volatile long newPosition = 0;          // Current position reading (counts)

// ***** Motor Control
DualMC33926MotorShield motor;           // Motor object

// ***** Communication with PI
void receiveData(int byteCount);
bool angleRead;
int desiredAngleCoeff = 2;              // Angle to move the motor to (Value between 0-3)
const unsigned long PI_SAMPLE_TIME = 5; // Number of milliseconds between data receive from Pi
unsigned long prevSampleTime = 0;       // Time the last data receive occurred

void setup() {

	// Initialize i2c as slave
	Wire.begin(SLAVE_ADDRESS);

	// Define callbacks for i2c communication
	Wire.onReceive(receiveData);

	// Begin serial communication
	Serial.begin(9600);

	// Get initial position and start time
	initialPosition = motorEnc.read();
	currentTime = millis();
	startTime = millis();

	// Initialize the motor
	motor.init();
	motor.setM1Speed(0);
}

void loop() {
	// Read encoders and update current time
	newPosition = motorEnc.read();

	// Find current angular motor position relative to initial position
	currentAngle = float(newPosition - initialPosition) * float((2.0 * PI) / float(CPR)); // Position*2PI/3200

	// Calculate desired angular position
	desiredAngle = desiredAngleCoeff * float(PI / 2);

	// use control() to determine target speed and direction
	targetSpeed = control(currentAngle, desiredAngle);

	// Set the motor to the target speed (also accounts for direction)
	motor.setM1Speed(targetSpeed);
}

// Current and desired are angles in radians
int control(float current, float desired) {

	// KI and KP were found with 1 mapped to 400, so no additional scaling is needed.
	// NOTE: This took way more work to create than it should have. Not to mention how much wrong advice I was given.
	// Everyone I talked to had to completely modify their calculated values to get their controller to work,
	// or find new KP and KI values by hand. I'm stunned by how much of this I had to determine on my own.

	double newSpeed = 0, maxSpeed = 400, minSpeed = -400;

	if (millis()-startTime >= currentTime + CONTROL_SAMPLE_RATE) {

		// Adjust elapsed time
		currentTime += CONTROL_SAMPLE_RATE;

		// Calculate error
		error = desired - current;

		// Calculate P component
		P = KP * error;

		// Calculate I component
		I += KI * float(CONTROL_SAMPLE_RATE/1000.0) * error;

		// Calculate D component
		if (currentTime > 0) {
			D = (error - pastError) / float(CONTROL_SAMPLE_RATE/1000.0);
			pastError = error;
			D *= KD;
		} else D = 0;

		// Calculate total controller output
		newSpeed = P + I + D;

		// Make sure the output is within [minSpeed, maxSpeed]
		if(newSpeed > maxSpeed) newSpeed = maxSpeed;
		if(newSpeed < minSpeed) newSpeed = minSpeed;
	}

	// Print current values for testing
	Serial.print("\nCurrent: "); Serial.print(current);
	Serial.print("\tDesired: "); Serial.print(desired);
	Serial.print("\terror: "); Serial.print(error);
	Serial.print("\tP: "); Serial.print(P);
	Serial.print("\tI: "); Serial.print(I);
	Serial.print("\tD: "); Serial.print(D);
	Serial.print("\tnewSpeed: "); Serial.println(int(newSpeed));

	// Return controller output
	return int(newSpeed);
}

// Callback for received data
void receiveData(int byteCount) {

	// While there are still bytes to read, read them and store the most recent to number
	while (Wire.available()) {
		int nextAngleCoef = Wire.read();
		unsigned long currSampleTime = millis();
		// If time since previous sample exceeds SAMPLE_TIME receive a new input
		if (millis() - prevSampleTime > PI_SAMPLE_TIME) {
			angleRead = true;
			desiredAngleCoeff = nextAngleCoef;
			prevSampleTime = currSampleTime;
			Serial.print("Desired Angle: ");
			Serial.println(desiredAngleCoeff);
		}
	}
}