//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:
// CLASS:	 EENG-350
// GROUP:    5
// TITLE:    Steering Control - Step Response Experiment
//
// FUNCTION:
// SOFTWARE:
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
#include <math.h>

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
const int CPR = 3200;	// Total encoder counts per revolution (CPR) of motor shaft: 3200counts/rotation
// Castor wheel is the back of the robot
#define ENC_R_WHITE 2 			// Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5  		// Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3			// Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6			// Left motor encoder output A (yellow wire)

// From mini project, pin1 on the Encoder object needs to connect to the white wire on the motor encoder
// for CCW to be positive (facing the wheel)

Encoder motorEncR(ENC_R_WHITE, ENC_R_YELLOW); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
Encoder motorEncL(ENC_L_WHITE, ENC_L_YELLOW);
Pair<long> newPosition;     	// Current position reading (counts)


// Parameters
DualMC33926MotorShield motors;                  // Motor 2 is the right wheel

const float WHEEL_RADIUS = 2.9375;              //Radius of wheel in inches
const float WHEELBASE = 13.25;                  //Wheelbase measurement in inches
const long SAMPLE_RATE = 5;     	            // Sample rate in ms
const int MAX_SPEED = 400;   	                // Desired speed. If using DualMC33926MotorShield.h: max speed = 400
const int MIN_SPEED = -400;                     // Desired speed. If using DualMC33926MotorShield.h: min speed = -400
bool motorsSet = false;

unsigned long now = 0;    			                // Elapsed time in ms
unsigned long start = 0;  			                // Experiment start time


/*

Pair<float> oldAngPos, newAngPos, angVel;
volatile float oldAngPos.R = 0, angPos.L = 0;     		    // position relative to initial position (radians)
volatile float newAngPos.R = 0, newAngPos.L = 0;            // current position relative to initial position (radians)
volatile float angVel.R = 0, angVel.L = 0;     		        // Current angular velocity (rad/s)
*/



// Steering
float motorDif, motorSum;                       // Parameters for steering
Pair<int> targetSpeed;


// Given the sum and difference [0,1], set the speed command of each motor
void setMotorValues(float commandDifference, float commandSum);


/*
bool commandReceived = false; 	                // flag for new serial input
bool run1, run2;				                // flags to indicate which experiment to run
String InputString = ""; 		                // a string to hold incoming data
*/

float currentPhi = 0;
float oldPhi = 0;
volatile float distanceLeft = 0;
volatile float distanceRight = 0;
int nLeft = 0;
int nRight = 0;
long currentEncR = 1;
long currentEncL = 1;
unsigned long oldTime=0;
float rho_dot = 0;
float phi_dot = 0;

///I couldnt get the pair to work (it was red)
float oldAngPosR=0, newAngPosR=0, angVelR=0;
float oldAngPosL=0, newAngPosL=0, angVelL=0;

////Trevors attempt at a PD implementation
float angleNewError = 0;
float angleOldError = 0;
float desiredAngle = 0;
float phi_Kd = 30;      //starting untuned at 30
float phi_Kp = 50;      //starting untuned at 50
float phi_PD = 0;
float phi_D = 0;
int motorSpeed;
int phiMotorChange = 0;
///control variable = speed passed to motor control function??

///Variables for PD for distance forward
float desiredForwardMotion = 0;            //units should be in inches
float currentForwardMotion = 0;
float rhoNewError = 0;
float rhoOldError = 0;
float rhoAngle = 0;
float rho_Kd = 30;      //starting untuned at 30
float rho_Kp = 50;      //starting untuned at 50
float rho_PD = 0;
float rho_D = 0;
int rhoMotorChange = 0;


//void readEnc();


void setup() {

    Serial.begin(115200);
//    InputString.reserve(200);      // reserve 200 bytes for the inputString
    motors.init();                       // Initialize motor
    motors.setSpeeds(0, 0);         // Set motor A and B speeds to 0
    Serial.println("Ready!");           // Tell Matlab that Arduino is ready
}

void loop() {
    /*// Change behavior based on serial input
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
    }*/

    currentEncR = motorEncR.read();
    currentEncL = motorEncL.read();
    if (abs(currentEncR) > CPR) {
        nRight = 1 + int(floorf(abs(currentEncR) / (CPR)));
    } else if (abs(currentEncL) > CPR) {
        nLeft = 1 + int(floorf(abs(currentEncL) / (CPR)));
    }

    newAngPosR = -(2 * PI * (currentEncR) / CPR);                        ////Inversion occurs Here with negative sign
    newAngPosL = (2 * PI * (currentEncL) / CPR);

    angVelR = 1000 * ((newAngPosR - oldAngPosR) / ((millis() - oldTime)));
    angVelL = 1000 * ((newAngPosL - oldAngPosL) / ((millis() - oldTime)));

    rho_dot = (WHEEL_RADIUS * (angVelL + angVelR)) / 2;
    phi_dot = (WHEEL_RADIUS * (angVelL - angVelR)) / WHEELBASE;

    distanceRight = nRight * 2 * PI * pow(WHEEL_RADIUS, 2);
    distanceLeft = nLeft * 2 * PI * pow(WHEEL_RADIUS, 2);


    //keep track of current angle
    currentPhi = oldPhi + ((WHEEL_RADIUS / WHEELBASE) * (distanceLeft - distanceRight));    //eq from Assignment 2: Localization
    oldPhi = currentPhi;

    //TODO have turn and distance control variables add or subtract to the motorSpeed

    ////Trevors attempt at a PD implementation - this method may only be good for Demo1

    while (desiredAngle != currentPhi) {
        if (desiredAngle > currentPhi) {

            angleNewError = desiredAngle - currentPhi;
            phi_D = angleNewError - angleOldError;
            phi_PD = (phi_Kp * angleNewError) + (phi_Kd * phi_D);


            phiMotorChange = int(400 * phi_PD);
            if (phiMotorChange > 400) {
                phiMotorChange = 400;
            } else if (phiMotorChange < -400) {
                phiMotorChange = -400;
            }
            motors.setM1Speed(motorSpeed + phiMotorChange);
            motors.setM2Speed(motorSpeed - phiMotorChange);
            angleOldError = angleNewError;
        }
        if (desiredAngle < currentPhi) {

            angleNewError = desiredAngle - currentPhi;
            phi_D = angleNewError - angleOldError;
            phi_PD = (phi_Kp * angleNewError) + (phi_Kd * phi_D);


            phiMotorChange = int(400 * phi_PD);
            if (phiMotorChange > 400) {
                phiMotorChange = 400;
            } else if (phiMotorChange < -400) {
                phiMotorChange = -400;
            }
            motors.setM1Speed(motorSpeed - phiMotorChange);
            motors.setM2Speed(motorSpeed + phiMotorChange);
            angleOldError = angleNewError;
        }

        ///ends here
        oldAngPosR = newAngPosR;
        oldAngPosL = newAngPosL;
        oldTime = millis();
    }


    ////controller for movement forward - this method may only be good for Demo1
    while(currentForwardMotion != desiredForwardMotion){
        rhoNewError = desiredForwardMotion - currentForwardMotion;
        rho_D = rhoNewError - rhoOldError;
        rho_PD = (rho_Kp * rhoNewError) + (rho_Kd * rho_D);


        rhoMotorChange = int(400 * phi_PD);
        if (rhoMotorChange > 400) {
            rhoMotorChange = 400;
        } else if (rhoMotorChange < -400) {
            rhoMotorChange = -400;
        }
        motors.setM1Speed(motorSpeed + rhoMotorChange);
        motors.setM2Speed(motorSpeed + rhoMotorChange);
        angleOldError = angleNewError;
    }
}

////Attempting to make encoder readings a simple function call
/*int readEnc(){
    currentEncR = motorEncR.read();
    currentEncL = motorEncR.read();
    if(abs(currentEncR) > 3200){
        nRight = 1+int(floorf(abs(currentEncR)/(3200)));
    }else if(abs(currentEncL) > 3200){
        nLeft = 1+int(floorf(abs(currentEncL)/(3200)));
    }
}*/

/*void setMotorValues(float commandDifference, float commandSum) {
    Pair<float> target = {0,0};

    // TODO do the math on the handout to get Va,1 and Va,2 from the sum and difference
    // commandSum: A decimal value between -1 and 1 describing the proportional voltage sum applied to left and right motors
    // (Voltage of left motor + voltage of right motor = commandSum. sum = -1 = full reverse, sum = 1 = full forward)
    // commandDifference: A decimal value between -1 and 1 describing the proportional difference in voltages applied to left nd right motors (controls rotation, higher value =

    target.R = (-commandSum+commandDifference)/float(2.0);      //Added negative to give negative speed and allow forward motion.
    target.L = (commandSum-commandDifference)/float(2.0);

    // Scale the ratios to map a max of 1 to a max of 400
    target = target * float(400);

    // Set the global targetSpeed variable with the
    targetSpeed = {int(target.L),int(target.R)};
    //targetSpeed = {targetL,targetR};
}*/
/*

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
}*/
