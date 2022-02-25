#include "DualMC33926MotorShield.h"

#define ENCODER_OPTIMIZE_INTERRUPTS         // Highly Optimized interrupt in background
#define SLAVE_ADDRESS 0x04

#include "Encoder.h"
#include "Arduino.h"
#include "math.h"
#include <Wire.h>

//DualMC33926MotorShield md;
/*DualMC33926MotorShield::DualMC33926MotorShield()
{
  //Pin map
  _nD2 = 4;
  _M1DIR = 7;
  _M1PWM = 9;
  _nSF = 12;
  _M1FB = A0;
}*/

const float KP = 1.2529, KI = 0.0903, KD = 0;
unsigned long Ts = 0, Tc = 0;
float error, pastError = 0;

void receiveData(int byteCount);

int control(float current, float desired);

#define ENC_PIN_A 3       // Encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define ENC_PIN_B 2       // Encoder output B (white wire)
Encoder motorEnc(ENC_PIN_A, ENC_PIN_B); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
DualMC33926MotorShield motor;

long initialPosition = 0;          // initial position of motor (counts)
volatile long position = 0;        // Previous position reading (counts)
volatile long newPosition = 0;     // Current position reading (counts)
unsigned long now = 0;    // Elapsed time in ms
unsigned long start = 0;  // Experiment start time
volatile float currentAngle = 0;     // position relative to initial position (radians)
volatile float angVelocity = 0;     // Current angular velocity (rad/s)

const float CPR = 50.0 * 64.0;           // Total encoder counts per revolution (CPR) of motor shaft
const float RAD_PER_COUNT = (2 * PI) / 3200;    // 0.0019634954 rad/count

int desiredAngleCoeff = 0;     // Angle to move the motor to (Value between 0-3 to be multiplied by PI/2 to find angle to move motor to)    ///// May need to set this to 0 for final implementation, not sure yet /////
float desiredAngle = 0;
bool angleRead;

const float FLOAT_DELTA = 0.01; // How close a float must be to a desired value to be considered equal (actualValue +- FLOAT_DELTA = desiredValue)
const int BRAKE_TIME = 50;      // Time in ms to give motor to stop after being told to do so       ////// May not need this //////

const unsigned long SAMPLE_TIME = 5;  // Number of milliseconds between data receive from Pi
unsigned long prevSampleTime = 0;        // Time the last data receive occured

const long SAMPLE_RATE = 5;     // Sample rate in ms
int targetSpeed = 0;

void setup() {
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  // Begin serial communication
  Serial.begin(9600);

  // Get initial position
  initialPosition = motorEnc.read();

  motor.init();                       // Initialize motor
  motor.setM1Speed(0);
}

void loop() {

  // Read encoders and update current time
  newPosition = motorEnc.read();
  Tc = millis();
  while (newPosition >= 3200) newPosition -= 3200; // MODIFIED SO 3200 COUNTS CHANGES TO 0!!!!!!

  // Find current angular motor position relative to initial position
  currentAngle = float(newPosition - initialPosition) * float((2.0 * PI) / float(CPR));

  // Calculate desired angular position
  desiredAngle = desiredAngleCoeff * float(PI / 2);

  // Find current angular velocity in rad/s: (x2 - x1) / ∆t
  //angVelocity = ((float((newPosition - initialPosition) - (position - initialPosition)) * float((2.0 * PI) / CPR)) *float(1000)) / float(Ts);

  // use control() to determine target speed and direction
  targetSpeed = control(newPosition, desiredAngleCoeff*800);

  // Set the motor to the target speed (also accounts for direction)
  motor.setM1Speed(targetSpeed);

  // Print elapsed time, target speed, angular velocity
  //Serial.print(millis() - start); // Prints elapsed time in ms
  //Serial.print("\t");
  //Serial.print((targetSpeed) * 51 / 80); // Same conversion used in the motor library for analogWrite()

}

// Current and desired are angles in radians
int control(float current, float desired) {
  float I = 0, D = 0;

  double newTargetSpeed, maxSpeed = 400;   // 255 or 400 if using the motorShield header

  error = current - desired;      // Update error

  if (Ts > 0) {
    // Calculate D
    D = (error - pastError) / float(Ts);
    pastError = error;
  } else {
    D = 0;
  }

  // Calculate I
  I += float(Ts) * error;
  // If the error is approximately 0, set I=0
  //if (abs(error) < 0.001) error = 0;

  // Calculate controller output
  newTargetSpeed = KP * error + KI * I + KD * D;

  // If calculated controller output is higher than the maximum output,
  // set output to the maximum output with matching sign
  if (abs(newTargetSpeed) > maxSpeed) {
    if (newTargetSpeed < 0) {
      newTargetSpeed = -1 * maxSpeed;
    } else {
      newTargetSpeed = maxSpeed;
    }
  }

  Ts = millis() - Tc; // Determine sample time Ts
  Tc = millis();      // Update current time? //TODO is this redundant if it gets set in loop()?

  //TODO scale output to [-255, 255]
  Serial.print("\nCurrentAngle: ");
  Serial.print(current);
  Serial.print("\tDesiredAngle: ");
  Serial.print(desired);
  Serial.println("\terror: ");
  Serial.println(error);
  Serial.print("\tcontroller: ");
  Serial.println(newTargetSpeed);
  Serial.print("\tcontroler(int): ");
  Serial.println(int(newTargetSpeed));

  newTargetSpeed = (newTargetSpeed)*50;

  return int(newTargetSpeed);
}

// callback for received data
void receiveData(int byteCount) {
  // While there are still bytes to read, read them and store the most recent to number
  while (Wire.available()) {
    int nextAngleCoef = Wire.read();
    unsigned long currSampleTime = millis();
    // If time since previous sample exceeds SAMPLE_TIME receive a new input
    if (millis() - prevSampleTime > SAMPLE_TIME) {
      desiredAngleCoeff = nextAngleCoef;
      angleRead = true;
      prevSampleTime = currSampleTime;
      Serial.print("Desired Angle: ");
      Serial.println(desiredAngleCoeff);
    }
  }
}
