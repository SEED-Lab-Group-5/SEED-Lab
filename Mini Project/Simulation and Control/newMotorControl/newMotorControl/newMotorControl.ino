// SEED LAB
// Mini Project - Controller Implementation
// UPDATED!

#include "DualMC33926MotorShield.h"
#include "Encoder.h"
#include "Arduino.h"
#include "math.h"
#include <Wire.h>
#define ENCODER_OPTIMIZE_INTERRUPTS         // Highly Optimized interrupt in background
#define SLAVE_ADDRESS 0x04

/*DualMC33926MotorShield::DualMC33926MotorShield()
{
  //Pin map
  _nD2 = 4;
  _M1DIR = 7;
  _M1PWM = 9;
  _nSF = 12;
  _M1FB = A0;
}*/

int control(float current, float desired);
const float KP = 0.6137, KI = 0.0888, KD = 0;
float I = 0, D = 0, error, pastError = 0;
unsigned long Ts = 0, Tc = 0;
int targetSpeed = 0;

int desiredAngleCoeff = 0;            // Angle to move the motor to (Value between 0-3)
volatile float currentAngle = 0, desiredAngle = 0;    // position relative to initial position (radians)

#define ENC_PIN_A 3       // Encoder output A (yellow wire, must be connected to 2 or 3 for interrupts)
#define ENC_PIN_B 2       // Encoder output B (white wire)
Encoder motorEnc(ENC_PIN_A, ENC_PIN_B); // 1st pin needs to be capable of interrupts (UNO: pin 2 or 3)
const float CPR = 50.0 * 64.0;           // Total encoder counts per revolution (CPR) of motor shaft
const float RAD_PER_COUNT = (2 * PI) / 3200;    // 0.0019634954 rad/count

DualMC33926MotorShield motor;
long initialPosition = 0;            // initial position of motor (counts)
volatile long position = 0;            // Previous position reading (counts)
volatile long newPosition = 0;        // Current position reading (counts)
volatile float velocity = 0;        // Current angular velocity (rad/s)

const float FLOAT_DELTA = 0.01;    // How close a float must be to a desired value to be considered equal (actualValue +- FLOAT_DELTA = desiredValue)
void receiveData(int byteCount);

bool angleRead;
const unsigned long SAMPLE_TIME = 5;// Number of milliseconds between data receive from Pi
unsigned long prevSampleTime = 0;   // Time the last data receive occurred

void setup() {
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);

  // Begin serial communication
  Serial.begin(9600);

  // Get initial position
  initialPosition = motorEnc.read();
  Tc = millis();

  motor.init(); // Initialize motor
  motor.setM1Speed(0);
}

void loop() {
  // Read encoders and update current time
  newPosition = motorEnc.read();
  //while (newPosition >= 3200) newPosition -= 3200; // MODIFIED SO 3200 COUNTS CHANGES TO 0!!!!!!

  // Find current angular motor position relative to initial position
  currentAngle = float(newPosition - initialPosition) * float((2.0 * PI) / float(CPR));

  // Calculate desired angular position
  desiredAngle = desiredAngleCoeff * float(PI / 2);

  // Find current angular velocity in rad/s: (x2 - x1) / ∆t
  //angVelocity = ((float((newPosition - initialPosition) - (position - initialPosition)) * float((2.0 * PI) / CPR)) *float(1000)) / float(Ts);

  // use control() to determine target speed and direction
  targetSpeed = control(currentAngle, desiredAngle);

  // Set the motor to the target speed (also accounts for direction)
  motor.setM1Speed(targetSpeed);

  // Print elapsed time, target speed, angular velocity
  //Serial.print(millis() - start); // Prints elapsed time in ms
  //Serial.print("\t");
}

// Current and desired are angles in radians
int control(float current, float desired) {
  double newTargetSpeed, maxSpeed = 400, minSpeed = -400;   // 400 if using the motorShield header
  error = desired - current;      // Update error

  // Calculate D
  if (Ts > 0) {
    D = (error - pastError) / float(Ts);
    pastError = error;
  } else {
    D = 0;
  }

  // TODO when should I be reset to 0?????
  // If I keeps accumulating, so will the controller output, and the speed will increase to max and stay there.
  // Should it be reset when error gets close to zero?
  // Or should it only be reset when the desired position changes?
  // Or should I just remove I entirely from the controller?

  // Calculate I
  I += float(Ts) * error;

  // Calculate controller output
  newTargetSpeed = KP * error + KI * I + KD * D;

  // TODO make sure this is correct
  // The step response experiment used 255 as the max command.
  // So the controller output needs to be scaled by 400/255
  newTargetSpeed *= 400.0 / 255.0;

  // If calculated controller output is higher than the maximum output,
  // set output to the maximum output with appropriate sign
  if (abs(newTargetSpeed) > maxSpeed) {

    if (newTargetSpeed < 0) newTargetSpeed = minSpeed;
    else newTargetSpeed = maxSpeed;
    I -= float(Ts) * error; // Undo integration
  }

  Ts = millis() - Tc; // Determine sample time
  Tc = millis();      // Update current time

  Serial.print("\nCurrentAngle: ");
  Serial.print(current);
  Serial.print("\tDesiredAngle: ");
  Serial.print(desired);
  Serial.println("\terror: ");
  Serial.println(error);
  Serial.print("\tcontroller: ");
  Serial.println(newTargetSpeed);
  Serial.print("\tcontroller(int): ");
  Serial.println(int(newTargetSpeed));

  // newTargetSpeed = newTargetSpeed*50; // TODO should this be scaled????

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

      angleRead = true;
      desiredAngleCoeff = nextAngleCoef;
      prevSampleTime = currSampleTime;
      Serial.print("Desired Angle: ");
      Serial.println(desiredAngleCoeff);
    }
  }
}
