//////////////////////////////////////////////////////////////////////// NAME:
// CLASS:     EENG-350
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

// Define data types needed for finite state machine
typedef enum {START, FOV_ROTATE, FIND_TAPE, TURN_TO_START, CALC_DIST_TO_START, DRIVE_TO_START, CALC_PATH_ANGLE, TURN_INLINE_TO_PATH, CALC_DIST_TO_END, DRIVE_TO_END, STOP} currentState_t;

///////////////////////////
// Communication with PI //
///////////////////////////
int data = 0;   // Value to store data received over I2C




////
void receiveData(int byteCount);
////


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
const float CPR = 3200;                                 //!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
const float RADIUS = 2.9375;                            //!< Measured radius of wheels in inches
const float BASE = 13.8;                                //!< Distance between center of wheels in inches
const float RAD_CONVERSION = float(2.0 * PI) / CPR;     //!< Scalar to convert counts to radians
const long CONTROL_SAMPLE_RATE = 5;                     //!< Controller sample rate in ms
const int MAX_SPEED = 400;                              //!< Maximum scaled PWM (max motor speed = 400)
const int MIN_SPEED = 84;                               //!< Minimum scaled PWM
#define ENC_R_WHITE 2                                   //!< Right motor encoder output B (white wire)
#define ENC_R_YELLOW 5                                  //!< Right motor encoder output A (yellow wire)
#define ENC_L_WHITE 3                                   //!< Left motor encoder output B (white wire)
#define ENC_L_YELLOW 6                                  //!< Left motor encoder output A (yellow wire)

// Controller globals
float rho = 0, targetRho = 0;                           //!< current and target distances in inches
float phi = 0, targetPhi = 0;                           //!< current and target angles in radians
float rhoOffset = 0;                                    //!< Contains initial forward counts after rotating
float motorDif, motorSum;                               //!< Parameters for speed control. motorDif [-400,400] and motorSum [-400, 400]
float error, pastErrorRho = 0, pastErrorPhi = 0;        //!< Variables used in calculating control output
float I_rho = 0, I_phi = 0;                             //!< Integral controller accumulations
unsigned long currentTime = 0, startTime = 0;           //!< For creating a discrete time controller

// State Flags and transmission codes
bool rotateComplete = false;      // Indicates if robot is done rotating 
#define ROTATE_COMPLETE_SET -127  // Transmitted to Pi when flag is set

bool tapeNotFound = false;        // Indicates if tape was not found in the field of view
#define TAPE_NOT_FOUND_SET -126   // Transmitted from Pi when flag is set

bool motionComplete = false;      // Indicates if robot has stopped moving forward
#define MOTION_COMPLETE_SET -125  // Transmitted to Pi when flag is set

bool flagSent = false;            // Indicates if a flag was sent to the Pi
bool dataReceived = false;        // Indicates if data was read from the Pi

////
bool tapeFound = false;             //!<flag to know whether to search for the tape or not
bool firstRho = true;               //!< Flag for accurately determining forward counts after rotating
bool rotating = true;               //!< Flag indicating the robot is currently turning
bool encReset = false;
////


// Pairs
Pair<int> targetSpeed;              //!< Scaled PWM values given to motors.setSpeeds() each ranging from -400 to 400
Pair<long> counts;                  //!< Left and right encoder readings (counts)

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
DualMC33926MotorShield motors;           //!< Motor 2 is the right wheel

void setMotors(float dif, float sum);

/**
 * Does initial setup
 */
void initialize() {
    // Begin serial communication
    Serial.begin(115200); // start serial for output
    
    // Initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // Define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData); 
    Serial.println("Ready!");

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

//////////////////////////
// Finite State Machine //
//////////////////////////
static currentState_t currentState = START;
void loop() {   
    int angleToStart;    // The angle to the start of the line 
    int distanceToStart; // The distance to the start of the line
    int angleToEnd;      // The angle to the end of the line
    int distanceToEnd;   // The distance to the end of the line
      
    switch (currentState) {
        
        // The START state runs at the start of the program, no code takes place
        case START:
//            Serial.println("START State");            
            currentState = FOV_ROTATE;
            break;


        // FOV_ROTATE State: Rotate the robot clockwise by slightly over half the field of view (30 degrees). The flag
        //                   rotateComplete will be set to -127 and sent to the Pi when the robot has finished rotating
        case FOV_ROTATE:
        
            rotateComplete = true;  //TODO: set flag true after robot finished rotating

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = FIND_TAPE; 
                flagSent = false;              
            }         
            break;


        // FIND_TAPE State: Wait for the Pi to find the angle to a line of tape in the camera's FOV and transmit it. The flag value
        //                  TAPE_NOT_FOUND_SET will be sent by the Pi if no tape is found. Otherwise the value sent will be the angle.
        case FIND_TAPE:
//            Serial.println("FIND_TAPE State");

            // If the Arduino received data from the Pi
            if (dataReceived) {
                // If the Pi indicated no tape was found in the screen, go back to FOV_ROTATE state
                if (data == TAPE_NOT_FOUND_SET) {
                    Serial.println("Tape Not Found");
                    currentState = FOV_ROTATE;
                // If the Pi indicated that it found tape, store the angle the Arduino received and move to TURN_TO_START state
                } else {
                    angleToStart = data;
                    Serial.print("angleToStart = ");
                    Serial.println(angleToStart);
                    currentState = TURN_TO_START;
                }
                dataReceived = false;   // Reset dataReceived flag
            }
            break;


        // TURN_TO_START State: Rotate the robot to be in line with the start of the tape path. Send the flag code ROTATE_COMPLETE_SET to the Pi
        //                      when the robot is done rotating
        case TURN_TO_START:
//            Serial.println("TURN_TO_START State");
            
            rotateComplete = true;  //TODO: set flag true after robot finished rotating

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = CALC_DIST_TO_START; 
                flagSent = false;              
            }
            break;


        // CALC_DIST_TO_START State: The Pi calculates the distance from the robot to the start of the tape path and transmits this value to the Arduino
        case CALC_DIST_TO_START:
//            Serial.println("CALC_DIST_TO_START State");

            // If the Arduino received data from the Pi
            if (dataReceived) {
                distanceToStart = data;
                Serial.print("distanceToStart = ");
                Serial.println(distanceToStart);
                currentState = DRIVE_TO_START;
                dataReceived = false;   // Reset dataReceived flag
            }
            break;


        // DRIVE_TO_START State: Drive the robot to the start of the tape path. Transmit the flag code MOTION_COMPLETE_SET to the Pi when the robot has
        //                       finished driving forward.
        case DRIVE_TO_START:
//            Serial.println("DRIVE_TO_START State");
            motionComplete = true;  //TODO: set flag true after robot finished moving forward

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = CALC_PATH_ANGLE;
                flagSent = false;              
            }
            break;          

        // CALC_PATH_ANGLE State: The Pi calculate the angle the robot needs to turn to to be in line with the tape path. This value is then sent to the Arduino
        case CALC_PATH_ANGLE:
//            Serial.println("CALC_PATH_ANGLE State");

            // If the Arduino received data from the Pi
            if (dataReceived) {
                angleToEnd = data;
                Serial.print("angleToEnd = ");
                Serial.println(angleToEnd);
                currentState = TURN_INLINE_TO_PATH;
                dataReceived = false;   // Reset dataReceived flag
            }
            break;   
            
        // TURN_INLINE_TO_PATH State: Turn the robot to be in line with the tape path. Transmit the flag code ROTATE_COMPLETE_SET to the Pi
        //                            when the robot is done rotating
        case TURN_INLINE_TO_PATH:
//            Serial.println("TURN_INLINE_TO_PATH State");/

            rotateComplete = true;  //TODO: set flag true after robot finished rotating

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = CALC_DIST_TO_END;
                flagSent = false;              
            }
            break;


        // CALC_DIST_TO_END State: The Pi calculate the distance from the robot to the end of the tape path. This value is then sent to the Arduino
        case CALC_DIST_TO_END:
//            Serial.println("CALC_DIST_TO_END State");

            // If the Arduino received data from the Pi
            if (dataReceived) {
                distanceToEnd = data;
                Serial.print("distanceToEnd = ");
                Serial.println(distanceToEnd);
                currentState = DRIVE_TO_END;
                dataReceived = false;   // Reset dataReceived flag
            }
            break;


        // DRIVE_TO_END State: Drive to the end of the tape path. Transmit the flag code MOTION_COMPLETE_SET to the Pi when the robot has
        //                     finished driving forward.
        case DRIVE_TO_END:
//            Serial.println("DRIVE_TO_END State");

            motionComplete = true;  //TODO: set flag true after robot finished moving forward

            // If the Arduino finished sending data to the Pi, reset the flag and move to the next state
            if (flagSent) {               
                currentState = STOP;
                flagSent = false;              
            }
            break;


        // STOP State: The end of the tape was reached. Exit the state machine
        case STOP:
            break;
    }   
}








////
//void loop() {
//    scanForTape();
//    runState();
//}
////

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

    while ((tapeFound==false) && phi < (2 * PI)) {
        
        while (phi < ((increment * 30)*(PI/180))) {
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
    
    if(encReset == true){ ////TODO must be set to true in FSM before calling drive function
        encoderReset();
        encReset = false;
    }    

    computeControllers();
    if (error < 1){
        return false;
    }
}

Pair<float> computeControllers() {
    // Controller Parameters
    const float KP_RHO = 41.507628, KI_RHO = 2, KD_RHO = 0.000000;      //!< Rho controller constants
    const float KP_PHI = 260.542014, KI_PHI = 5, KD_PHI = 0.000000;     //!< Phi controller constants

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

    return {motorDif, motorSum};
} // End computeControllers


float controlRho(float current, float desired, const float KP, const float KI, const float KD) {
    // TODO revert changes that made movement choppy. The camera should help correct the robot
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

    return output;
} // End controlRho

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

    return output;
} // End controlPhi

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
} // End setMotors

/**
 * Callback for received data (From mini project)
 * @param byteCount
 */
// callback for received data
void receiveData(int byteCount){
    dataReceived = true;                // Indicate that the Arduino received data from the Pi
    Serial.println("Data Received");
  
    // While there are still bytes to read, read them and store the most recent to number
    while(Wire.available()) {
        data = Wire.read();   
        
        // Convert to Two's compliment representation
        if (data > 127) {
            data = 256 - data;
            data *= -1; 
        }
        Serial.print("Data: ");
        Serial.println(data);
    }
} // End receiveData


// callback for sending data
void sendData(){
   
    Serial.println("Data Requested");

    // If a write request was received from the Pi
    switch (currentState) {
        case FOV_ROTATE:
        case TURN_TO_START:
        case TURN_INLINE_TO_PATH:
            // Wait until robot has finished rotating
            if (rotateComplete) {
                Wire.write(ROTATE_COMPLETE_SET);
                flagSent = true;
                Serial.println("FLAG Sent");
            }
            break;     
            
        case DRIVE_TO_START: 
        case DRIVE_TO_END:
            // Wait until robot has finished driving forward
            if (motionComplete) {
                Wire.write(MOTION_COMPLETE_SET);
                flagSent = true;
                Serial.println("FLAG Sent");
            }
            break; 
    }
    delay(100);
} // End sendData
