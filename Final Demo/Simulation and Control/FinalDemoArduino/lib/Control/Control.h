////////////////////////////////////////////////////////////////////////
// NAME: 		Alex Curtis
// CLASS:		EENG-350
// GROUP: 		5
// TITLE:   	Control
// FUNCTION:	Self-contained controller class responsible for controlling the robot's motion
// HARDWARE: 	Any hardware connections you must make to your device
// SOFTWARE: 	Any software that must be installed to the device
// EXECUTE: 	Execution instructions for your program
// RESOURCE: 	Link to any resource you used
// PURPOSE:  	What the resource was used for
// RESOURCE: 	Link to any resource you used
// PURPOSE:  	What the resource was used for
//////////////////////////////////////////////////////////////////////

#ifndef Control_h
#define Control_h
#include "Arduino.h"

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

class Control {
public:
	Control(); //!< Default constructor
	// Constants
	const float CPR = 3200;                                 //!< Total encoder counts per revolution (CPR) of motor shaft = 3200 counts/rot
	const float RADIUS = 2.9375;                            //!< Measured radius of wheels in inches
	const float BASE = 13.8;                                //!< Distance between center of wheels in inches
	const float RAD_CONVERSION = float(2.0 * PI) / CPR;     //!< Scalar to convert counts to radians
	// Public variables
	const float KP_RHO = 41.507628, KI_RHO = 17.532965, KD_RHO = 0.000000; //!< Rho controller constants
	const float KP_PHI = 260.542014, KI_PHI = 124.273019, KD_PHI = 0.000000; //!< Phi controller constants
	float rho = 0;                           //!< current and target distances in inches
	float phi = 0;                           //!< current and target angles in radians
	// Public methods
	bool drive(float targetPhi, float targetRho);	//!< Drive to a targetPhi and targetRho
	// Status
	void getPositions();
private:
	const long CONTROL_SAMPLE_RATE = 5; //!< Controller sample rate in ms
	const int MAX_SPEED = 400;			//!< Maximum value to send to the motors
	float rhoOffset = 0;              //!< Contains initial forward counts after rotating
	float motorDif = 0, motorSum = 0; //!< Parameters for speed control. motorDif [-400,400] and motorSum [-400, 400]
	float error = 0, pastErrorRho = 0, pastErrorPhi = 0; //!< Variables used in calculating control output
	float I_rho = 0, I_phi = 0;      //!< Integral controller accumulations
	unsigned long currentTime = 0, startTime = 0, lastTime = 0;  //!< For creating a discrete time controller
	unsigned long MIN_SETTLING_TIME = 300; //!< Time in ms to wait for motors to settle
	bool firstRho = true; 			//!< Flag for accurately determining forward counts after rotating
	bool driveStarted = false;		//!< Set to true when startControl() is called for the first time
	void startControl(); 		//!< effectively setup()
	static void stopControl();	//!< Resets encoders and stops the motors
	float controlRho(float current, float desired);
	float controlPhi(float current, float desired);
	bool isDone();
	void setMotors(float diff, float sum) const;
};

#endif //Control_h
