#pragma once
#ifndef PID_v1_h
#define PID_v1_h
//#define LIBRARY_VERSION	1.1.1

/**********************************************************************************************
* Arduino PID Library - Version 1.1.1
* by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/

class PID {


public:

	//Constants used in some of the functions below
#define AUTOMATIC	true
#define MANUAL	false
#define DIRECT  false
#define REVERSE  true

	//commonly used functions **************************************************************************
	PID(double* Input, double* Output, double* Setpoint,
		double* Kp, double* Ki, double* Kd
#ifdef USE_PID_LINEARIZATION
		, double* Mult
#endif
		, bool ControllerDirection, int OutputMaxLimit, unsigned long long* TimeChange);     //   Setpoint.  Initial tuning parameters are also set here

	void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

	bool Compute();                       // * performs the PID calculation.  it should be
										  //   called every time loop() cycles. ON/OFF and
										  //   calculation frequency can be set using SetMode
										  //   SetSampleTime respectively

	void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



										  //available but not commonly used functions ********************************************************
//	void SetTunings(double, double,       // * While most users will set the tunings once in the 
//		double);         	  //   constructor, this function gives the user the option
							  //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
//	void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
										  //   the PID calculation is performed.  default is 100



										  //Display functions ****************************************************************
//	double GetKp();						  // These functions query the pid for interal values.
//	double GetKi();						  //  they were created mainly for the pid front-end,
//	double GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

private:
	void Initialize();

//	double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
//	double dispKi;				//   format for display purposes
//	double dispKd;				//

	double* kp;                  // * (P)roportional Tuning Parameter
	double* ki;                  // * (I)ntegral Tuning Parameter
	double* kd;                  // * (D)erivative Tuning Parameter
	double* mult;				 // multiplies the error to linearly scale all other components
	int controllerDirection;

	unsigned long long* timeChange;
	double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
	double *myOutput;             //   This creates a hard link between the variables and the 
	double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
								  //   what these values are.  with pointers we'll just know.
	double input;
	double error;

//	unsigned long(*micros)();	// Pointer to a function that gives the time in ms
//	unsigned long lastTime;
	double ITerm, lastInput;

//	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto;
};
#endif