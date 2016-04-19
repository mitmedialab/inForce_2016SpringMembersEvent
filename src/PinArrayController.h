#pragma once
#ifndef PIN_ARRAY_CONTROLLER_H
#define PIN_ARRAY_CONTROLLER_H

#include "ofMain.h"
#include "ofThread.h"

#include "PID.h"
#include "PinInterface.h"
#include "ArduinoPressureReader.h"

class PinArrayController : public ofThread {
public:
	PinArrayController(NodeID NumPins, unsigned long LoopPeriod);
	~PinArrayController();
	void threadedFunction();
	void startPins();

	// Tuning and behavior adjustment
	void setLoopPeriod(unsigned long period);
	void setPIDGains(double, double, double);
	void setPinTouchDebounceTimes(unsigned long debounceActivationTime, unsigned long debounceDeactivationTime);
	void setPIDSetPointMultiplier(double m);

	// INTERFACE
	void setDesiredPinHeights(int* Positions);
	void setDesiredPinForces(int* Pressures);
	void getCommandedPinPositions(int* pinPositionsBuffer);
	void getSensedPinPressures(int* pinPressuresBuffer);
	void getPinActivations(bool* pinActivationsBuffer);

	void enableForceControl();
	void dissableForceControl();

	PinInterface* getPinController();
private:
	bool enabled = false;
	bool forceControlEnabled = true;
	double proportionalGain = DEFAULT_PROPORTIONAL_GAIN;
	double derivativeGain = DEFAULT_DERIVATIVE_GAIN;
	double integralGain = DEFAULT_INTEGRAL_GAIN;
	unsigned long pinActivationDebounceTime = DEFAULT_PIN_ACTIVATION_TOUCH_DEBOUNCE_WINDOW_US;
	unsigned long pinDeactivationDebounceTime = DEFAULT_PIN_DEACTIVATION_TOUCH_DEBOUNCE_WINDOW_US;
#ifdef USE_PID_LINEARIZATION
	double PIDSetpointMultiplier = DEFAULT_PID_SETPOINT_MULTIPLIER;
#endif
	NodeID numPins;
	PID** PIDs = NULL;
	PinInterface* pinController = NULL;
	ArduinoPressureReader* pressureInterface = NULL;

	// Time stuff
	static inline unsigned long long getTime();
	unsigned long loopPeriod = 0;
	unsigned long startTime = 0;
	unsigned long long nowTime = 0;
	unsigned long long ellapsedTime = 0;
	unsigned long long prevTime = 0;
	unsigned long long prevNow = 0;

	// the output of the PID controls is added to the pin positions each loop (effectivly, controlling force with speed)
	// The reasoning is that the error will be centered around 0, so the control signal shoudld be too.
	// Otherwise, we get instan negative-value positions as soon as there's a negative error (any high pressure sends the pins to the min)

	bool* pinActivations; // Keep track of pin touches
	void initPinActivations();

	double* desiredPinPositions;
	void initDesiredPinPositions();

	double* pinPositions; // This buffer contains the COMMANDED pin positions at any given time
	void initPinPositions();

	double* desiredPinForces;
	void initDesiredPinForces();

	double* pinPressures; // This array stores the last-received pressures sensed by each pin, converted to doubles
	void initPinPressures();

	unsigned long long* lastActivatedTimes;
	unsigned long long* lastDeactivatedTimes;
	void initLastActivatedTimes();

	double* PIDOutputs;
	void setupPIDs();

	inline void updatePinPressures();
	inline void sendPinPositions(); // TODO: Put pin position wiriting in another thread - unless PCAN doesn't block when sending messages?
	inline void calcPinPressurePIDs();
	inline void mainLoop();

};


#endif