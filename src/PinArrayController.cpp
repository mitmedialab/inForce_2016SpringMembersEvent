#include "PinArrayController.h"
#include <chrono>

PinArrayController::PinArrayController(NodeID NumPins, unsigned long LoopPeriod):
	numPins(NumPins),
	loopPeriod(LoopPeriod)
{

	ofLog(OF_LOG_NOTICE, "Initializing Interface Objects...");
	pressureInterface = new ArduinoPressureReader(ARDUINO_PORT_NUMBER, ARDUINO_BAUD, (NodeID)NUM_DRIVES,getTime);
	pinController = new PinInterface((NodeID)NUM_DRIVES);

	// Set up pins
	initPinActivations();
	initDesiredPinPositions();
	initPinPositions();
	initDesiredPinForces();
	initPinPressures();
	initLastActivatedTimes();
	setupPIDs();
	ofLog(OF_LOG_NOTICE, "\nInitialized Pin Array Controller with " + ofToString(numPins) + " pins.");

	//pinController->testMultipleDrives();
}

PinArrayController::~PinArrayController() {
	
	this->stopThread();
	enabled = false;
//	usleep(200);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

	ofLog(OF_LOG_NOTICE, "Shutting down drives...");
	pinController->shutdownAllDrives();
	pinController->tearDownCAN();

	ofLog(OF_LOG_NOTICE, "Closing Serial...");
	pressureInterface->closeSerial();

	delete pressureInterface;
	delete pinController;

	// Clean up PIN stuff
	ofLog(OF_LOG_NOTICE, "Deleting pins...");
	for (unsigned short i = 0; i < numPins; i++) {
		delete[] PIDs[i];
	}
	delete[] PIDs;
	delete PIDOutputs; 
	delete[] pinActivations;
	delete[] desiredPinPositions;
	delete[] pinPositions;
	delete[] desiredPinForces;
	delete[] pinPressures;

	//usleep(2000);
}

////////////////////////////////////////// TIME STUFF //////////////////////////////////////////
unsigned long long PinArrayController::getTime(){
	//std::chrono::steady_clock::time_point startTime;
	//return ofGetElapsedTimeMicros();
	//std::chrono::duration_cast<std::chrono::duration<uint64_t>>
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	//std::chrono::duration_cast<std::chrono::duration<double>>std::chrono::high_resolution_clock::now();
	//return std::chrono::steady_clock::now();
}

/*Inits and sets the pinActivations bool array which keeps track of which pins are activated.*/
void PinArrayController::initPinActivations() {
	pinActivations = new bool[numPins];
	// Start by assuming no pins are being touched
	for (unsigned short  i = 0; i < NUM_DRIVES; i++) {
		pinActivations[i] = false;
	}
}

/*Inits and sets the desired pin positions array which keeps track of the desired pin positions when not activated.*/
void PinArrayController::initDesiredPinPositions() {
	// TODO: replace dummy values, use this as API to model simulation positions output
	desiredPinPositions = new double[numPins];
	double desiredPosition = 108758;// POSITION_MAX;//(POSITION_MAX + POSITION_MIN) / 2.0;
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		desiredPinPositions[i] = desiredPosition;
	}
}

void PinArrayController::initPinPositions() {
	pinPositions = new double[numPins];
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		pinPositions[i] = desiredPinPositions[i]; // Set the initial positions as desired
	}
}

/*Inits and sets the desired pin forces array which keeps track of the desired pin forces when activated.*/
void PinArrayController::initDesiredPinForces() {
	// TODO: replace dummy values, use this as API to model simulation forces output
	desiredPinForces = new double[numPins];
	//double desiredForceRange = PIN_MAX_CONTROLLABLE_PRESSURE - PIN_MIN_CONTROLLABLE_PRESSURE;
	//double denominator = NUM_DRIVES + 1; // use just NUM_DRIVES if you want the last drive to exhert the max force
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		desiredPinForces[i] = 0;// PIN_MIN_CONTROLLABLE_PRESSURE + ((i + 1.0) / denominator) * desiredForceRange;
	}
}

void PinArrayController::initPinPressures() {
	pinPressures = new double[numPins];
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		pinPressures[i] = 0;
	}
}

void  PinArrayController::initLastActivatedTimes() {
	lastActivatedTimes = new unsigned long long[numPins];
	lastDeactivatedTimes= new unsigned long long[numPins];
	nowTime = getTime();
	for (unsigned short i = 0; i < numPins; i++) {
		lastActivatedTimes[i] = nowTime;
		lastDeactivatedTimes[i] = nowTime;
	}
}

/*initializes the arrays used to keep track of curren pin positions and pressures/forces, sets time variables, then initialized the PID control objects for each pin.*/
void PinArrayController::setupPIDs() {
	ofLog(OF_LOG_NOTICE, "Setting up PIDs...");

	PIDOutputs = new double[numPins];
	PIDs = new PID*[numPins];

	// init time stuff to safe values
	nowTime = getTime();
	prevNow = nowTime;
	prevTime = nowTime;
	ellapsedTime = 0;

	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		// NOTE: PIDs all start in DIRECT mode, not AUTO (they start dissabled)
		PIDs[i] = new PID(	&(pinPressures[i]), &(PIDOutputs[i]), &(desiredPinForces[i])
		, &proportionalGain, &integralGain, &derivativeGain
#ifdef USE_PID_LINEARIZATION
			, &PIDSetpointMultiplier
#endif
		, DIRECT, POSITION_MAX, &ellapsedTime);
	}
}

void PinArrayController::setLoopPeriod(unsigned long period) {
	loopPeriod = period;
}
void PinArrayController::setPIDGains(double KP, double KI, double KD) {
	proportionalGain = KP;
	integralGain = KI;
	derivativeGain = KD;
}
void PinArrayController::setPinTouchDebounceTimes(unsigned long debounceActivationTime, unsigned long debounceDeactivationTime) {
	pinActivationDebounceTime = debounceActivationTime;
	pinDeactivationDebounceTime = debounceDeactivationTime;
}
#ifdef USE_PID_LINEARIZATION
void PinArrayController::setPIDSetPointMultiplier(double m) {
	PIDSetpointMultiplier = m;
}
#endif

void PinArrayController::sendPinPositions() {
	if (enabled) {
		for (NodeID id = 1; id <= NUM_DRIVES; id++) {
			pinController->sendDriveToPosition(id, (int)pinPositions[id - 1]);
		}

		// PCAN Bus error detection and handling
		TPCANStatus PCANStatus = pinController->getPCANStatus();
		if (PCANStatus != PCAN_ERROR_OK) {
			cout << "\nPCAN ERROR: " << PinInterface::PCAN_STATUS_STRING(PCANStatus);
//			usleep(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

		}
	}
}

void PinArrayController::updatePinPressures() {
	pressureInterface->getPinPressures(pinPressures);
}

void PinArrayController::calcPinPressurePIDs() {
	//nowTime = getTime();
	if (forceControlEnabled) {
		for (unsigned short i = 0; i < NUM_DRIVES; i++) {

			// LOGIC FOR POSITION VS> FORCE CONTROL

			// Pin has pressure (is being touched) - use force control
			if (pinPressures[i] > PIN_ACTIVATION_PRESSURE) {

				// If the pin was not active before, then we need to flag that it is currently active and re-enable it's PID
				if (!pinActivations[i] && (nowTime - lastActivatedTimes[i] >= pinActivationDebounceTime)) {
					pinActivations[i] = true;
					PIDs[i]->SetMode(AUTOMATIC);
					lastDeactivatedTimes[i] = nowTime;
				}

				// Pin has no pressure on it (not being touched) - position control
			}
			else {

				// If the pin was previously active (was being touched) but is no longer, then we need to flag that it is currently inactive and dissable it's PID
				if (pinActivations[i] && (nowTime - lastDeactivatedTimes[i] >= pinDeactivationDebounceTime)) {
					pinActivations[i] = false;
					PIDs[i]->SetMode(MANUAL);
					pinPositions[i] = desiredPinPositions[i]; // we also need to make sure to set the position for position control instead of letting PID control position through force control
					lastActivatedTimes[i] = nowTime;
				}
			}

			PIDs[i]->Compute(); // even if the PID isn't AUTOMATIC (active), we still call compute() to the PID can keep track of time and derivatives - it just wont output anything

			// if the pid is active and we care about it's output, then add it's output to the position (speed output control signal)
			if (pinActivations[i]) {
				pinPositions[i] += PIDOutputs[i];

				// Range sanity check
				if (pinPositions[i] > POSITION_MAX) {
					pinPositions[i] = POSITION_MAX;
				}
				else if (pinPositions[i] < 0) {
					pinPositions[i] = 0;
				}
			}
		}
	}
}

void PinArrayController::startPins() {
	// Init the pins
	ofLog(OF_LOG_NOTICE, "Starting pins...");
	pinController->shutdownAllDrives();
//	usleep(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	pinController->initializeAllDrives();
	pinController->homeAllDrivesAtCurrentPosition();
//	usleep(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	enabled = true;
	sendPinPositions();
//	usleep(1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

}

void PinArrayController::mainLoop() {
//	unsigned long mainStart = getTime();

	// Screw it - sometimes we don't get new position data before the new control loop - we don't know why.
	pressureInterface->parseNextPinPressureBuffer();
	if (!pressureInterface->hasUpdated()) {
		//ofLog(OF_LOG_WARNING, "WARNING - Controller: Pressures not updated since last control loop!");
	}

//	unsigned long mainRead = getTime();

	// Update drives
	updatePinPressures();
	calcPinPressurePIDs();
	sendPinPositions();

//	unsigned long mainEnd = getTime();
	//cout << "\nMain read: " << mainRead - mainStart << "\tmain control: " << mainEnd - mainRead << ".";
}

// INTERFACE
void PinArrayController::setDesiredPinHeights(int* Positions) {
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		desiredPinPositions[i] = (double)Positions[NUM_DRIVES - i - 1];
		if (!pinActivations[i]) {
			pinPositions[i] = desiredPinPositions[i];
		}
	}
}
void PinArrayController::setDesiredPinForces(int* Pressures) {
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		desiredPinForces[i] = (double)Pressures[NUM_DRIVES - i - 1];
	}
}
void PinArrayController::getCommandedPinPositions(int* pinPositionsBuffer) {
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		pinPositionsBuffer[NUM_DRIVES-i-1] = (int)(pinPositions[i]);
	}
}
void PinArrayController::getSensedPinPressures(int* pinPressuresBuffer) {
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		pinPressuresBuffer[NUM_DRIVES-i-1] = (int)(pinPressures[i]);
	}
}
void PinArrayController::getPinActivations(bool* pinActivationsBuffer) {
	for (unsigned short i = 0; i < NUM_DRIVES; i++) {
		pinActivationsBuffer[NUM_DRIVES-i-1] =(bool)(pinActivations[i]);
	}
}
PinInterface* PinArrayController::getPinController() {
	return pinController;
}
void PinArrayController::enableForceControl() {
	for (unsigned short i = 0; i < numPins; i++) {
		PIDs[i]->SetMode(AUTOMATIC);
	}
	forceControlEnabled = true;
}
void PinArrayController::dissableForceControl() {
	forceControlEnabled = false;
	for (unsigned short i = 0; i < numPins; i++) {
		PIDs[i]->SetMode(MANUAL);
		PIDOutputs[i] = 0;
	}
}


/* This function is called when the thread starts*/
void PinArrayController::threadedFunction() {

	ofLog(OF_LOG_NOTICE, "Starting Pin Array Controller Thread...");
	// Set the start time
	startTime = getTime();
	nowTime = startTime;
	ellapsedTime = 0;
	prevTime = startTime;
	prevNow = startTime;

	while(enabled) { // infinite loop thread
		// Twiddle our thumbs until the next period comes around
		nowTime = getTime();
		while (nowTime - prevTime < loopPeriod && enabled) {
			nowTime = getTime();
			
			if (nowTime == prevNow) {
				continue;
			}
			prevNow = nowTime;
			//ofSleepMillis(1);
		}

		ellapsedTime = (unsigned long long)(nowTime - prevTime);
		prevTime = nowTime;

		//cout << "\nElapsd " << ellapsedTime;
		if (enabled) {
			mainLoop();
		}

	}
	ofLog(OF_LOG_NOTICE, "Exiting Control Thread...");
}
