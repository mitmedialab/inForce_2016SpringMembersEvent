#pragma once
#ifndef ARDUINOPRESSUREREADER_H
#define ARDUINOPRESSUREREADER_H

#include "ofSerial.h"
#include "ofThread.h"

#define PRESSURE_LIST_DELIMITER 0xFF

class ArduinoPressureReader
#if THREADED
	: public ofThread 
#endif
{

public:
	ArduinoPressureReader(int deviceNumber, int baudRate, unsigned short nPins, 
#ifdef DO_BENCHMARKING
		uint64_t * StartTime,
#endif
	unsigned long long(*getTimeFun)(void));

	~ArduinoPressureReader();
	void closeSerial();
	void parseNextPinPressureBuffer();
	void parseNextPinPressureBufferAlt();
#if THREADED
	void threadedFunction();
#endif
	bool hasUpdated();
	void getPinPressures(double* buffer);

	// Time stuff
#ifdef DO_BENCHMARKING
	uint64_t* pStartTime = NULL;
	unsigned long startTime = 0;
#endif
//	unsigned long loopPeriod;
	unsigned long long(*getTime)(void);
	unsigned long long now = 0;
	unsigned long ellapsed = 0;
	unsigned long long prevTime = 0;
	unsigned long long prevNow = 0;

private:
	ofSerial serial;
	unsigned short numPins;
	unsigned char* rawPressuresBuffer;
	volatile bool updateFlag;	// Flag is set (1, True) when the pressured have been updated by the arduino interface thread and cleared (0, False) whehn those pressure values are converted to the double array
						// The main loop ony bothers doing anything 
    int maxBufferLengthWithSingleFrame; // the maximum number of bytes that the serial buffer can have available while still only containing one full frame							 // a "Frame" is one delimiter (assumed to be one byte) plus <numPins> bytes (assuming one byte per pin)
};

#endif
