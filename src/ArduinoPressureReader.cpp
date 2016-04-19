
#include "ArduinoPressureReader.h"
#include "ofMain.h"

#include "Parameters.h"

ArduinoPressureReader::ArduinoPressureReader(int deviceNumber, int baudRate, unsigned short nPins, 
	#ifdef DO_BENCHMARKING
	uint64_t* StartTime, 
	#endif 
	unsigned long long(*getTimeFun)(void)) :

	numPins(nPins),
	updateFlag(false),
	maxBufferLengthWithSingleFrame(2 * (nPins + 1) - 1), // the maximum number of bytes that the serial buffer can have available while still only containing one full frame
	getTime(getTimeFun)
{
	rawPressuresBuffer = new unsigned char[numPins];

	//cout << "\nSetting up Arduino Serial...";
	//cout << "\nAvailable Serial Devices:";
	ofLog(OF_LOG_NOTICE, "Available Serial Devices:");
	//serial.listDevices(); // Print the available serial port devices
	vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
	serial.setup(deviceNumber, baudRate); //open the first device and talk to it at 57600 baud
	//cout << "\nConnected to device number " << deviceNumber << "(" << (deviceList[deviceNumber]).getDeviceName() << ")" << " at baut rate " << baudRate;
	//ofLog(OF_LOG_NOTICE, "\nConnected to serial device number ", deviceNumber, "(", (deviceList[deviceNumber]).getDeviceName(), ")", " at baut rate ", baudRate);
}

ArduinoPressureReader::~ArduinoPressureReader() {
	closeSerial();
}

void ArduinoPressureReader::closeSerial() {
	serial.close();
}

/*
// check for data in the serial buffer, find the delimiter, put the next <numPins> bytes into our pressuresBuffer array
void ArduinoPressureReader::parseNextPinPressureBuffer() {
	
	unsigned long long timeAtStart = getTime();
	// TODO: What strategy do we want for reading pressure list frames?
	// A) ("greedy") Assume that we're running fast enough that the buffer is usually empty. Wait for bytes to be available, read until we find the next delimiter, wait for the rest of the frame, copy to pressures array.
	// B) ("lossy") Assume we're running too slow to keep up with the buffer, which is usually getting full. Clear almost all the buffer except for the last <maxBufferLengthWithSingleFram> bytes, the find the "last full frame" in there.
	// Can they be used together (A must be used on program startup when the buffer is full.)

	unsigned long long parseStart = getTime();

	// Clear any frames preceeding the most recent (last) one - discard them
	int initialAvailable = serial.available();
	int extraPrecedingBytes = initialAvailable - maxBufferLengthWithSingleFrame;
	bool hadToTrim = false;
	if (extraPrecedingBytes >  0) {
		unsigned char* bytes = new unsigned char[extraPrecedingBytes];
		serial.readBytes(bytes, extraPrecedingBytes);
		hadToTrim = true;
	}
	// at this point the serial buffer should have <maxBufferLengthWithSingleFrame> or less bytes available
	
	unsigned long long parseTrim = getTime();

	// NEXT: Wait for more bytes to arrive in the buffer, if needed
	int available = serial.available();
	bool neededToWaitForNext = (available <= 0);
	while (available <= 0) {
		available = serial.available();
	}
	// At this point the serial buffer should have <maxBufferLengthWithSingleFrame> or less but more than zero bytes

	unsigned long long parseNext = getTime();

	// DELIMITER: Find and discard the next delimiter, and anything before it
	unsigned char delimiter = '0';
	unsigned int delimitersTested = 0;
	serial.readBytes(&delimiter, 1);
	while (delimiter != PRESSURE_LIST_DELIMITER) {
		
		// Wait for more bytes if theres nothing
		available = serial.available();
		while (available <= 0) {
			available = serial.available();
		}

		serial.readBytes(&delimiter, 1);
		//delimiter = serial.readByte();
		delimitersTested++;
	}

	unsigned long long parseDelimiter = getTime();

	// FILL: Wait for the rest of the buffer to fill if needed, then read the next <numPins> into the buffer
	int numToFill = (int)numPins - serial.available();
	bool neededToFill = (numToFill > 0);
	if (neededToFill) {
		available = serial.available();
		while (available <= (int)numPins) {
			available = serial.available();
		}
	}
	
	unsigned long long parseFill = getTime();

#ifdef USE_MUTEXES
	lock();
#endif

	int bytesRead = serial.readBytes(rawPressuresBuffer, numPins);
	updateFlag = true;

#ifdef USE_MUTEXES
	unlock();
#endif

	unsigned long long parseRead = getTime();
	std::setw(10);
	cout << '\n' << timeAtStart
		<< "\tParse (" << initialAvailable << ") \t: " << (hadToTrim?"TRIM ": "Trim ") << parseTrim - parseStart << "\t(" << extraPrecedingBytes
		<< (neededToWaitForNext?")\tNEXT ":")\tNext ") << parseNext - parseTrim 
		<< "\tdelim " << parseDelimiter-parseNext << " \t(" << delimitersTested 
		<< (neededToFill?")\tFILL ":")\tFill ") << parseFill - parseDelimiter << "    \t(" << numToFill 
		<< ")\tRead " << parseRead - parseFill << ".";

	serial.flush();

	// Check for errors
	if (bytesRead != numPins) {
		ofLog(OF_LOG_NOTICE, "WARNING: Not all ", numPins, " pin pressures could be read from serial buffer!.");
	}
	if (bytesRead == OF_SERIAL_ERROR) {
		ofLog(OF_LOG_ERROR, "ERROR reading serial bytes!");
	}
}
*/

unsigned long prevCallTime = 0;
void ArduinoPressureReader::parseNextPinPressureBuffer() {
	unsigned long startTime = getTime();

	int bufferSize = serial.available();
	
	unsigned long availableTime = getTime();
	unsigned long bufferCopyTime = availableTime;

	// We need at-least <numPins> + 1 bytes in out buffer to have any hope of holding a whole frame of values plus a delimiter
	if (bufferSize <= numPins) {
//		cout << "\nWarning! Serial buffer not filled before next call to parseNextPinPressuresBufferAlt()";
	}else{
		unsigned char* buf = new unsigned char[bufferSize];
		int bytesRead=serial.readBytes(buf, bufferSize);

		bufferCopyTime = getTime();

		if (bytesRead != bufferSize) {
			ofLog(OF_LOG_NOTICE, "WARNING: Not all bytes could be read from serial buffer!.");
		
		}else {

			int delimiterIndex = 0; // assume we happen to have landed on a new frame delimiter

			// If our buffer contains multiple frames, skip all the ones before the last few we coyuld possibly care about (that could contain the last received frame)
			if (bufferSize > maxBufferLengthWithSingleFrame) {
				delimiterIndex = bufferSize - maxBufferLengthWithSingleFrame;
			}

			// Find the delimiter
			int lastPossibleDelimiterIndex = bufferSize - numPins;
			bool delimiterFound = true;
			while (buf[delimiterIndex] != PRESSURE_LIST_DELIMITER) {
				delimiterIndex++;
				if (delimiterIndex > lastPossibleDelimiterIndex) {
					delimiterFound = false;
					break;
				}
			}
			if (delimiterFound) {

#ifdef USE_MUTEXES
				lock();
#endif
				// DelimiterIndex should not actually be at the index of the delimiter of the last received frame. Copy these values to our internal rawPressuresBuffer
				std::memcpy(rawPressuresBuffer, &(buf[delimiterIndex + 1]), numPins);
				updateFlag = true;
#ifdef USE_MUTEXES
				unlock();
#endif

//				cout << "\nSUCCESS: Pressures read from buffer " << bufferSize;

			// Delimiter was not found
			}else{
				cout << "\nWARNING: No delimiter found in serial buffer!";
			}

			delete buf;
		}

		// Check for errors
		if (bytesRead == OF_SERIAL_ERROR) {
			ofLog(OF_LOG_ERROR, "\nERROR reading serial bytes!");
		}
	}

	unsigned long endTime = getTime();
//	cout << "\n\nStart: " << startTime << "\tEnd " << endTime << "\t(" << endTime - startTime << ")\tE " << startTime- prevCallTime;
	prevCallTime = startTime;

}

/*Returns true if new pressure values have arrived since the last call to getPinPressures(), false otherwise*/
bool ArduinoPressureReader::hasUpdated() {
	#ifdef USE_MUTEXES
	lock();
	#endif

	bool result = updateFlag;

#ifdef DO_BENCHMARKING
	unsigned long prevReadingsSinceLastGet = readingsSinceLastGet;
	readingsSinceLastGet = 0;
#endif

	#ifdef USE_MUTEXES
	unlock();
	#endif

#ifdef DO_BENCHMARKING
	if (result) {
		hits++;
//		cout << "Y";
//	}else {
//		cout << "N";
	}
	queries++;
#endif

	//cout << "\nQueried with readings since last get: " << readingsSinceLastGet << "\tQueries: " << queries<< "\tHits: "<<hits<<"\treadings: "<< numReadings<<".";

	return result;
}

/*Loads the given double[] buffer with the most recent pressure values.*/
void ArduinoPressureReader::getPinPressures(double* pinPressures) {
	#ifdef USE_MUTEXES
	lock();
	#endif
	
	for (unsigned short i = 0; i < numPins; i++) {
		pinPressures[i] = (double)rawPressuresBuffer[i];
	}
	updateFlag = false;

	#ifdef USE_MUTEXES
	unlock();
	#endif
}