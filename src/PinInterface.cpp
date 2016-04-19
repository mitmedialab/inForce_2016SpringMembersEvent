#include "PinInterface.h"

////////////////////////////////////////// PUBLIC PIN CONTROL FUNCTIONS //////////////////////////////////////////

PinInterface::PinInterface(NodeID numPins, TPCANBaudrate m_Baudrate, TPCANType m_HwType, unsigned int ioPort, unsigned short interrupt, TPCANHandle handle):
	m_PcanHandle(handle),
	numPins(numPins)
{
//	cout << "\nSetting up CAN for PinInterface for baudrate:" << m_Baudrate << "\thardware type " << m_HwType << "\tioPort " << ioPort << "\tinterrupt " << interrupt<<".";
	
	// Connects a selected PCAN-Basic channel
	TPCANStatus stsResult = CAN_Initialize(m_PcanHandle, m_Baudrate, m_HwType, ioPort, interrupt);
	if (stsResult != PCAN_ERROR_OK) {
		cout << stsResult;
	}

	if (MESSAGES_FILLED) {
		ofLog(OF_LOG_NOTICE, "PCAN Messages valid.");
	}
	else {
		ofLog(OF_LOG_ERROR, "\nPCAN Messages INVALID.");
	}

//	pinPositions = new int[numPins];

	ofLog(OF_LOG_NOTICE, "Set up CAN with PCAN version " +ofToString(PCAN_API_VERSION)+ " and PCAN channel version " + ofToString(PCAN_CHANNEL_VERSION)+ ".");
	
	ofLog(OF_LOG_NOTICE, "\nSet up CAN for PinInterface for baudrate:" + ofToString(m_Baudrate) + "\thardware type " + ofToString(m_HwType)+ "\tioPort " + ofToString(ioPort)+ "\tinterrupt "+ ofToString(interrupt)+ ".");

}

PinInterface::~PinInterface() {
	tearDownCAN();
}

void PinInterface::tearDownCAN(){
	CAN_Uninitialize(m_PcanHandle);
}

TPCANStatus PinInterface::getPCANStatus() {
	return CAN_GetStatus(m_PcanHandle);
}

void PinInterface::sendCANMessage(DWORD id, BYTE msgData[8]) {
	TPCANMsg canMsg;
	canMsg.ID = id;
	canMsg.LEN = 8;
	canMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
	copyBytes(msgData, canMsg.DATA, 8);
	CAN_Write(m_PcanHandle, &canMsg);
}

void PinInterface::sendPDOMessage(NodeID nodeID, BYTE messageData[8]) {
	sendCANMessage(nodeID2COB(nodeID), messageData);

#ifdef PRINT_CAN_MESSAGES
	cout << "\n";
	for (unsigned short i = 0; i < 8; i++) {
		cout << std::hex << setfill('0') << setw(2) << (int)(messageData[i]) << " ";
	}
#endif
	// << messageData[1] << messageData[2] << messageData[3] << messageData[4] << messageData[5] << messageData[6] << messageData[7];
}

void PinInterface::initializeDrive(NodeID id) {
	sendPDOMessage(id, SHUT_DOWN_MSG);
	sendPDOMessage(id, SWITCH_ON_MSG);
	sendPDOMessage(id, ENABLE_OPERATION_MSG);
}

void PinInterface::initializeAllDrives() {
	for (NodeID id = 1; id <= this->numPins; id++) {
		initializeDrive(id);
	}
}

void PinInterface::shutdownDrive(NodeID id) {
	sendPDOMessage(id, SHUT_DOWN_MSG);
}

void PinInterface::shutdownAllDrives() {
	// Send all drives to zero (home)
	for (NodeID id = 1; id <= numPins; id++) {
		sendDriveToPosition(id, (int)POSITION_MIN + 10000);
		usleep(2);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	// wait for them to get there
//	usleep(1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


	// Turn them off
	for (NodeID id = 1; id <= numPins; id++) {
		shutdownDrive(id);
	}
}

void PinInterface::homeDrive(NodeID id) {
	sendPDOMessage(id, MODE_OF_OPERATION_HOMING_MSG);
	sendPDOMessage(id, HOMING_METHOD_WITHOUTINDEXPULSE_MSG);
	sendPDOMessage(id, HOMING_START_MSG);
	sendPDOMessage(id, MODE_OF_OPERATION_POSITION_MSG);
}

void PinInterface::homeAllDrives() {
	for (NodeID id = 1; id < numPins; id++) {
		homeDrive(id);
	}
	// TODO: receive messages confirming that homing is done for each drive. Return when all drives have homed.
	//usleep(3000);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

}

void PinInterface::homeDriveAtCurrentPosition(NodeID id) {
	sendPDOMessage(id, MODE_OF_OPERATION_HOMING_MSG); // 1MODES_OF_OPERATION6 = 2F 60 60 00 06 00 00 00
	sendPDOMessage(id, HOMING_METHOD_CURRENTPOS_MSG); // 1HOMING_METHOD35 = 2F 98 60 00 23 00 00 00
	sendPDOMessage(id, HOMING_START_MSG); // 1HOMING_START = 2B 40 60 00 1F 00 00 00
	sendPDOMessage(id, MODE_OF_OPERATION_POSITION_MSG); // 1MODES_OF_OPERATION1 = 2F 60 60 00 01 00 00 00
}

void PinInterface::homeAllDrivesAtCurrentPosition() {
	for (NodeID id = 1; id < numPins; id++) {
		homeDriveAtCurrentPosition(id);
	}
}

void PinInterface::readCANMessages() {
	//TODO: Magic
}

// TODO: update pinPositions with the actual pin positions read from CAN
/*void PinInterface::getPinPositions(int positionsBuffer[]) {
	std::memcpy(pinPositions, positionsBuffer, numPins*sizeof(int));
}*/

////////////////////////////////////////// TEST FUNCTIONS //////////////////////////////////////////
void PinInterface::testPositionSpeed(NodeID id) {
	int numRepetitions = 3;
	for (int i = 0; i < numRepetitions; i++) {
		sendDriveToPosition(id, POSITION_MAX);
        std::this_thread::sleep_for(std::chrono::milliseconds(TEST_TIME_SETTLE_MS));
		//usleep(TEST_TIME_SETTLE_MS);
		sendDriveToPosition(id, 400);
        std::this_thread::sleep_for(std::chrono::milliseconds(TEST_TIME_SETTLE_MS));
		//usleep(TEST_TIME_SETTLE_MS);
	}
}
void PinInterface::testPositionPrecision(NodeID id) {
	for (int pos = 0; pos < POSITION_MAX; pos += 100) {
		sendDriveToPosition(id, pos);
		cout << "\n" << pos;
		//usleep(10);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

	}
}
void PinInterface::testMoveDown(NodeID id) {
	sendDriveToPosition(id, POSITION_MAX);
	//usleep(500);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
	cout << "\nMoving down...";
	for (int pos = POSITION_MAX; pos > POSITION_MIN; pos -= TEST_TIME_STEP_MS) {
		sendDriveToPosition(id, pos);
		cout << "\n" << pos;
		//usleep(TEST_TIME_STEP_MS);
        std::this_thread::sleep_for(std::chrono::milliseconds(TEST_TIME_SETTLE_MS));
	}
}
void PinInterface::testMoveUp(NodeID id) {
	sendDriveToPosition(id, POSITION_MIN);
	//usleep(500);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
	cout << "\nMoving up...";
	for (int pos = POSITION_MIN; pos < POSITION_MAX; pos += TEST_TIME_STEP_MS) {
		sendDriveToPosition(id, pos);
		cout << "\n" << pos;
		//usleep(TEST_TIME_STEP_MS);
        std::this_thread::sleep_for(std::chrono::milliseconds(TEST_TIME_SETTLE_MS));
	}
}
void PinInterface::testMultipleDrives() {
	int SIN_MIN = 40000;
	int SIN_MAX = POSITION_MAX;
	int SIN_AMPLITUDE = (SIN_MAX - SIN_MIN);
	int SIN_OFFSET = SIN_AMPLITUDE / 2;
	float multiplier = PI * 2.0 / 11.0;

	double x = 0.0;
	double height = 0.0;
	for (;;) {
		x += 0.5;
		//cout << "\n";
		for (NodeID id = 1; id <= numPins; id++) {
			height = SIN_MIN + SIN_OFFSET + SIN_AMPLITUDE * sin((id+x)*multiplier);
			//cout << height << "\t";
			sendDriveToPosition(id, (int)height);
			//usleep(2);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

		}
		//

		/*
		for (NodeID id = 1; id <= numPins; id++) {
			sendDriveToPosition(id, POSITION_MAX);
			cout << "\nDrive " << id << "\tup";
			usleep(500);
		}
		usleep(500);
		for (NodeID id = 1; id <= numPins; id++) {
			sendDriveToPosition(id, 500);
			cout << "\nDrive " << id << "\tdown";
			usleep(500);
		}
		usleep(500);
		*/
	}
}
void PinInterface::testSendAllPinsToTop() {
	for (NodeID id = 1; id <= numPins; id++) {
		sendDriveToPosition(id, (int)POSITION_MAX);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
		//usleep(1);
	}
}

////////////////////////////////////////// STATIC HELPER FUNCTIONS //////////////////////////////////////////

// Swap endianess of 4-byte ints
inline void PinInterface::intToPositionData(positionData* bigEndian, int32_t* num) {
	copyBytes((BYTE*)num, (BYTE*)bigEndian, 4); // TODO: why does this work? Don't they need to be endian-swapped?
}

void  PinInterface::sendDriveToPosition(NodeID id, int pos) {
	BYTE positionMsgData[8];
	std::copy(std::begin(SET_POSITION_BASE_MSG), std::end(SET_POSITION_BASE_MSG), std::begin(positionMsgData));
	//copyBytes(SET_POSITION_BASE_MSG, positionMsgData, 4);
	intToPositionData((positionData*)(&(positionMsgData[4])), &pos);
	sendPDOMessage(id, positionMsgData);
	sendPDOMessage(id, ENOP_MSG);
	sendPDOMessage(id, MOVE_ABSOLUTE_MSG);
	//	sendPDOMessage(REQUEST_ACTUAL_POSITION_MSG);
}

string PinInterface::hexStringToByteArray(BYTE data[], string strHex) {
	for (unsigned short i = 0; i < strHex.length(); i += 2) {
		std::string byteString = strHex.substr(i, 2);
		char byte = (char)strtol(byteString.c_str(), NULL, 16);
		data[i / 2] = byte;
	}
	return strHex;
}

string PinInterface::PCAN_STATUS_STRING(TPCANStatus s) {
	switch (s) {
		/// No error
	case (PCAN_ERROR_OK) :
		return "PCAN_ERROR_OK";
		break;

		/// Transmit buffer in CAN controller is full
	case (PCAN_ERROR_XMTFULL) :
		return "PCAN_ERROR_XMTFULL";
		break;

		/// CAN controller was read too late
	case (PCAN_ERROR_OVERRUN) :
		return "PCAN_ERROR_OVERRUN";
		break;

		/// Bus error: an error counter reached the 'light' limit
	case (PCAN_ERROR_BUSLIGHT) :
		return "PCAN_ERROR_BUSLIGHT";
		break;

		/// Bus error: an error counter reached the 'warning' limit
	case (PCAN_ERROR_BUSHEAVY) :
		return "PCAN_ERROR_BUSWARNING/HEAVY";
		break;

		/// Bus error: the CAN controller is error passive
//	case (PCAN_ERROR_BUSPASSIVE) :
//		return "PCAN_ERROR_BUSPASSIVE";
//		break;

		/// Bus error: the CAN controller is in bus-off state
	case (PCAN_ERROR_BUSOFF) :
		return "PCAN_ERROR_BUSOFF";
		break;

		/// Receive queue is empty
	case (PCAN_ERROR_QRCVEMPTY) :
		return "PCAN_ERROR_QRCVEMPTY";
		break;

		/// Receive queue was read too late
	case (PCAN_ERROR_QOVERRUN) :
		return "PCAN_ERROR_QOVERRUN";
		break;

		/// Transmit queue is full
	case (PCAN_ERROR_QXMTFULL) :
		return "PCAN_ERROR_QXMTFULL";
		break;

		/// Test of the CAN controller hardware registers failed (no hardware found)
	case (PCAN_ERROR_REGTEST) :
		return "PCAN_ERROR_REGTEST";
		break;

		/// Driver not loaded
	case (PCAN_ERROR_NODRIVER) :
		return "PCAN_ERROR_NODRIVER";
		break;

		/// Hardware already in use by a Net
	case (PCAN_ERROR_HWINUSE) :
		return "PCAN_ERROR_HWINUSE";
		break;

		/// A Client is already connected to the Net
	case (PCAN_ERROR_NETINUSE) :
		return "PCAN_ERROR_NETINUSE";
		break;

		/// Hardware handle is invalid
	case (PCAN_ERROR_ILLHW) :
		return "PCAN_ERROR_ILLHW";
		break;

		/// Net handle is invalid
	case (PCAN_ERROR_ILLNET) :
		return "PCAN_ERROR_ILLNET";
		break;

		/// Client handle is invalid
	case (PCAN_ERROR_ILLCLIENT) :
		return "PCAN_ERROR_ILLCLIENT";
		break;

		/// Resource (FIFO, Client, timeout) cannot be created
	case (PCAN_ERROR_RESOURCE) :
		return "PCAN_ERROR_RESOURCE";
		break;

		/// Invalid parameter
	case (PCAN_ERROR_ILLPARAMTYPE) :
		return "PCAN_ERROR_ILLPARAMTYPE";
		break;

		/// Invalid parameter value
	case (PCAN_ERROR_ILLPARAMVAL) :
		return "PCAN_ERROR_ILLPARAMVAL";
		break;

		/// Unknown error
	case (PCAN_ERROR_UNKNOWN) :
		return "PCAN_ERROR_UNKNOWN";
		break;

		/// Invalid data, function, or action.
	case (PCAN_ERROR_ILLDATA) :
		return "PCAN_ERROR_ILLDATA";
		break;

		/// An operation was successfully carried out, however, irregularities were registered
//	case (PCAN_ERROR_CAUTION) :
//		return "PCAN_ERROR_CAUTION";
//		break;

		/// Channel is not initialized
	case (PCAN_ERROR_INITIALIZE) :
		return "PCAN_ERROR_INITIALIZE";
		break;

		/// Invalid operation
	case (PCAN_ERROR_ILLOPERATION) :
		return "PCAN_ERROR_ILLOPERATION";
		break;
	default:
		return "<Unknown PCAN Status>";
		break;
	}
}

// generates the COB ID SDO_RX for the given CAN node ID
inline unsigned short PinInterface::nodeID2COB(NodeID nodeID) {
	return RxSDO1 + nodeID;
}

inline void PinInterface::copyBytes(BYTE* from, BYTE* to, unsigned short size) {
	std::memcpy(to, from, size);
}

// these are common commands to send to a drive in hex form. W convert to binary form and save as 
// TODO: This is a mess. Clean it up.
#define SPOS100000							"237A6000A0860100" // Set POSition 100000, used for testing
#define ENOP								"2B4060000F000000"
#define MOVE_ABSOLUTE						"2B4060003F000000"
#define SWITCH_ON							"2B40600007000000"
#define SHUT_DOWN							"2B4060000E000000"
#define ENABLE_OPERATION					"2B4060000F000000"
#define DISSABLE_VOLTAGE					"2B4060000D000000"
#define HOMING_START						"2B4060001F000000" 
#define HOMING_METHOD_CURRENTPOS			"2F98600023000000"  // 1homing_method35
#define HOMING_METHOD_WITHOUTINDEXPULSE		"2F98600011000000"	// 1homing_method17
#define MODE_OF_OPERATION_POSITION			"2F60600001000000"  //(Profile Position 1)
#define MODE_OF_OPERATION_VELOCITY			"2F60600003000000"  //(Profile Velocity 3)
#define MODE_OF_OPERATION_HOMING			"2F60600006000000"  //(Home Mode 6)
#define REQUEST_ACTUAL_POSITION				"4064600000000000"
#define POSITION_UPLOAD_RESPONSE_BASE		"43646000"
#define REQUEST_CURRENT						"4078600000000000"
#define CURRENT_UPLOAD_RESPONSE_BASE		"4b786000"
#define SET_POSITION_BASE					"237A6000"
BYTE PinInterface::SPOS100000_MSG[8];
BYTE PinInterface::ENOP_MSG[8];
BYTE PinInterface::MOVE_ABSOLUTE_MSG[8];
BYTE PinInterface::SWITCH_ON_MSG[8];
BYTE PinInterface::SHUT_DOWN_MSG[8];
BYTE PinInterface::ENABLE_OPERATION_MSG[8];
BYTE PinInterface::DISSABLE_VOLTAGE_MSG[8];
BYTE PinInterface::HOMING_METHOD_CURRENTPOS_MSG[8];
BYTE PinInterface::HOMING_METHOD_WITHOUTINDEXPULSE_MSG[8];
BYTE PinInterface::HOMING_START_MSG[8];
BYTE PinInterface::MODE_OF_OPERATION_POSITION_MSG[8];
BYTE PinInterface::MODE_OF_OPERATION_VELOCITY_MSG[8];
BYTE PinInterface::MODE_OF_OPERATION_HOMING_MSG[8];
BYTE PinInterface::REQUEST_ACTUAL_POSITION_MSG[8];
BYTE PinInterface::REQUEST_CURRENT_MSG[8];
BYTE PinInterface::SET_POSITION_BASE_MSG[4];
bool PinInterface::initMessages() {
	hexStringToByteArray(SPOS100000_MSG, SPOS100000);
	hexStringToByteArray(ENOP_MSG, ENOP);
	hexStringToByteArray(SHUT_DOWN_MSG, SHUT_DOWN);
	hexStringToByteArray(MOVE_ABSOLUTE_MSG, MOVE_ABSOLUTE);
	hexStringToByteArray(SWITCH_ON_MSG, SWITCH_ON);
	hexStringToByteArray(ENABLE_OPERATION_MSG, ENABLE_OPERATION);
	hexStringToByteArray(DISSABLE_VOLTAGE_MSG, DISSABLE_VOLTAGE);
	hexStringToByteArray(HOMING_START_MSG, HOMING_START);
	hexStringToByteArray(HOMING_METHOD_CURRENTPOS_MSG, HOMING_METHOD_CURRENTPOS);
	hexStringToByteArray(HOMING_METHOD_WITHOUTINDEXPULSE_MSG, HOMING_METHOD_WITHOUTINDEXPULSE);
	hexStringToByteArray(MODE_OF_OPERATION_POSITION_MSG, MODE_OF_OPERATION_POSITION);
	hexStringToByteArray(MODE_OF_OPERATION_VELOCITY_MSG, MODE_OF_OPERATION_VELOCITY);
	hexStringToByteArray(MODE_OF_OPERATION_HOMING_MSG, MODE_OF_OPERATION_HOMING);
	hexStringToByteArray(REQUEST_ACTUAL_POSITION_MSG, REQUEST_ACTUAL_POSITION);
	hexStringToByteArray(REQUEST_CURRENT_MSG, REQUEST_CURRENT);	hexStringToByteArray(SET_POSITION_BASE_MSG, SET_POSITION_BASE);
	hexStringToByteArray(SET_POSITION_BASE_MSG, SET_POSITION_BASE);
	return true;
}
bool PinInterface::MESSAGES_FILLED = initMessages(); // initialize static member variables