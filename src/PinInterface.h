#pragma once
#ifndef PIN_CONTROL_H
#define PIN_CONTROL_H

#include "ofMain.h"
#include "PCBUSB.h"
#include "Parameters.h"

//#define PRINT_CAN_MESSAGES

typedef unsigned short NodeID;

//NOTE: the drive controller CAN Node IDs are assumed to to be in the range [1...NUM_DRIVES]  - ensure for loops iterating through drives start at 1 and end on NUM_DRIVES inclusive.
#define MAX_NODES 128 // There can be a maximum of 128 nodes on the CAN Open network.
// NOTE: Node IDs are 1-based, not 0-based, so the first node is 1 and the last possible node is 127
// The difference between the max and min COB IDs for a function code will therefor be 127.
// For example, TxPDO3 ranges from 0x381 (897) to 0x400 (1024) (127 is the difference, so there are 127 nodes in this inclusive range)
// Note that there is 127 values between the following CAN ID Types.
#define TxPDO1 0x180 // Receive drive data (e.g. status values)
#define RxPDO1 0x200 // Send data to the drives (e.g. control commands)
#define TxPDO2 0x280 // Receive drive data (e.g. status values)
#define RxPDO2 0x300 // Send data to the drives (e.g. control commands)
#define TxPDO3 0x380 // Receive drive data (e.g. status values)
#define RxPDO3 0x400 // Send data to the drives (e.g. control commands)
#define TxPDO4 0x480 // Receive drive data (e.g. status values)
#define RxPDO4 0x500 // Send data to the drives (e.g. control commands)
#define TxSDO1 0x580 // Read entry of the object dictionary
#define RxSDO1 0x600 // Write entry of the object dictionary

// The 4-bit IDs that start the 11-bit CAN ID for SDO messages. The remaining 7 bits are the Node ID
#define SDO_OD_ENTRY_SEND 1536 // 0x600 We send these messages to the drive ("Client -> Server") to read ("upload request") or write ("download request") OD entries
#define SDO_OD_ENTRY_RECEIVE 1407 //0x580 The drive sends these messages to us ("Server -> Client") to reply with the values of requested OD entries ("upload response") or confirm it received a write entry ("download response")

//The first byte of a CANOpen message
#define  SDO_OD_ENTRY_FIRSTBYTE_SEND_READ 64 // 0x40 Indicates we want to read an OD entry.
#define  SDO_OD_ENTRY_FIRSTBYTE_SEND_WRITE 32 // 0x2X indicates we want to write an OD entry.The last 4 bits determine how long the data is.
#define  SDO_OD_ENTRY_FIRSTBYTE_RECEIVED_READ 96 // 0x60
#define  SDO_OD_ENTRY_FIRSTBYTE_RECEIVED_WRITE 64 // 0x4X

// Constants used for testing
#define TEST_POSITION_STEP 100
#define TEST_TIME_SETTLE_MS 100
#define TEST_TIME_STEP_MS 10

// Convert between 4-element byte arrays and 4-byte ints
typedef union {
	BYTE byteArray[4];
	int32_t num;
} positionData;

enum FUNCTION_CODES {
    EMERGENCY = 129,//0x81
    PDO1tx = 385,	//0x181
    PDO1rx = 513,		//0x201
    PDO2tx641,		//0x281
    PDO2rx = 769,		//0x301
    PDO3tx = 897,		//0x381
    PDO3rx = 1025,	//0x401
    PDO4tx = 1153,	//0x481
    PDO4rx = 1281,	//0x501
    SDOtx = 1409,		//0x581
    SDOrx = 1537,		//0x601
    NMTErrorControl = 1793//0x701
};

class PinInterface {
public:
	PinInterface(NodeID numPins, TPCANBaudrate m_Baudrate = PCAN_BAUD_1M, TPCANType m_HwType = PCAN_ISA, unsigned int ioPort = 100, unsigned short interrupt = 3, TPCANHandle m_PcanHandle = PCAN_USBBUS1);
	~PinInterface();
	void sendCANMessage(DWORD id, BYTE msgData[8]);
	void sendPDOMessage(NodeID nodeID, BYTE messageData[8]);
	void initializeDrive(NodeID id);
	void initializeAllDrives();
	void shutdownDrive(NodeID id);
	void shutdownAllDrives();
	void homeDrive(NodeID id);
	void homeAllDrives();
	void homeDriveAtCurrentPosition(NodeID id);
	void homeAllDrivesAtCurrentPosition();
	void sendDriveToPosition(NodeID id, int pos);
	void readCANMessages();
	//void getPinPositions(int positions[]);
	void tearDownCAN();
	static string PCAN_STATUS_STRING(TPCANStatus s);
	TPCANStatus getPCANStatus();
	//void blockUntilCANBusClear();

	// Test Functions
	void testPositionSpeed(NodeID id);
	void testPositionPrecision(NodeID id);
	void testMoveDown(NodeID id);
	void testMoveUp(NodeID id);
	void testMultipleDrives();
	void testSendAllPinsToTop();
private:
	TPCANHandle m_PcanHandle;
	unsigned short numPins;
	//int* pinPositions;

	static string hexStringToByteArray(BYTE data[], string strHex);
	static inline unsigned short nodeID2COB(NodeID nodeID);
	static inline void copyBytes(BYTE* from, BYTE* to, unsigned short size);
	static inline void intToPositionData(positionData* bigEndian, int32_t* num);

	// These are binary forms of common messages we send to pins. they are initialized by a helper function initMessages() called to initialize the static variable MESSAGES_FILLED
	static BYTE SPOS100000_MSG[8];
	static BYTE ENOP_MSG[8];
	static BYTE MOVE_ABSOLUTE_MSG[8];
	static BYTE SWITCH_ON_MSG[8];
	static BYTE SHUT_DOWN_MSG[8];
	static BYTE ENABLE_OPERATION_MSG[8];
	static BYTE DISSABLE_VOLTAGE_MSG[8];
	static BYTE HOMING_METHOD_CURRENTPOS_MSG[8];
	static BYTE HOMING_METHOD_WITHOUTINDEXPULSE_MSG[8];
	static BYTE HOMING_START_MSG[8];
	static BYTE MODE_OF_OPERATION_POSITION_MSG[8];
	static BYTE MODE_OF_OPERATION_VELOCITY_MSG[8];
	static BYTE MODE_OF_OPERATION_HOMING_MSG[8];
	static BYTE REQUEST_ACTUAL_POSITION_MSG[8];
	static BYTE REQUEST_CURRENT_MSG[8];
	static BYTE SET_POSITION_BASE_MSG[4];
	static bool MESSAGES_FILLED;
	static bool initMessages();
};

#endif