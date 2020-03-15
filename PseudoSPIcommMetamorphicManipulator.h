/*
 *  Library for Pseudojoints communication with MASTER(OpenCR1.0 Board)
 *  using the RF24L01 module for Arduino
 *  Created by N.A. Stravopodis, March, 2020.
 */

#ifndef PseudoSPIcommMetamorphicManipulator_h
#define PseudoSPIcommMetamorphicManipulator_h

#define SERIAL_BAUDRATE		115200

// PINS CONFIGURATION
#define MOSI_NANO 			11
#define MISO_NANO 			12
#define SCK_NANO 			13
#define TXled_Pin 			7
#define RXled_Pin 			6
#define RELAY_lock_Pin		A0
#define dirPin_NANO			A5

#define SSpinPseudo1		3
#define SSpinPseudo2		4
#define SSpinPseudo3		5

#define PSEUDO1_MASTER 		0
#define PSEUDO2_MASTER 		1
#define PSEUDO3_MASTER 		2
#define PSEUDO4_MASTER 		3
#define PSEUDO5_MASTER 		4
#define PSEUDO6_MASTER 		5

#define ADDRESS_WIDTH		6

#define STATE_LOCKED 	  	1
#define STATE_UNLOCKED 		11
#define IN_POSITION 		111
#define IS_MOVING 			112
#define STATE_READY			113
#define GP_SET				114

#define CMD_LOCK		    2
#define CMD_UNLOCK	    	22
#define CMD_SGP	  			3		// Calls pseudojoint to set goal position
#define CMD_MOVE			4
#define CMD_HOME			5	
#define CMD_CONNECT		    6
#define CMD_GIVE_CS		    7

#define PSEUDO_NUMBER1 		1
#define PSEUDO_NUMBER2		2
#define PSEUDO_NUMBER3		3
#define PSEUDO_NUMBER4		4
#define PSEUDO_NUMBER5		5
#define PSEUDO_NUMBER6		6

#define MASTER_DELAY_TIME	100
#define SLAVE_DELAY_TIME	5
#define SLAVE_RESPONSE_TIME	20

#define GEAR_FACTOR			60
#define spr					800		// Depends on driver dip switches

// EEPROM AREA ADDRESSES [0~255]
#define ID_EEPROM_ADDR		0		// byte
#define MAX_POS_LIM_ADDR	20		// float
#define MIN_POS_LIM_ADDR	30		// float
#define STEP_ANGLE_ADDR		40		// float
#define CS_EEPROM_ADDR		50		// byte
#define CP_EEPROM_ADDR		60		// byte

// Default includes for driving pseudojoint steppers
#include "Arduino.h"

// For RF PseudoCommunication
#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>
//#include <Eventually.h>

// External Variables
extern bool return_write_attempt;
extern bool return_read_attempt;
extern bool result;
extern bool continue_exec;
extern bool return_function_state;

extern byte CURRENT_STATE;
extern byte PSEUDO_CURRENT_POSITION;

extern double theta_p_current;
extern double theta_p_goal;

extern int  COMMAND;
extern int  slaveCommandReceived;
extern int  currentDirStatusPseudo;
extern int  currentMoveRelPseudo;
extern int  currentAbsPosPseudo;
extern int  RELATIVE_STEPS_TO_MOVE;
extern int  theta_p_current_steps;

extern unsigned long time_now_micros;

// Constants
const char STATE_LOCKED_STRING[] 	= "LOCKED";
const char STATE_UNLOCKED_STRING[] 	= "UNLOCKED";
const char STATE_IN_POSITION_STRING[] = "IN_POSITION"; 
const char COMMAND_LOCK_STRING[] 	= "LOCK";
const char COMMAND_UNLOCK_STRING[] 	= "UNLOCK";
const char COMMAND_SGP_STRING[] 	= "SET GOAL POSITION";
const char COMMAND_HOME_STRING[] 	= "HOME";
const char MOVING[]					= "MOVING";

const float ag  = ( 2 * PI ) / ( GEAR_FACTOR * spr ); 		// Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]

// New type definitions
enum Mode{Tx, Rx}; 

extern enum Mode slaveMode;
extern enum Mode masterMode;

typedef const byte typeAddresses[ADDRESS_WIDTH]; 
typeAddresses pseudoAddresses[] = {"1PMAd", "2PMAd"};

typedef struct aliasPacketReceived{
    unsigned long   micros_talked;
    int             pseudoID_talked;
    int             command_state; 
}packets;

typedef union aliasPacketReceivedUnion{
  packets       packet_received;
  unsigned char bytes[sizeof(packets)];
}packets_u;

class PseudoRFcommMetamorphicManipulator
{
	public:
	
	struct pseudoStateDataStruct{
	  //unsigned long   _micros;
	  int             _pseudoID;
	  int             pseudoState;      
	};
	
	struct pseudoCommandDataStruct{
	  //unsigned long   _micros;
	  int             _pseudoID;
	  int             pseudoCommand;      
	};

	struct pseudoGoalDataStruct{
	  //unsigned long   _micros;
	  int             _pseudoID;
	  float           pseudoAngle;      
	};
	
	// Constructor
	PseudoRFcommMetamorphicManipulator(RF24 RADIO, int pseudoID, int csnPin, int cePin, int misoPin, int mosiPin, int txLedPin, int rxLedPin);
	
	// sets MASTER to Tx mode and SLAVE to Rx mode
	bool setTxMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] );
	bool setRxSlave(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] );

	// sets MASTER to Rx mode and SLAVE to Tx mode
	bool setRxMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] );
	bool setTxSlave(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] );

	// Sends data
	bool writePseudoStatePacket(RF24 TALKER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] , int pseudoState);
	
	// Gets data
	bool readPseudoStatePacket(RF24 LISTENER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] );
	
	// send command
	bool writeCommandPseudoPacket(RF24 TALKER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] , int command_code);
	
	// read command
	bool readCommandPseudoPacket(RF24 LISTENER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] );

	bool execTxRxBlockMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[], int command_code, int * state_code );


	private:
	int _pseudoID;
	int _csnPin;
	int _cePin;
	int _misoPin;
	int _mosiPin;
	int _txLedPin;
	int _rxLedPin;
	
};

class PseudoSPIcommMetamorphicManipulator{

	public:

	// Structs for Master messaging
	struct masterBlockStruct1{
	  unsigned long   _micros;
	  int             _pseudoID;
	  int             command_sent;      
	};

	struct masterBlockStruct2{
	  unsigned long   _micros;
	  int             _pseudoID;
	  int             state_received;      
	};

	// Structs for Slaves messaging
	struct slaveBlockStruct1{
	  unsigned long   _micros;
	  int             _pseudoID;
	  int             command_received;      
	};

	struct slaveBlockStruct2{
	  unsigned long   _micros;
	  int             _pseudoID;
	  int             state_sent;      
	};

	// Constructor
	PseudoSPIcommMetamorphicManipulator(enum Mode mode, int pseudoID, int mosiPin, int misoPin, int sckPin, int txLedPin, int rxLedPin, int ssPins[]);
	
	// Master Demand and Slave Response functions for data structs commands

	void constructPacket(aliasPacketReceived *PACKET, int pseudoID, int command_code);
	
	void constructEptyPacket(aliasPacketReceived *PACKET);

	bool executeTxRxMasterBlock(aliasPacketReceived PACKET, int pseudoID , int ssPins[]);
	
    // Master Demand and Slave Response functions for single byte commands

	/*  *Master*  */

	bool connectPseudoMaster(int pseudoID, int ssPins[]);
	
	bool readInitialStateMaster(int pseudoID, int ssPins[] );

	bool lockPseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE );

	bool unlockPseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE );
	
	bool setGoalPositionMaster(int pseudoID, int ssPins[], byte GP, byte *CURRENT_STATE );

	/*  *Slave*  */

	byte readInitialStateSlave();
	
	byte connectPseudoSlave();

	bool lockPseudoSlave(byte *CURRENT_STATE);
	
	bool unlockPseudoSlave(byte *CURRENT_STATE);

	bool setGoalPositionSlave(byte *PSEUDO_GOAL_POSITION, int *RELATIVE_STEPS_TO_MOVE, byte *CURRENT_STATE);

	private:

	int 			_pseudoID;
	unsigned long   _micros;

	int 			_mosiPin;
	int 			_misoPin;
	int 			_sckPin;

	int 			_txLedPin;
	int 			_rxLedPin;

	byte singleByteTransfer(byte packet, unsigned long wait_for_response);

	void readCurrentPseudoPosition(double *theta_p_current, int *theta_p_current_steps);

	bool calculateRelativeStepsToMove(double *theta_p_goal, int *RELATIVE_STEPS_TO_MOVE);
};

 #endif

