/*
 *  Library for Pseudojoints communication with MASTER(OpenCR1.0 Board)
 *  using the RF24L01 module for Arduino
 *  Created by N.A. Stravopodis, March, 2020.
 */

#ifndef PseudoSPIcommMetamorphicManipulator_h
#define PseudoSPIcommMetamorphicManipulator_h

// Default includes for driving pseudojoint steppers
#include "Arduino.h"

// For RF PseudoCommunication
#include "SPI.h"
#include <EEPROM.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <definitions.h> 
//#include <Eventually.h>

// External Variables
extern bool return_write_attempt;
extern bool return_read_attempt;
extern bool result;
extern bool continue_exec;
extern bool return_function_state;
extern bool END_METAMORPHOSIS;					// flag to control loop for Metamorphosis <OPERATION MODE>
extern bool metaExecution;				// ...
extern bool slave_responded_correct_flag;

extern byte CURRENT_STATE[];
extern byte PSEUDO_CURRENT_POSITION;

extern float theta_p_current;
extern float theta_p_goal;

extern int  COMMAND;
extern int  slaveCommandReceived;
extern byte  currentDirStatusPseudo;
extern int  currentMoveRelPseudo;
extern int  currentAbsPosPseudo;
extern int  RELATIVE_STEPS_TO_MOVE;
extern int  theta_p_current_steps;

extern unsigned long time_now_micros;
extern unsigned long time_now_millis;

extern float step_angle;

// Constants
const char STATE_LOCKED_STRING[] 	= "LOCKED";
const char STATE_UNLOCKED_STRING[] 	= "UNLOCKED";
const char STATE_IN_POSITION_STRING[] = "IN_POSITION"; 
const char COMMAND_LOCK_STRING[] 	= "LOCK";
const char COMMAND_UNLOCK_STRING[] 	= "UNLOCK";
const char COMMAND_SGP_STRING[] 	= "SET GOAL POSITION";
const char COMMAND_HOME_STRING[] 	= "HOME";
const char MOVING[]					= "MOVING";
const char HOMING[]					= "HOMING";
const char EXIT_METAMORPHOSIS[]		= "EXIT_METAMORPHOSIS";
const char REPEAT_METAMORPHOSIS[]	= "REPEAT_METAMORPHOSIS";

const float ag  = ( 2 * PI ) / ( GEAR_FACTOR_PSEUDO1 * SPR_PSEUDO1 ); 		// Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
const float min_pseudo_angle = -PI/2;

const unsigned long wait_total_response_time_micros = 4000;  //250

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
	bool readCommandPseudoPacket(RF24 LISTENER, uint8_t radioPseudoNumber, byte *CURRENT_STATE, typeAddresses pseudoAddresses[] );

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
	PseudoSPIcommMetamorphicManipulator(enum Mode mode, int pseudoID, int statusLedPin,  int mosiPin, int misoPin, int sckPin, int txLedPin, int rxLedPin, int ssPins[]);
	
	// Master Demand and Slave Response functions for data structs commands

	void constructPacket(aliasPacketReceived *PACKET, int pseudoID, int command_code);
	
	void constructEptyPacket(aliasPacketReceived *PACKET);

	bool executeTxRxMasterBlock(aliasPacketReceived PACKET, int pseudoID , int ssPins[]);
	
    // Master Demand and Slave Response functions for single byte commands

	/*  *Master*  */

	bool connectPseudoMaster(int pseudoID, int ssPins[]);
	
	bool readCurrentStateMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE  );

	bool lockPseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE );

	bool unlockPseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE );
	
	bool setGoalPositionMaster(int pseudoID, int ssPins[], byte * GP, byte *CURRENT_STATE );

	bool movePseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE );

	bool continueMetaExecutionMaster(int pseudoID, int ssPins[], byte USER_COMMAND, bool *finishMetaMode, byte *CURRENT_STATE );

	/*  *Slave*  */

	void setupEEPROMslave(int newID, float max_angle_limit, float min_angle_limit, float pseudoStepAngle);

	bool readCurrentStateSlave( byte *CURRENT_STATE );
	
	bool saveCurrentStateSlave( byte *CURRENT_STATE );

	byte connectPseudoSlave();

	bool lockPseudoSlave(byte *CURRENT_STATE);
	
	bool unlockPseudoSlave(byte *CURRENT_STATE);

	bool setGoalPositionSlave(byte *PSEUDO_GOAL_POSITION, int *RELATIVE_STEPS_TO_MOVE, byte *CURRENT_STATE);

	bool setGoalPositionSlave2( byte *PSEUDO_GOAL_POSITION, byte * currentAbsPosPseudo_ci, int *RELATIVE_STEPS_TO_MOVE, byte * currentDirStatusPseudo, byte *CURRENT_STATE );

	bool saveEEPROMsettingsSlave(byte *CURRENT_STATE , byte * currentAbsPosPseudo_ci, byte * currentDirStatusPseudo);

	bool repeatMetaSlave(byte *CURRENT_STATE);

	void readEEPROMsettingsSlave(int pseudoID, byte *CURRENT_STATE , byte * currentAbsPosPseudo_ci,  byte *currentDirStatusPseudo, int *currentAbsPosPseudo);

	void statusLEDblink( int number_of_blinks, unsigned long blink_for_ms);

	void txrxLEDSblink( int number_of_blinks, unsigned long blink_for_ms);
	
	bool movePseudoSlave(  byte *CURRENT_STATE , int *RELATIVE_STEPS_TO_MOVE);

	bool setHomePositionSlave( int * currentAbsPosPseudo, byte *currentAbsPosPseudo_ci);

	private:

	int 			_pseudoID;
	unsigned long   _micros;

	int 			_mosiPin;
	int 			_misoPin;
	int 			_sckPin;

	int 			_txLedPin;
	int 			_rxLedPin;
	int 			_statusLedPin;

	byte singleByteTransfer(byte packet, unsigned long wait_for_response);
	
	void readCurrentPseudoPosition(float *theta_p_current, int *theta_p_current_steps);

	bool calculateRelativeStepsToMove(float *theta_p_goal, int *RELATIVE_STEPS_TO_MOVE);

};

 #endif

