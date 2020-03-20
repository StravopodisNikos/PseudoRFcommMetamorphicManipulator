/*
 *  Library for Pseudojoints communication with MASTER(OpenCR1.0 Board)
 *  using the RF24L01 module for Arduino
 *  Created by N.A. Stravopodis, March, 2020.
 */

#ifndef PseudoSPIcommMetamorphicManipulator_h
#define PseudoSPIcommMetamorphicManipulator_h

#define SERIAL_BAUDRATE		115200

// PINS CONFIGURATION MASTER
const byte interruptPin 	= 2;

// PINS CONFIGURATION SLAVE
#define MOSI_NANO 			11
#define MISO_NANO 			12
#define SCK_NANO 			13
#define TXled_Pin 			7
#define RXled_Pin 			6
#define statusLED_Pin		2		// indicates master-pseudo com status
#define dirPin_NANO			A5
#define stepPin_NANO		A4
#define enabPin_NANO		A3
#define hallSwitch_Pin		A1
#define RELAY_lock_Pin		A0

#define SSpinPseudo1		3
#define SSpinPseudo2		4
#define SSpinPseudo3		5

#define PSEUDO1_ID 			1
#define PSEUDO2_ID 			2
#define PSEUDO3_ID 			3
#define PSEUDO4_ID 			4
#define PSEUDO5_ID 			5
#define PSEUDO6_ID 			6

#define ADDRESS_WIDTH		6

// COMMANDS SENT -> in single byte transfer each ci is regarded as command!
// ci's: these are the bytes given to setGoalPositionMaster() for desired anatomy Metamorphosis

#define c1					1
#define c2					2
#define c3					3
#define c4					4
#define c5					5	
#define c6					6
#define c7					7
#define c8					8
#define c9					9	
#define c10					10
#define c11					11
#define c12					12
#define c13					13

#define CMD_LOCK		    20
#define CMD_UNLOCK	    	21
#define CMD_SGP	  			30		// In single byte transfer is overrided
#define CMD_MOVE			40
#define CMD_STOP			41		// Danger Stop Event!
#define CMD_HOME			42	
#define CMD_CONNECT		    60
#define CMD_GIVE_IS		    70
#define CMD_GIVE_CS		    71
#define CMD_EXIT_META_EXEC  80
#define CMD_CONT_META_EXEC  81
#define CMD_GIVE_EEPROM	    90		// only at setup

// STATES RETURNED
#define STATE_LOCKED 	  	100
#define STATE_UNLOCKED 		110
#define IN_POSITION 		111
#define IS_MOVING 			112
#define TALKED_DONE			113
#define IS_TALKING 			114
#define STATE_READY			115
#define GP_SET				116
#define META_FINISHED		117
#define META_REPEAT			118

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
#define spr					800	// Depends on driver dip switches

// EEPROM AREA ADDRESSES [0~255]
#define ID_EEPROM_ADDR		0		// int
#define MAX_POS_LIM_ADDR	10		// float
#define MIN_POS_LIM_ADDR	20		// float
#define STEP_ANGLE_ADDR		30		// float
#define CS_EEPROM_ADDR		40		// byte			// Always updated with homing
#define CP_EEPROM_ADDR		50		// byte			// ...
#define CD_EEPROM_ADDR		60		// uint32_t		// ...

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

const float ag  = ( 2 * PI ) / ( GEAR_FACTOR * spr ); 		// Geared Motor Step Angle(Angular Position of Output shaft of Gearbox )[rad]
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
	
	bool setGoalPositionMaster(int pseudoID, int ssPins[], byte GP, byte *CURRENT_STATE );

	bool movePseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE );

	bool continueMetaExecutionMaster(int pseudoID, int ssPins[], byte USER_COMMAND, bool *finishMetaMode, byte *CURRENT_STATE );

	/*  *Slave*  */

	void setupEEPROMslave(int newID, float max_angle_limit, float min_angle_limit, float pseudoStepAngle);

	byte readInitialStateSlave();
	
	byte connectPseudoSlave();

	bool lockPseudoSlave(byte *CURRENT_STATE);
	
	bool unlockPseudoSlave(byte *CURRENT_STATE);

	bool setGoalPositionSlave(byte *PSEUDO_GOAL_POSITION, int *RELATIVE_STEPS_TO_MOVE, byte *CURRENT_STATE);

	bool saveEEPROMsettingsSlave(byte *CURRENT_STATE , byte currentDirStatusPseudo, int currentAbsPosPseudo);

	void readEEPROMsettingsSlave(byte *CURRENT_STATE , byte *currentDirStatusPseudo, int *currentAbsPosPseudo);

	void statusLEDblink( int number_of_blinks, unsigned long blink_for_ms);

	void txrxLEDSblink( int number_of_blinks, unsigned long blink_for_ms);
	
	bool movePseudoSlave(  byte *CURRENT_STATE , int *RELATIVE_STEPS_TO_MOVE);

	bool setHomePositionSlave( int * currentAbsPosPseudo);

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

