/*
 *  Library for Pseudojoints communication with MASTER(OpenCR1.0 Board)
 *  using the RF24L01 module for Arduino
 *  Created by N.A. Stravopodis, March, 2020.
 */

#ifndef PseudoRFcommMetamorphicManipulator_h
#define PseudoRFcommMetamorphicManipulator_h

#define PSEUDO_NUMBER1 	1
#define STATE_LOCKED 	  1
#define STATE_UNLOCKED 	11
#define CMD_LOCK		    2
#define CMD_UNLOCK	    22
#define CMD_MOVE	  3	// Calls pseudojoint to move to goal position
#define CMD_HOME		    2

// Default includes for driving pseudojoint steppers
#include "Arduino.h"

// For RF PseudoCommunication
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>

// External Variables
extern bool return_write_attempt;
extern bool return_read_attempt;
extern bool result;

const char STATE_LOCKED_STRING[] = "LOCKED";
const char STATE_UNLOCKED_STRING[] = "UNLOCKED";
const char MOVING[]= "MOVING";

#define STATE_LOCKED 	  1

class PseudoRFcommMetamorphicManipulator
{
	public:
	
	struct pseudoStateDataStruct{
	  unsigned long   _micros;
	  int             _pseudoID;
	  int             pseudoState;      
	};
	
	struct pseudoCommandDataStruct{
	  unsigned long   _micros;
	  int             _pseudoID;
	  int             pseudoCommand;      
	};

	struct pseudoGoalDataStruct{
	  unsigned long   _micros;
	  int             _pseudoID;
	  float           pseudoAngle;      
	};
	
	// Constructor
	PseudoRFcommMetamorphicManipulator(RF24 RADIO, int pseudoID, int csnPin, int cePin, int misoPin, int mosiPin);
	
	// Sends data
	bool writePseudoStatePacket(RF24 TALKER, uint8_t radioPseudoNumber, char *pseudoAddress, int pseudoState);
	
	// Gets data
	bool readPseudoStatePacket(RF24 LISTENER, uint8_t radioPseudoNumber, char *pseudoAddresses);
	
	// send command
	bool writeCommandPseudoPacket(RF24 TALKER, uint8_t radioPseudoNumber, char *pseudoAddresses, int command_code);
	
	// read command
	bool readCommandPseudoPacket(RF24 LISTENER, uint8_t radioPseudoNumber, char *pseudoAddresses);

	private:
	int _pseudoID;
	int _csnPin;
	int _cePin;
	int _misoPin;
	int _mosiPin;
	
};
  
 #endif

