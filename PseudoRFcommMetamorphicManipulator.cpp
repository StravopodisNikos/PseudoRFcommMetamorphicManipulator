/*
 *  Source Code for library PseudoRFcommMetamorphicManipulator.h
 *  using the RF24L01 module for Arduino
 *  Created by N.A. Stravopodis, March, 2020.
 */

// Default includes for driving pseudojoint steppers
#include "Arduino.h"
#include <printf.h>
#include "PseudoRFcommMetamorphicManipulator.h"

// For RF PseudoCommunication
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>

// Constructor
PseudoRFcommMetamorphicManipulator::PseudoRFcommMetamorphicManipulator(RF24 RADIO, int pseudoID, int csnPin, int cePin, int misoPin, int mosiPin)
{
	
	_pseudoID = pseudoID;
	_csnPin   = csnPin;
	_cePin    = cePin;
	_misoPin  = misoPin;
	_mosiPin  = mosiPin;	
}

bool PseudoRFcommMetamorphicManipulator::writePseudoStatePacket(RF24 TALKER, uint8_t radioPseudoNumber, char *pseudoAddresses, int pseudoState)
{
	struct pseudoStateDataStruct PSEUDO_STATE_STRUCT;
	
	TALKER.openWritingPipe(&pseudoAddresses);

	TALKER.stopListening();

	PSEUDO_STATE_STRUCT._micros = micros();
	PSEUDO_STATE_STRUCT._pseudoID = radioPseudoNumber;
	PSEUDO_STATE_STRUCT.pseudoState  = pseudoState;
  	
	return_write_attempt = TALKER.write(&PSEUDO_STATE_STRUCT,sizeof(PSEUDO_STATE_STRUCT));

	if(return_write_attempt == false){
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] FAILED");
		result = false;
	}
	else{
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] SUCCESS");
		result = true;
	}

	TALKER.startListening();
  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::readPseudoStatePacket(RF24 LISTENER, uint8_t radioPseudoNumber, char *pseudoAddresses)
{
	struct pseudoStateDataStruct PSEUDO_STATE_STRUCT;
	
	LISTENER.openReadingPipe(radioPseudoNumber,&pseudoAddresses);
	
	LISTENER.startListening();
	
	if( LISTENER.available(&radioPseudoNumber) ) 
	{

		while(LISTENER.available(&radioPseudoNumber)){
			LISTENER.read(&PSEUDO_STATE_STRUCT, sizeof(PSEUDO_STATE_STRUCT)); //
	    	}

	  	Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_STATE_STRUCT._pseudoID); Serial.println(" ]");
	  	
	  	switch(PSEUDO_STATE_STRUCT.pseudoState){
	  	case  STATE_LOCKED:
	  		Serial.print("[ STATE: "); Serial.print(STATE_LOCKED_STRING); Serial.println(" ]");
	  		break;
	  	case STATE_UNLOCKED:
	  		Serial.print("[ STATE: "); Serial.print(STATE_UNLOCKED_STRING); Serial.println(" ]");
	  		break;
	  	default:
	  		Serial.println("MISMATCHING STATES");
	  		break;
	  	}
  		result = true;
  	}
  	else
  	{
  		Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_STATE_STRUCT._pseudoID); Serial.print(" ]"); Serial.println(" READ FAILED");
  		result = false;
  	}	
  	
  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::writeCommandPseudoPacket(RF24 TALKER, uint8_t radioPseudoNumber, char *pseudoAddresses, int command_code)
{
	struct pseudoCommandDataStruct PSEUDO_COMMAND_STRUCT;
	
	TALKER.openWritingPipe(&pseudoAddresses);

	TALKER.stopListening();

	PSEUDO_COMMAND_STRUCT._micros = micros();
	PSEUDO_COMMAND_STRUCT._pseudoID = radioPseudoNumber;
	PSEUDO_COMMAND_STRUCT.pseudoCommand  = command_code;
  	
	return_write_attempt = TALKER.write(&PSEUDO_COMMAND_STRUCT,sizeof(PSEUDO_COMMAND_STRUCT));

	if(return_write_attempt == false){
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] FAILED");
		result = false;
	}
	else{
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] SUCCESS");
		result = true;
	}

	TALKER.startListening();
  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::readCommandPseudoPacket(RF24 LISTENER, uint8_t radioPseudoNumber, char *pseudoAddresses)
{

}
