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

// Addresses
typedef const byte typeAddresses[ADDRESS_WIDTH]; 

// Constructor
PseudoRFcommMetamorphicManipulator::PseudoRFcommMetamorphicManipulator(RF24 RADIO, int pseudoID, int csnPin, int cePin, int misoPin, int mosiPin)
{
	_pseudoID = pseudoID;
	_csnPin   = csnPin;
	_cePin    = cePin;
	_misoPin  = misoPin;
	_mosiPin  = mosiPin;	
}

bool PseudoRFcommMetamorphicManipulator::setTxMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	// When MASTER is Tx it uses one adress to write to each pseudo, the remaining 5 are used for reading(I can read from up to 5 pseudos at the same time)

	// writing/reading address
	const byte *writeAdress = pseudoAddresses[radioPseudoNumber-1];
	const byte *read1Adress = pseudoAddresses[radioPseudoNumber];

	OBJECT.openWritingPipe(writeAdress);    						// MASTER WRITES TO THIS PIPE TO PSEUDO1
	OBJECT.openReadingPipe(radioPseudoNumber,read1Adress);			// MASTER READS FROM THIS PIPE FROM PSEUDO1

	OBJECT.stopListening();

	enum Mode setSlaveMode = Rx;											// Slave MUST be informed that becomes Rx!

	return_write_attempt = OBJECT.write(&setSlaveMode,sizeof(setSlaveMode));
	if(return_write_attempt == false){
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to SLAVE: FAILED");
		result = false;
	}
	else{
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to SLAVE: SUCCESS");
		result = true;
	}

	return result;
}
/*
bool PseudoRFcommMetamorphicManipulator::setRxMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	// When MASTER is Rx it must WRITE to Pseudo that it is Listening 

	const byte *readAdress = pseudoAddresses[radioPseudoNumber-1];

	// reading from address
	OBJECT.openReadingPipe(radioPseudoNumber,readAdress);			// MASTER READS FROM THIS PIPE FROM PSEUDO defined by radioPseudoNumber

	OBJECT.startListening();

	bool setTxSlave = true;											// Slave becomes Tx

	return_write_attempt = OBJECT.write(&setRxSlave,sizeof(setRxSlave));
	if(return_write_attempt == false){
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode FAILED");
		result = false;
	}
	else{
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode SUCCESS");
		result = true;
	}

	return result;
}
*/

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::setRxSlave(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	// writing/reading address
	const byte *writeAdress = pseudoAddresses[radioPseudoNumber];
	const byte *read1Adress = pseudoAddresses[radioPseudoNumber-1];

	OBJECT.openWritingPipe(writeAdress);    						// PSEUDO1 WRITES TO THIS PIPE TO MASTER
	OBJECT.openReadingPipe(radioPseudoNumber,read1Adress);			// PSEUDO1 READS FROM THIS PIPE FROM MASTER

	OBJECT.startListening();

	if(OBJECT.available()){
	
	while (OBJECT.available()) {                          
	OBJECT.read( &slaveMode, sizeof(slaveMode) );       
	}     

	Serial.print("slaveMode="); Serial.println(slaveMode);
	
	if(slaveMode == Rx)
	{
	Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] MODE: Rx");
	
	result = true;
	}
	else
	{
	Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] MODE: FAILED"); 
	result = false;
	}
  }
  
  return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::writePseudoStatePacket(RF24 TALKER, uint8_t radioPseudoNumber, char *pseudoAddress, int pseudoState)
{
	struct pseudoStateDataStruct PSEUDO_STATE_STRUCT;
	
	TALKER.openWritingPipe(&pseudoAddress);

	TALKER.stopListening();

	PSEUDO_STATE_STRUCT._micros = micros();
	PSEUDO_STATE_STRUCT._pseudoID = radioPseudoNumber;
	PSEUDO_STATE_STRUCT.pseudoState  = pseudoState;
  	
	return_write_attempt = TALKER.write(&PSEUDO_STATE_STRUCT,sizeof(PSEUDO_STATE_STRUCT));

	if(return_write_attempt == false){
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE STATE FAILED");
		result = false;
	}
	else{
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE STATE SUCCESS");
		result = true;
	}

	TALKER.startListening();
  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::readPseudoStatePacket(RF24 LISTENER, uint8_t radioPseudoNumber, char *pseudoAddress)
{
	struct pseudoStateDataStruct PSEUDO_STATE_STRUCT;
	
	LISTENER.openReadingPipe(radioPseudoNumber,&pseudoAddress);
	
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
	  		Serial.println("MISMATCHING STATE CODE");
	  		break;
	  	}
  		result = true;
  	}
  	else
  	{
  		Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_STATE_STRUCT._pseudoID); Serial.print(" ]"); Serial.println("STATE READ FAILED");
  		result = false;
  	}

	LISTENER.stopListening();	
  	LISTENER.closeReadingPipe(radioPseudoNumber);

  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::writeCommandPseudoPacket(RF24 TALKER, uint8_t radioPseudoNumber, char *pseudoAddress, int command_code)
{
	struct pseudoCommandDataStruct PSEUDO_COMMAND_STRUCT;
	
	TALKER.openWritingPipe(&pseudoAddress);

	TALKER.stopListening();

	PSEUDO_COMMAND_STRUCT._micros = micros();
	PSEUDO_COMMAND_STRUCT._pseudoID = radioPseudoNumber;
	PSEUDO_COMMAND_STRUCT.pseudoCommand  = command_code;
  	
	return_write_attempt = TALKER.write(&PSEUDO_COMMAND_STRUCT,sizeof(PSEUDO_COMMAND_STRUCT));

	if(return_write_attempt == false){
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE COMMAND FAILED");
		result = false;
	}
	else{
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE COMMAND SUCCESS");
		result = true;
	}

	TALKER.startListening();

  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::readCommandPseudoPacket(RF24 LISTENER, uint8_t radioPseudoNumber, char *pseudoAddress)
{
	struct pseudoCommandDataStruct PSEUDO_COMMAND_STRUCT;
	
	LISTENER.openReadingPipe(radioPseudoNumber,&pseudoAddress);
	
	LISTENER.startListening();
	
	if( LISTENER.available(&radioPseudoNumber) ) 
	{

		while(LISTENER.available(&radioPseudoNumber)){
			LISTENER.read(&PSEUDO_COMMAND_STRUCT, sizeof(PSEUDO_COMMAND_STRUCT)); //
	    	}

	  	Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_COMMAND_STRUCT._pseudoID); Serial.println(" ]");
	  	
	  	switch(PSEUDO_COMMAND_STRUCT.pseudoCommand){
	  	case  CMD_LOCK:
	  		Serial.print("[ COMMAND: "); Serial.print(COMMAND_LOCK_STRING); Serial.println(" ]");
			// ENABLE LOCK_PIN_RELAY

			// SET FLAG TO WRITE NEW STATE

	  		break;
	  	case CMD_UNLOCK:
	  		Serial.print("[ COMMAND: "); Serial.print(COMMAND_UNLOCK_STRING); Serial.println(" ]");
			// DISABLE LOCK_PIN_RELAY

			// SET FLAG TO WRITE NEW STATE

	  		break;
	  	case  CMD_SGP:
	  		Serial.print("[ COMMAND: "); Serial.print(COMMAND_SGP_STRING); Serial.println(" ]");
			// DRIVES MOTOR TO GOAL POSITION

			// SET FLAG TO WRITE NEW COMMAND: LOCK

	  		break;
	  	case CMD_HOME:
	  		Serial.print("[ COMMAND: "); Serial.print(COMMAND_HOME_STRING); Serial.println(" ]");
			// DRIVES MOTOR TO ZERO POSITION

			// SET FLAG TO WRITE NEW COMMAND: LOCK

	  		break;			  
	  	default:
	  		Serial.println("MISMATCHING COMMAND CODE");
	  		break;
	  	}
  		result = true;
  	}
  	else
  	{
  		Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_COMMAND_STRUCT._pseudoID); Serial.print(" ]"); Serial.println(" COMMAND READ FAILED");
  		result = false;
  	}	
  	
	LISTENER.stopListening();
	LISTENER.closeReadingPipe(radioPseudoNumber);

  	return result;
}
