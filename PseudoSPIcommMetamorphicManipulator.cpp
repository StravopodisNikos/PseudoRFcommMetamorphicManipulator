/*
 *  Source Code for library PseudoRFcommMetamorphicManipulator.h
 *  using the RF24L01 module for Arduino
 *  Created by N.A. Stravopodis, March, 2020.
 */

// Default includes for driving pseudojoint steppers
#include "Arduino.h"
#include <printf.h>
#include "PseudoSPIcommMetamorphicManipulator.h"
//#include <Eventually.h>

// For RF PseudoCommunication
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>

// Addresses
typedef const byte typeAddresses[ADDRESS_WIDTH]; 

// Constructor
PseudoRFcommMetamorphicManipulator::PseudoRFcommMetamorphicManipulator(RF24 RADIO, int pseudoID, int csnPin, int cePin, int misoPin, int mosiPin, int txLedPin, int rxLedPin)
{
	pinMode(txLedPin,OUTPUT);
	pinMode(rxLedPin,OUTPUT);

	digitalWrite(txLedPin,LOW);
	digitalWrite(rxLedPin,LOW);

	_pseudoID = pseudoID;
	_csnPin   = csnPin;
	_cePin    = cePin;
	_misoPin  = misoPin;
	_mosiPin  = mosiPin;
	_txLedPin = txLedPin;
	_rxLedPin = rxLedPin;	
}

bool PseudoRFcommMetamorphicManipulator::setTxMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	// When MASTER is Tx it uses one adress to write to each pseudo, the remaining 5 are used for reading(I can read from up to 5 pseudos at the same time)
	OBJECT.stopListening();											// calls before openWritingPipe when needs to change pipe address

	const byte *writeAdress = pseudoAddresses[radioPseudoNumber-1]; // set the pipe address depending on which pseudo we write
	//const byte *read1Adress = pseudoAddresses[radioPseudoNumber]; // will be removed

	OBJECT.openWritingPipe(writeAdress);    						// MASTER WRITES TO THIS PIPE TO PSEUDO1
	//OBJECT.openReadingPipe(radioPseudoNumber,read1Adress);		// will be removed

	OBJECT.stopListening();											// stops listening in order to write

	enum Mode setSlaveMode = Rx;									// Slave MUST be informed that becomes Rx!

	do{
		return_write_attempt = OBJECT.write(&setSlaveMode,sizeof(setSlaveMode));
		if(return_write_attempt == false){
			Serial.print("[   MASTER  ]  "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to SLAVE: FAILED");
			result = false;
		}
		else{
			Serial.print("[   MASTER  ]  "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to SLAVE: SUCCESS");
			result = true;
			digitalWrite(_txLedPin,HIGH);
			digitalWrite(_rxLedPin,LOW);
		}
	}while(!result);

	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::setRxMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	// 1. When MASTER is Rx it must WRITE to Pseudo first the mode change 
	const byte *writeAdress = pseudoAddresses[radioPseudoNumber-1];
	
	OBJECT.stopListening();
	
	OBJECT.openWritingPipe(writeAdress);    						// MASTER WRITES TO THIS PIPE TO PSEUDO1
	
	OBJECT.stopListening();

	enum Mode setSlaveMode = Tx;									// Slave MUST be informed that becomes Tx!

	do
	{
		return_write_attempt = OBJECT.write(&setSlaveMode,sizeof(setSlaveMode));
		if(return_write_attempt == false)
		{
			Serial.print("[   MASTER  ]  "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Tx Mode to SLAVE: FAILED");
			continue_exec = false;
		}
		else
		{
			Serial.print("[   MASTER  ]  "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Tx Mode to SLAVE: SUCCESS");
			continue_exec = true;
		}

	} while (!continue_exec);

	if(continue_exec)
	{
		// 2. Now, waits for slave(Tx) response
		const byte *read1Adress = pseudoAddresses[radioPseudoNumber-1];	// Uses the same address for reading!
		// reading from address
		OBJECT.openReadingPipe(radioPseudoNumber,read1Adress);			// MASTER READS FROM THE PIPE USING THE PSEUDO: FIXED ADRESS

		OBJECT.startListening();

		do{
			if(OBJECT.available()){
			
				while (OBJECT.available()) {                          
					OBJECT.read( &masterMode, sizeof(masterMode) );       
				}     
				
				if(masterMode == Rx)
				{
					//Serial.println("[ MASTER ] WRITE Rx Mode to MASTER: SUCCESS");
					result = true;
						digitalWrite(_txLedPin,LOW);
						digitalWrite(_rxLedPin,HIGH);
				}
				else
				{
					//Serial.println("[ MASTER ] WRITE Rx Mode to MASTER: FAILED"); 
					result = false;
				}
			}
		}while(!result);
	}
	else
	{
		result = false;
	}

	//OBJECT.stopListening();	

  	//OBJECT.closeReadingPipe(radioPseudoNumber);

	// 3. If it gets a response returns TRUE
	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::setTxSlave(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	// 1.  When SLAVE is Tx it must first READ from MASTER the mode change
	const byte *read1Adress = pseudoAddresses[radioPseudoNumber-1];

	OBJECT.openReadingPipe(radioPseudoNumber,read1Adress);			// PSEUDO1 READS FROM THIS PIPE FROM MASTER

	OBJECT.startListening();

	do
	{
		if(OBJECT.available()){

			while (OBJECT.available()) {                          
			OBJECT.read( &slaveMode, sizeof(slaveMode) );       
			}     
			
			if(slaveMode == Tx)
			{
				//Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] READ Tx MODE from MASTER: SUCCESS");
				continue_exec = true;
			}
			else
			{
				//Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] READ Tx MODE from MASTER: FAILED"); 
				continue_exec = false;
			}

		}
	} while (!continue_exec);

	OBJECT.closeReadingPipe(radioPseudoNumber);

	if(continue_exec)
	{
		// 2. Now, reply to master(Rx)
		const byte *writeAdress = pseudoAddresses[radioPseudoNumber-1];
		
		OBJECT.stopListening();
		
		OBJECT.openWritingPipe(writeAdress);    						// MASTER WRITES TO THIS PIPE TO PSEUDO1
		
		OBJECT.stopListening();

		enum Mode setMasterMode = Rx;									// Master MUST be informed that becomes Rx!

		do{
			return_write_attempt = OBJECT.write(&setMasterMode,sizeof(setMasterMode));
			if(return_write_attempt == false){
				Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to MASTER FAILED");
				result = false;
			}
			else{
				Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to MASTER SUCCESS");
				result = true;
				digitalWrite(_txLedPin,HIGH);
				digitalWrite(_rxLedPin,LOW);
			}
		}while(!result);
	}
	else
	{
		result = false;
	}
  	
  	// 3. If it gets a response returns TRUE
  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::setRxSlave(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	//const byte *writeAdress = pseudoAddresses[radioPseudoNumber];
	const byte *read1Adress = pseudoAddresses[radioPseudoNumber-1];

	//OBJECT.openWritingPipe(writeAdress);    						// PSEUDO1 WRITES TO THIS PIPE TO MASTER
	OBJECT.openReadingPipe(radioPseudoNumber,read1Adress);			// PSEUDO1 READS FROM THIS PIPE FROM MASTER

	OBJECT.startListening();

	do
	{
		if(OBJECT.available()){
	
				while (OBJECT.available()) {                          
				OBJECT.read( &slaveMode, sizeof(slaveMode) );       
				}     

				//Serial.print("slaveMode="); Serial.println(slaveMode);
				
				if(slaveMode == Rx)
				{
					Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] READ Rx MODE from MASTER: SUCCESS");
					continue_exec = true;
					digitalWrite(_txLedPin,LOW);
					digitalWrite(_rxLedPin,HIGH);
				}
				else
				{
					Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] READ Rx MODE from MASTER: FAILED"); 
					continue_exec = false;
					digitalWrite(_txLedPin,HIGH);
					digitalWrite(_rxLedPin,HIGH);
				}
		}
	} while (!continue_exec);

  	OBJECT.stopListening();
	OBJECT.closeReadingPipe(radioPseudoNumber);
	result = true;

	if(continue_exec){
		return result;
	}
	else
	{
		return result;
	}
	
  	
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::writePseudoStatePacket(RF24 TALKER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] , int pseudoState)
{
	struct pseudoStateDataStruct PSEUDO_STATE_STRUCT;
	
	const byte *writeAdress = pseudoAddresses[radioPseudoNumber-1];
	TALKER.stopListening();

	TALKER.openWritingPipe(writeAdress);

	TALKER.stopListening();

	//PSEUDO_STATE_STRUCT._micros = micros();
	PSEUDO_STATE_STRUCT._pseudoID = radioPseudoNumber;
	PSEUDO_STATE_STRUCT.pseudoState  = pseudoState;
  	
	return_write_attempt = TALKER.write(&PSEUDO_STATE_STRUCT,sizeof(PSEUDO_STATE_STRUCT));

	if(return_write_attempt == false){
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE STATE: FAILED");
		result = false;
	}
	else{
		Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE STATE: SUCCESS");
		result = true;
	}

	TALKER.startListening();
  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::readPseudoStatePacket(RF24 LISTENER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	struct pseudoStateDataStruct PSEUDO_STATE_STRUCT;
	
	const byte *read1Adress = pseudoAddresses[radioPseudoNumber-1];

	LISTENER.openReadingPipe(radioPseudoNumber,read1Adress);
	
	LISTENER.startListening();
	
	continue_exec = false;

	do{
		if( LISTENER.available(&radioPseudoNumber) ) 
		{

			while(LISTENER.available(&radioPseudoNumber)){
				LISTENER.read(&PSEUDO_STATE_STRUCT, sizeof(PSEUDO_STATE_STRUCT)); //
			}

			Serial.print("[   MASTER  ]   READ FROM: "); Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_STATE_STRUCT._pseudoID); Serial.print(" ]");
			
			switch(PSEUDO_STATE_STRUCT.pseudoState){
			case  STATE_LOCKED:
				Serial.print("[ STATE: "); Serial.print(STATE_LOCKED_STRING); Serial.println(" ]");
				continue_exec = true;
				break;
			case STATE_UNLOCKED:
				Serial.print("[ STATE: "); Serial.print(STATE_UNLOCKED_STRING); Serial.println(" ]");
				continue_exec = true;
				break;
			case IN_POSITION:
				Serial.print("[ STATE: "); Serial.print(STATE_IN_POSITION_STRING); Serial.println(" ]");
				continue_exec = true;
				break;		  
			default:
				Serial.println("MISMATCHING STATE CODE");
				continue_exec = true;
				break;
			}
			result = true;
		}
		else
		{
			Serial.print("[   MASTER  ]  "); Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_STATE_STRUCT._pseudoID); Serial.print(" ]"); Serial.println("READ STATE: FAILED");
			result = false;
			continue_exec = false;
		}
	}while(!continue_exec);

	LISTENER.stopListening();

  	LISTENER.closeReadingPipe(radioPseudoNumber);

  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::writeCommandPseudoPacket(RF24 TALKER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] , int command_code)
{
	struct pseudoCommandDataStruct PSEUDO_COMMAND_STRUCT;
	
	const byte *writeAdress = pseudoAddresses[radioPseudoNumber-1];

	TALKER.stopListening();

	TALKER.openWritingPipe(writeAdress);

	TALKER.stopListening();

	//PSEUDO_COMMAND_STRUCT._micros = micros();
	PSEUDO_COMMAND_STRUCT._pseudoID = radioPseudoNumber;
	PSEUDO_COMMAND_STRUCT.pseudoCommand  = command_code;
  	
	do{
		return_write_attempt = TALKER.write(&PSEUDO_COMMAND_STRUCT,sizeof(PSEUDO_COMMAND_STRUCT));

		if(return_write_attempt == false){
			Serial.print("[   MASTER  ]   "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE COMMAND: FAILED");
			result = false;
		}
		else{
			Serial.print("[   MASTER  ]   "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE COMMAND: SUCCESS");
			result = true;
		}
	}while(!result);

  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::readCommandPseudoPacket(RF24 LISTENER, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[] )
{
	struct pseudoCommandDataStruct PSEUDO_COMMAND_STRUCT;
	
	const byte *read1Adress = pseudoAddresses[radioPseudoNumber-1];

	LISTENER.openReadingPipe(radioPseudoNumber,read1Adress);
	
	LISTENER.startListening();
	
	continue_exec = false;

	Serial.println(radioPseudoNumber);

	do{

		if( LISTENER.available(&radioPseudoNumber) ) 
		{

			while(LISTENER.available(&radioPseudoNumber)){
				LISTENER.read(&PSEUDO_COMMAND_STRUCT, sizeof(PSEUDO_COMMAND_STRUCT)); 
			}

			Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_COMMAND_STRUCT._pseudoID); Serial.print(" ] EXECUTES: ");
			
			switch(PSEUDO_COMMAND_STRUCT.pseudoCommand){
			case  CMD_LOCK:
				Serial.print("[ COMMAND: "); Serial.print(COMMAND_LOCK_STRING); Serial.println(" ]");
				// ENABLE LOCK_PIN_RELAY

				slaveCommandReceived = PSEUDO_COMMAND_STRUCT.pseudoCommand;
				// SET FLAG TO WRITE NEW STATE
				continue_exec = true;
				CURRENT_STATE = STATE_LOCKED;
				break;
			case CMD_UNLOCK:
				Serial.print("[ COMMAND: "); Serial.print(COMMAND_UNLOCK_STRING); Serial.println(" ]");
				// DISABLE LOCK_PIN_RELAY

				// SET FLAG TO WRITE NEW STATE
				continue_exec = true;
				CURRENT_STATE = STATE_UNLOCKED;	
				break;
			case  CMD_SGP:
				Serial.print("[ COMMAND: "); Serial.print(COMMAND_SGP_STRING); Serial.println(" ]");
				// DRIVES MOTOR TO GOAL POSITION

				// SET FLAG TO WRITE NEW COMMAND: LOCK
				continue_exec = true;
				CURRENT_STATE = IN_POSITION;	
				break;
			case CMD_HOME:
				Serial.print("[ COMMAND: "); Serial.print(COMMAND_HOME_STRING); Serial.println(" ]");
				// DRIVES MOTOR TO ZERO POSITION

				// SET FLAG TO WRITE NEW COMMAND: LOCK
				continue_exec = true;
				CURRENT_STATE = IN_POSITION;
				break;			  
			default:
				continue_exec = true;
				Serial.print("MISMATCHING COMMAND CODE");
				break;
			}
			result = true;
		}
		else
		{
			Serial.print("[ PSEUDO: "); Serial.print(PSEUDO_COMMAND_STRUCT._pseudoID); Serial.print(" ]"); Serial.println(" COMMAND READ: FAILED");
			result = false;
			continue_exec = false;
		}

	}while(!continue_exec);
	  

	LISTENER.stopListening();
	LISTENER.closeReadingPipe(radioPseudoNumber);

  	return result;
}

// =========================================================================================================== //

bool PseudoRFcommMetamorphicManipulator::execTxRxBlockMaster(RF24 OBJECT, uint8_t radioPseudoNumber, typeAddresses pseudoAddresses[], int command_code, int * state_code )
{
	/*
	 *  Master becomes Tx -> Sets Slave to Rx
	 *  Master writes command_code
	 *  Waits until Slave responds with state_code
	 *  Return TRUE if the appropriate response is received
	 */

	// I. Set Master to Tx

	// When MASTER is Tx it uses one adress to write to each pseudo, the remaining 5 are used for reading(I can read from up to 5 pseudos at the same time)
	OBJECT.stopListening();											// calls before openWritingPipe when needs to change pipe address

	const byte *writeAdress = pseudoAddresses[radioPseudoNumber-1]; // set the pipe address depending on which pseudo we write
	//const byte *read1Adress = pseudoAddresses[radioPseudoNumber]; // will be removed

	OBJECT.openWritingPipe(writeAdress);    						// MASTER WRITES TO THIS PIPE TO PSEUDO1
	//OBJECT.openReadingPipe(radioPseudoNumber,read1Adress);		// will be removed

	OBJECT.stopListening();											// stops listening in order to write

	enum Mode setSlaveMode = Rx;									// Slave MUST be informed that becomes Rx!

	return_write_attempt = OBJECT.write(&setSlaveMode,sizeof(setSlaveMode));
	if(return_write_attempt == false){
		Serial.print("[   MASTER  ]  "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to SLAVE: FAILED");
		result = false;
	}
	else{
		Serial.print("[   MASTER  ]  "); Serial.print("[ PSEUDO: "); Serial.print(radioPseudoNumber); Serial.println(" ] WRITE Rx Mode to SLAVE: SUCCESS");
		result = true;
		digitalWrite(_txLedPin,HIGH);
		digitalWrite(_rxLedPin,LOW);
	}

	// II. Write command to Slave


} // END FUNCTION

// =========================================================================================================== //
