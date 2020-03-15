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
#include <EEPROM.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>

// Addresses
typedef const byte typeAddresses[ADDRESS_WIDTH]; 

/*
 *  FUNCTIONS FOR PseudoRFcommMetamorphicManipulator CLASS
 */

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

/*
 *  FUNCTIONS FOR PseudoSPIcommMetamorphicManipulator CLASS
 */

PseudoSPIcommMetamorphicManipulator::PseudoSPIcommMetamorphicManipulator(enum Mode mode, int pseudoID, int mosiPin, int misoPin, int sckPin, int txLedPin, int rxLedPin, int ssPins[]){

// Construct MASTER/SLAVE object => sets the corresponding pin modes
if (mode == Tx)
{
	pinMode(sckPin, OUTPUT);

	pinMode(mosiPin, OUTPUT);
	pinMode(misoPin, INPUT);

	for (size_t i = 0; i < sizeof(ssPins); i++)
	{
		pinMode(ssPins[i], OUTPUT);
	}
}

if (mode == Rx)
{
	pinMode(sckPin, !OUTPUT);

	pinMode(mosiPin, !OUTPUT);
	pinMode(misoPin, !INPUT);

	for (size_t i = 0; i < sizeof(ssPins); i++)
	{
		pinMode(ssPins[i], !OUTPUT);
	}
}

	// LEDS ARE OFF @ construction
	pinMode(txLedPin,OUTPUT);
	pinMode(rxLedPin,OUTPUT);
	digitalWrite(txLedPin,LOW);
	digitalWrite(rxLedPin,LOW);

	_pseudoID = pseudoID;
	_mosiPin  = mosiPin;
	_misoPin  = misoPin;
	_sckPin   = sckPin;

	_txLedPin = txLedPin;
	_rxLedPin = rxLedPin;

}

// =========================================================================================================== //

void PseudoSPIcommMetamorphicManipulator::constructPacket(aliasPacketReceived *PACKET, int pseudoID, int command_code){
  PACKET->micros_talked    = micros();
  PACKET->pseudoID_talked  = pseudoID;
  PACKET->command_state    = command_code;
}

// =========================================================================================================== //

void PseudoSPIcommMetamorphicManipulator::constructEptyPacket(aliasPacketReceived *PACKET){
    PACKET->micros_talked    = 0;
    PACKET->pseudoID_talked  = 0;
    PACKET->command_state    = 0;
}
// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::executeTxRxMasterBlock(aliasPacketReceived PACKET, int pseudoID , int ssPins[]){

  aliasPacketReceivedUnion PACKET_UNION;

  PACKET_UNION.packet_received = PACKET;

  digitalWrite(ssPins[pseudoID-1], LOW);              // enable Slave Select

  for (size_t index_count = 0; index_count < sizeof(packets); index_count++)
  {
      SPI.transfer(PACKET_UNION.bytes[index_count]);
	  Serial.print(PACKET_UNION.bytes[index_count]);
  }

  // End Write byte command
  delay(50);
  Serial.println("");
  Serial.print("receive time   ="); Serial.println(PACKET.micros_talked);
  Serial.print("pseudo talked  ="); Serial.println(PACKET.pseudoID_talked);
  Serial.print("motor state    ="); Serial.println(PACKET.command_state);

  digitalWrite(ssPins[pseudoID-1], HIGH);             // disable Slave Select

  return true;
}

// =========================================================================================================== //

byte PseudoSPIcommMetamorphicManipulator::singleByteTransfer(byte packet, unsigned long wait_for_response)
{	
	time_now_micros = micros();

	byte packet_received = SPI.transfer(packet);
	while(micros() < time_now_micros + wait_for_response){}

	return packet_received;
}

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::connectPseudoMaster(int pseudoID, int ssPins[])
{
	/*
	 *  Checks 1 time only for the ID of the pseudo connected in the ssPin specified
	 */

	byte receivedID;
	unsigned long eeprom_read_time_micros = 4000;  		// 4ms (4000μs) for read 

	digitalWrite(ssPins[pseudoID-1], LOW);				// enable Pseudo Slave Select pin

	receivedID = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_CONNECT, eeprom_read_time_micros);

	digitalWrite(ssPins[pseudoID-1], HIGH);				// disable Pseudo Slave Select pin

	if( pseudoID == (int) receivedID)
	{
		return true;
	}
	else
	{
		return false;
	}
	
} // END FUNCTION: connectPseudoMaster

// =========================================================================================================== //

byte PseudoSPIcommMetamorphicManipulator::connectPseudoSlave()
{
	/*
	 *  This is the response from connectPseudoMaster.
	 *  Reads default EEPROM Memory adress and returns the ID
	 */

	byte PSEUDO_ID;

	PSEUDO_ID = EEPROM.read(ID_EEPROM_ADDR);

	return PSEUDO_ID;
} // END FUNCTION: connectPseudoSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::readInitialStateMaster(int pseudoID, int ssPins[] )
{
	/*
	 *  Reads 1 time only for the current stete of the pseudo connected in the ssPin specified
	 *  Returns true only if pseudo's initial state is locked
	 */	

	byte receivedCS;
	unsigned long eeprom_read_time_micros = 4000;  		// 4ms (4000μs) for read 

	digitalWrite(ssPins[pseudoID-1], LOW);				// enable Pseudo Slave Select pin

	receivedCS = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_GIVE_CS, eeprom_read_time_micros);

	digitalWrite(ssPins[pseudoID-1], HIGH);				// disable Pseudo Slave Select pin

	if( (receivedCS == STATE_LOCKED) )
	{
		return true;
	}
	else
	{
		return false;
	}	
} // END FUNCTION: readInitialStateMaster

// =========================================================================================================== //

byte PseudoSPIcommMetamorphicManipulator::readInitialStateSlave()
{
	/*
	 *  This is the response from readInitialStateMaster.
	 *  Reads default EEPROM Memory adress and returns the CS
	 */

	byte PSEUDO_CS;

	PSEUDO_CS = EEPROM.read(CS_EEPROM_ADDR);

	return PSEUDO_CS;
} // END FUNCTION: readInitialStateSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::lockPseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE  )
{
	/*
	 *  Orders slave to lock
	 *  Returns true only if locked achieved
	 */	

	unsigned long wait_for_slave_response = SLAVE_RESPONSE_TIME;  		// 10μs for response

	digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

	*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_LOCK, wait_for_slave_response);

	digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	if( (*CURRENT_STATE == STATE_LOCKED) )
	{
		return true;
	}
	else
	{
		return false;
	}	
} // END FUNCTION: lockPseudoMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::lockPseudoSlave(byte *CURRENT_STATE)
{
	/*
	 *  This is the response from lockPseudoMaster.
	 *  1. Checks if motor is IN_POSTION  -> if not responds false
	 *  2. Controls the relay of lock pin
	 *  3. Returns the new state
	 */

	if((*CURRENT_STATE == IN_POSITION))		// check current state
	{
		digitalWrite(RELAY_lock_Pin, HIGH);       // locks when NO connected
		
		*CURRENT_STATE = STATE_LOCKED;		// Change state

		result = true;
	}
	else
	{
		result = false;
	}

	return result;	
} // END FUNCTION: lockPseudoSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::unlockPseudoMaster(int pseudoID, int ssPins[], byte *CURRENT_STATE  )
{
	/*
	 *  Orders slave to lock
	 *  Returns true only if locked achieved
	 */	

	unsigned long wait_for_slave_response = SLAVE_RESPONSE_TIME;  		// 10μs for response

	digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

	*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_UNLOCK, wait_for_slave_response);

	digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	if( (*CURRENT_STATE == STATE_UNLOCKED) )
	{
		return true;
	}
	else
	{
		return false;
	}	
} // END FUNCTION: lockPseudoMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::unlockPseudoSlave(byte *CURRENT_STATE)
{
	/*
	 *  This is the response from lockPseudoMaster.
	 *  1. Checks if motor is READY(GOAL POSITION is set)  -> if not responds false
	 *  2. Controls the relay of lock pin
	 *  3. Returns the new state
	 */

	if((*CURRENT_STATE == STATE_READY))			// check current state
	{
		digitalWrite(RELAY_lock_Pin, LOW);      // unlocks when NO connected
		
		*CURRENT_STATE = STATE_UNLOCKED;		// Change state

		result = true;
	}
	else
	{
		result = false;
	}

	return result;	
} // END FUNCTION: lockPseudoSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::setGoalPositionMaster(int pseudoID, int ssPins[], byte GP, byte *CURRENT_STATE )
{
	/*
	 *  Orders slave to set Goal Position, GP is specified by user
	 *  GP range is determined by min/max pseudo position and step angle
	 *  Returns true only if Ready state achieved
	 */	

	unsigned long wait_for_slave_response = SLAVE_RESPONSE_TIME;  		// 10μs for response

	digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

	*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer(GP, wait_for_slave_response);

	digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	if( (*CURRENT_STATE == STATE_READY) )
	{
		return true;
	}
	else
	{
		return false;
	}
} // END FUNCTION: setGoalPositionMaster

// =========================================================================================================== //
