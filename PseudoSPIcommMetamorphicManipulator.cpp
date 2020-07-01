/*
 *  Source Code for library PseudoRFcommMetamorphicManipulator.h
 *  using the RF24L01 module for Arduino
 *  Created by N.A. Stravopodis, March, 2020.
 */

// Default includes for driving pseudojoint steppers
#include "Arduino.h"
//#include <printf.h>
#include "PseudoSPIcommMetamorphicManipulator.h"
//#include <Eventually.h>

// For RF PseudoCommunication
//#include <SPI.h>
//#include <EEPROM.h>
//#include <RF24.h>
//#include <nRF24L01.h>
//#include <RF24_config.h>

// Include Motor Configuration files from folder ~/Arduino/libraries/test_metamorphic_manipulator_configuration
#include <definitions.h>                            
#include <motorIDs.h>                               
#include <contolTableItems_LimitValues.h>
#include <StepperMotorSettings.h>

// Addresses
typedef const byte typeAddresses[ADDRESS_WIDTH]; 

/*
 *  FUNCTIONS FOR PseudoRFcommMetamorphicManipulator CLASS
 */
/*
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

bool PseudoRFcommMetamorphicManipulator::readCommandPseudoPacket(RF24 LISTENER, uint8_t radioPseudoNumber, byte *CURRENT_STATE, typeAddresses pseudoAddresses[] )
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
				*CURRENT_STATE = STATE_LOCKED;
				break;
			case CMD_UNLOCK:
				Serial.print("[ COMMAND: "); Serial.print(COMMAND_UNLOCK_STRING); Serial.println(" ]");
				// DISABLE LOCK_PIN_RELAY

				// SET FLAG TO WRITE NEW STATE
				continue_exec = true;
				*CURRENT_STATE = STATE_UNLOCKED;	
				break;
			case  CMD_SGP:
				Serial.print("[ COMMAND: "); Serial.print(COMMAND_SGP_STRING); Serial.println(" ]");
				// DRIVES MOTOR TO GOAL POSITION

				// SET FLAG TO WRITE NEW COMMAND: LOCK
				continue_exec = true;
				*CURRENT_STATE = IN_POSITION;	
				break;
			case CMD_HOME:
				Serial.print("[ COMMAND: "); Serial.print(COMMAND_HOME_STRING); Serial.println(" ]");
				// DRIVES MOTOR TO ZERO POSITION

				// SET FLAG TO WRITE NEW COMMAND: LOCK
				continue_exec = true;
				*CURRENT_STATE = IN_POSITION;
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
	//
	//  Master becomes Tx -> Sets Slave to Rx
	//  Master writes command_code
	//  Waits until Slave responds with state_code
	//  Return TRUE if the appropriate response is received
	//

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
*/
// =========================================================================================================== //

/*
 *  FUNCTIONS FOR PseudoSPIcommMetamorphicManipulator CLASS
 */

PseudoSPIcommMetamorphicManipulator::PseudoSPIcommMetamorphicManipulator(enum Mode mode, int pseudoID, int statusLedPin, int mosiPin, int misoPin, int sckPin, int txLedPin, int rxLedPin, int ssPins[]){

// Construct MASTER/SLAVE object => sets the corresponding pin modes
if (mode == Tx)
{
	pinMode(sckPin, OUTPUT);

	pinMode(mosiPin, OUTPUT);
	pinMode(misoPin, INPUT);

	for (size_t i = 0; i < sizeof(ssPins); i++)
	{
		pinMode(ssPins[i], OUTPUT);
		pinMode(ssPins[i], HIGH);
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
	pinMode(statusLedPin,OUTPUT);

	digitalWrite(txLedPin,LOW);
	digitalWrite(rxLedPin,LOW);
	digitalWrite(statusLedPin,LOW);
	
	_pseudoID = pseudoID;
	_mosiPin  = mosiPin;
	_misoPin  = misoPin;
	_sckPin   = sckPin;

	_txLedPin = txLedPin;
	_rxLedPin = rxLedPin;
	_statusLedPin = statusLedPin;

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
	unsigned long time_now_micros = micros();

	byte packet_received = SPI.transfer(packet);
	delayMicroseconds (wait_for_response);					// Code MUST BLOCK!!! NO while()...
	return packet_received;
}

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::connectPseudoMaster(int pseudoID, int ssPins[])
{
	/*
	 *  Checks 1 time only for the ID of the pseudo connected in the ssPin specified
	 */
	bool result;

	byte receivedID;
	unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
	unsigned long time_now_micros 			= micros();
	bool slave_responded_correct_flag		= false;
	unsigned long eeprom_read_time_micros 	= 3500;  					// 3.5ms (3500μs) for read 

	digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

	do{
		receivedID = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_CONNECT, eeprom_read_time_micros);
		if( pseudoID == (int) receivedID)
		{
			slave_responded_correct_flag = true;
			result =  true;
		}
		else
		{	
			result =  false;
		}
		
	//}while( (micros() < time_now_micros + wait_total_response_time_micros) && (!slave_responded_correct_flag) );
	}while( (!slave_responded_correct_flag) );

	digitalWrite(ssPins[pseudoID-1], HIGH);				// disable Pseudo Slave Select pin

	return result;
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

bool PseudoSPIcommMetamorphicManipulator::readCurrentStateMaster(int pseudoID, int ssPins[], volatile byte *CURRENT_STATE)
{
	/*
	 *  Reads for the current state of the pseudo connected in the ssPin specified
	 *  Returns true only if pseudo's initial state is locked
	 */	
	bool result;

	unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
	unsigned long time_now_micros 			= micros();
	bool slave_responded_correct_flag		= false;
	unsigned long eeprom_read_time_micros 	= 3500;  					// 3.5ms (3500μs) for read 

	digitalWrite(ssPins[pseudoID-1], LOW);				// enable Pseudo Slave Select pin

	do{
		//Serial.print("I read state in readCurrentStateMaster to start meta: "); Serial.println(*CURRENT_STATE);

		*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_GIVE_CS, slave_response);
		
		if( (*CURRENT_STATE == STATE_LOCKED) || (*CURRENT_STATE == META_REPEAT) || (*CURRENT_STATE == META_FINISHED) || (*CURRENT_STATE == IN_POSITION) )
		{
			slave_responded_correct_flag = true;
			result = true;
		}
		else
		{
			result = false;
		}
		
	}while( (!slave_responded_correct_flag) );

	digitalWrite(ssPins[pseudoID-1], HIGH);				// disable Pseudo Slave Select pin

	return result;	
} // END FUNCTION: readInitialStateMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::readCurrentStateSlave(volatile byte *CURRENT_STATE )
{
	/*
	 *  This is the response from readCurrentStateMaster.
	 *  Slave after each metamorphosis execution saves CS to EEPROM memory
	 *  Each time a meta mode starts/repeats reads the last state of slave
	 *  Reading CS only expects 3 appropriate returns!!! Otherwise errors!!!
	 */

	bool result;

	*CURRENT_STATE = EEPROM.read(CS_EEPROM_ADDR);

	if ( (*CURRENT_STATE == STATE_LOCKED) || (*CURRENT_STATE == META_FINISHED) || (*CURRENT_STATE == META_REPEAT) || (*CURRENT_STATE == IN_POSITION) )
	{
		result = true;
	}
	else
	{
		result = false;
	}
	

	return result;
} // END FUNCTION: readCurrentStateSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::saveCurrentStateSlave( volatile byte *CURRENT_STATE )
{
	/*
	 *  This function is executed after continueMetaExecutionMaster/REPEAT is executed from Master
	 *  Saves META_REPEAT to EEPROM memory
	 */

	bool result;

	if ( (*CURRENT_STATE == STATE_LOCKED) )
	{
		*CURRENT_STATE = META_REPEAT;

		EEPROM.update(CS_EEPROM_ADDR, *CURRENT_STATE);

		result = true;
	}
	else
	{
		result = false;
	}
	
	return result;
} // END FUNCTION: readCurrentStateSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::lockPseudoMaster(int pseudoID, int ssPins[], volatile byte *CURRENT_STATE  )
{
	/*
	 *  Orders slave to lock only if is in position
	 *  Returns true only if locked achieved
	 */	
	bool result;

	if (*CURRENT_STATE == IN_POSITION) 
	{
		unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
		unsigned long time_now_micros 			= micros();
		bool slave_responded_correct_flag		= false;

		digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

		do{

			*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_LOCK, slave_response);

			if( (*CURRENT_STATE == STATE_LOCKED) )
			{
				slave_responded_correct_flag = true;
				result = true;
			}
			else
			{
				result = false;
			}	

		//}while( (micros() < time_now_micros + wait_total_response_time_micros) && (!slave_responded_correct_flag) );
		}while( (!slave_responded_correct_flag) );

		digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	}
	else
	{
		result =  false;
	}
	

	return result;
} // END FUNCTION: lockPseudoMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::lockPseudoSlave(volatile byte *CURRENT_STATE)
{
	/*
	 *  This is the response from lockPseudoMaster.
	 *  1. Checks if motor is IN_POSTION  -> if not responds false
	 *  2. Controls the relay of lock pin
	 *  3. Returns the new state
	 */
	if((*CURRENT_STATE == IN_POSITION))		// check current state
	{
		digitalWrite(RELAY_lock_Pin, HIGH);        
		digitalWrite(RELAY_lock_Pin2, HIGH);

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

bool PseudoSPIcommMetamorphicManipulator::unlockPseudoMaster(int pseudoID, int ssPins[], volatile byte *CURRENT_STATE  )
{
	/*
	 *  Orders slave to unlock only if GP has been set
	 *  Returns true only if locked achieved
	 */	
	bool result;

	if((*CURRENT_STATE == STATE_READY) || (*CURRENT_STATE == META_FINISHED))										// check current state
	{
		unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
		unsigned long time_now_micros 			= micros();
		bool slave_responded_correct_flag		= false;

		digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

		do{

			*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte ) CMD_UNLOCK, slave_response);
		
			if( (*CURRENT_STATE == STATE_UNLOCKED) )
			{
				slave_responded_correct_flag = true;
				result = true;
			}
			else
			{
				result = false;
			}

		//}while( (micros() < time_now_micros + wait_total_response_time_micros) && (!slave_responded_correct_flag) );
		}while( (!slave_responded_correct_flag) );

		digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	}
	else
	{
		result = false;
	}
	
	return result;
} // END FUNCTION: lockPseudoMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::unlockPseudoSlave(volatile byte *CURRENT_STATE)
{
	/*
	 *  This is the response from lockPseudoMaster.
	 *  1. Checks if motor is READY(GOAL POSITION is set)  -> if not responds false
	 *  2. Controls the relay of lock pin
	 *  3. Returns the new state
	 */
	bool result;
	Serial.println(*CURRENT_STATE);

	if((*CURRENT_STATE == STATE_LOCKED) || (*CURRENT_STATE == STATE_READY) || (*CURRENT_STATE == META_REPEAT) || (*CURRENT_STATE == META_FINISHED) )				// check current state
	{
		digitalWrite(RELAY_lock_Pin, LOW);      	
		digitalWrite(RELAY_lock_Pin2, LOW);

		*CURRENT_STATE = STATE_UNLOCKED;			// Change state

		result = true;
	}
	else
	{
		result = false;
	}

	return result;	
} // END FUNCTION: lockPseudoSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::setGoalPositionMaster(int pseudoID, int ssPins[], byte *GP, volatile byte *CURRENT_STATE )
{
	/*
	 *  Orders slave to set Goal Position, GP is specified by user
	 *  GP range is determined by min/max pseudo position and step angle
	 *  Returns true only if Ready state achieved
	 */	
	bool result;
	
	//Serial.println("setGoalPositionMaster!!!!!!!!!!!!"); // OKOK

	if ( (*CURRENT_STATE == STATE_LOCKED) || (*CURRENT_STATE == META_FINISHED) || (*CURRENT_STATE == META_REPEAT)  ) 	
	{		
		unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  		// how much waits inside transfer function
		unsigned long time_now_micros 			= micros();
		bool slave_responded_correct_flag		= false;
		unsigned long eeprom_read_time_micros 	= 4000;  						// 3.5ms (3500μs) for read 

		digitalWrite(ssPins[pseudoID-1], LOW);									// enable Pseudo Slave Select pin
		
		do{

			*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer( *GP , slave_response);
			//Serial.print("current state = "); Serial.println(*CURRENT_STATE);
			if( (*CURRENT_STATE == STATE_READY) )
			{
				slave_responded_correct_flag	= true;
				result =  true;
			}
			else
			{
				slave_responded_correct_flag	= false;
				result = false;
			}

		//}while( (micros() < time_now_micros + wait_total_response_time_micros) && (!slave_responded_correct_flag) );
		}while( (!slave_responded_correct_flag) );
		
		digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	}
	else
	{
		result = false;
	}
	
	return result;
} // END FUNCTION: setGoalPositionMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::setGoalPositionSlave( byte *PSEUDO_GOAL_POSITION, int *RELATIVE_STEPS_TO_MOVE, volatile byte *CURRENT_STATE )
{
	/*
	 *  Respond to setGoalPositionMaster, to execute pseudo must be locked
	 *  1. Reads current position from global variable after initialized at setup() from EEPROM (currentAbsPosPseudo_ci)
	 *  2. Calculates relative position(from external library function)
	 *  3. Saves relative position motor must move
	 *  4. Returns true only if Ready state achieved
	 */	
	bool result;

	if( (*CURRENT_STATE == STATE_LOCKED) || (*CURRENT_STATE == META_FINISHED) || (*CURRENT_STATE == META_REPEAT) )
	{
		
		float step_angle = 0.00f;

		EEPROM.get(STEP_ANGLE_ADDR, step_angle);    			// @setup: float f = 123.456f; EEPROM.put(eeAddress, f);

		theta_p_goal    = step_angle * (*PSEUDO_GOAL_POSITION - 1) + min_pseudo_angle;

		Serial.print("theta_p_goal="); Serial.println(theta_p_goal);

		// calculate relative motion steps/direction
		return_function_state = PseudoSPIcommMetamorphicManipulator::calculateRelativeStepsToMove(&theta_p_goal, RELATIVE_STEPS_TO_MOVE);
		if (return_function_state)
		{
			*CURRENT_STATE = STATE_READY;
		
			result = true;
		}
		else
		{
			result = false;
		}	

	}
	else
	{
		result = false;
	}

	return result;

} // END FUNCTION: setGoalPositionSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::setGoalPositionSlave2( byte *PSEUDO_GOAL_POSITION, byte *currentAbsPosPseudo_ci, int *RELATIVE_STEPS_TO_MOVE, byte *currentDirStatusPseudo, volatile byte *CURRENT_STATE )
{
	/*
	 *  Respond to setGoalPositionMaster, to execute pseudo must be locked = > Set number of relative steps to move and direction !!!
	 *  1. Reads goal position ci (PSEUDO_GOAL_POSITION) 
	 *  2. Reads current position from global variable after initialized at setup() from EEPROM (currentAbsPosPseudo_ci)
	 *  3. Reads theta step (float)
	 *  4. Calculates Delta theta_p (float) and (int) using METAMORPHOSIS_Ci_STEPS
	 *  5. Calculates direction
	 */	
	bool result;
	//Serial.print("state in setGoalPositionSlave2 = "); Serial.println(*CURRENT_STATE);

	if( (*CURRENT_STATE == STATE_LOCKED) || (*CURRENT_STATE == META_FINISHED) || (*CURRENT_STATE == META_REPEAT) )
	{
		//Serial.println("PSEUDO_GOAL_POSITION = "); Serial.println(*PSEUDO_GOAL_POSITION);
		//delay(1000);
		//Serial.println("Mphka setGoalPositionSlave2");
		// 3. Reading theta step
		//float step_angle = 0.0000f;

		//EEPROM.get(STEP_ANGLE_ADDR, step_angle);    				

		// 4. Delta theta_p (float)
		//float delta_theta_p = step_angle * abs( *PSEUDO_GOAL_POSITION - *currentAbsPosPseudo_ci );

		//*RELATIVE_STEPS_TO_MOVE = round (delta_theta_p / ag );// inaccuracies bugs

		*RELATIVE_STEPS_TO_MOVE = METAMORPHOSIS_Ci_STEPS * abs( *PSEUDO_GOAL_POSITION - *currentAbsPosPseudo_ci );

		// 5. Direction status
		if ( *PSEUDO_GOAL_POSITION <= *currentAbsPosPseudo_ci)
		{
			*currentDirStatusPseudo = LOW;						// move CCW
		}
		else
		{
			*currentDirStatusPseudo = HIGH;						// move CW
		}
		
		digitalWrite(dirPin_NANO, *currentDirStatusPseudo);

		// Print the results in serial monitor
		/*
		Serial.print("[   setGoalPositionSlave2   ]   SUCCESS"); 
		Serial.println("-------------------------------------");
		Serial.print("[    CURRENT_POS_ci    ]    [   "); Serial.print(*currentAbsPosPseudo_ci); Serial.println("     ]");
		Serial.print("[      GOAL_POS_ci     ]    [   "); Serial.print(*PSEUDO_GOAL_POSITION); Serial.println("       ]");
		Serial.print("[RELATIVE_STEPS_TO_MOVE]    [   "); Serial.print(*RELATIVE_STEPS_TO_MOVE); Serial.println(" ]");
		Serial.print("[ FLOAT_ANGLE_TO_MOVE  ]    [   "); Serial.print(delta_theta_p); Serial.println(" ]");
		Serial.print("[   DIRECTION_TO_MOVE  ]    [   "); Serial.print(*currentDirStatusPseudo); Serial.println("     ]");
		Serial.println("-------------------------------------");
		delay(10000);
		*/

		*currentAbsPosPseudo_ci = *PSEUDO_GOAL_POSITION;		// assumes that moveSlave executes successfully and changes global variable

		//Added on 30.6->
		EEPROM.update(CD_EEPROM_ADDR, *currentDirStatusPseudo);
		Serial.println("[INFO]  SAVED	[	CD	]	TO 	EEPROM");

		// 2. Save current absolute position in which pseudo is locked after metamorphosis
		EEPROM.update(CP_EEPROM_ADDR, *currentAbsPosPseudo_ci);
		Serial.println("[INFO]  SAVED	[	CP	] 	TO 	EEPROM");
		//<-

		*CURRENT_STATE = STATE_READY;
		
		result = true;
		
	}
	else
	{
		result = false;
	}

	return result;

} // END FUNCTION: setGoalPositionSlave

// =========================================================================================================== //

void PseudoSPIcommMetamorphicManipulator::readCurrentPseudoPosition(float *theta_p_current, int *theta_p_current_steps)
{
	/*
	 *	EXECUTED at setup() ONLY: 
	 *  1. Reads the current position of pseudojoint saved at EEPROM
	 *  2. saves the variable to RAM Memory as global variable for metamorphosis tests
	 */

	PSEUDO_CURRENT_POSITION = EEPROM.read(CP_EEPROM_ADDR);

	float step_angle = 0.00f;

	EEPROM.get(STEP_ANGLE_ADDR, step_angle);    			// @setup: float f = 123.456f; EEPROM.put(eeAddress, f);
	
	// calculate float angles from anatomy ci 
	*theta_p_current = step_angle * (PSEUDO_CURRENT_POSITION - 1) + min_pseudo_angle;

	*theta_p_current_steps = round( *theta_p_current / ag);
}

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::calculateRelativeStepsToMove(float *theta_p_goal, int *RELATIVE_STEPS_TO_MOVE){

 	/*
	 *  Called inside setGoalPositionSlave()
	 *  Returns the max value for counter inside moveSlave()
	 */ 
	uint32_t newDirStatus;

	int inputAbsPos = round( *theta_p_goal / ag);

	byte previousDirStatus 		= currentDirStatusPseudo;
	int previousMoveRel  		= currentMoveRelPseudo; 
	int previousAbsPos   		= currentAbsPosPseudo;
	
	if (inputAbsPos == previousAbsPos){
		return false;                                 	// Already there
	}

	*RELATIVE_STEPS_TO_MOVE = inputAbsPos - previousAbsPos;
	Serial.print("RELATIVE_STEPS_TO_MOVE = "); Serial.println(*RELATIVE_STEPS_TO_MOVE);

	if( *RELATIVE_STEPS_TO_MOVE*previousMoveRel >=0 )
	{
		newDirStatus = previousDirStatus;
		digitalWrite(dirPin_NANO, newDirStatus);     	  	// Direction doesn't change
	}
	else
	{
		newDirStatus = !previousDirStatus;
		digitalWrite(dirPin_NANO, newDirStatus);       		// Direction changes
	}
	
		// Saves global variables for next call
		currentDirStatusPseudo = newDirStatus;
		currentMoveRelPseudo   = *RELATIVE_STEPS_TO_MOVE;
		currentAbsPosPseudo    = inputAbsPos;

		// Calculate Velocity - Acceleration
		return true;
}

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::movePseudoMaster(int pseudoID, int ssPins[], volatile byte *CURRENT_STATE )
{
	/*
	 *  Orders slave to lock
	 *  Returns true only if locked achieved
	 */	
	bool result;

	if ( *CURRENT_STATE == STATE_UNLOCKED )
	{
		unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
		unsigned long time_now_micros 			= micros();
		bool slave_responded_correct_flag		= false;

		digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

		do{

			*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_MOVE, slave_response);


			if( (*CURRENT_STATE == IN_POSITION) )
			{
				slave_responded_correct_flag = true;
				result = true;
			}
			else
			{
				result = false;
			}

		//}while( (micros() < time_now_micros + wait_total_response_time_micros) && (!slave_responded_correct_flag) );
		}while( (!slave_responded_correct_flag) );

		digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	}
	else
	{
		result = false;
	}
	
	return result;
} // END FUNCTION: lockPseudoMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::movePseudoSlave(  volatile byte *CURRENT_STATE , int *RELATIVE_STEPS_TO_MOVE)
{
	/*
	 *  Respond to movePseudoMaster, to execute pseudo must be unlocked
	 *  1. executes stepper motion for RELATIVE_STEPS_TO_MOVE
	 *  2. Returns true only if IN_POSITION state achieved
	 */	

	if( (*CURRENT_STATE == STATE_UNLOCKED) )
	{
		// moves motor
		for(int motor_step = 0; motor_step < abs( *RELATIVE_STEPS_TO_MOVE ); motor_step++){
          
		    time_now_micros = micros();

			digitalWrite(stepPin_NANO, HIGH);
    		while(micros() < time_now_micros + 250){}                   //wait approx. [μs]
			//delayMicroseconds(250);
    		digitalWrite(stepPin_NANO, LOW);

          	Serial.println("Stepper moving");
          	Serial.print("current step = "); Serial.println(motor_step);
        }	

		// motor finished - change state only. CP - CD have already changed in setGoalPositionSlave2
		*CURRENT_STATE = IN_POSITION;

		result = true;
	}
	else
	{
		result = false;
	}

	return result;

} // END FUNCTION: movePseudoSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::continueMetaExecutionMaster(int pseudoID, int ssPins[], byte USER_COMMAND, bool *finishMetaMode, volatile byte *CURRENT_STATE )
{
	/*
	 *  Orders slave to stop Metamorphosis Execution
	 *  Slave must execute saveEEPROMsettingsSlave() and change the flags respectively
	 *  The same flags(for Master) must change accordingly!!!
	 *  Master acknowledges that <METAMORPHOSIS> operation mode has finished successfully
	 */	
	bool result;

	if (*CURRENT_STATE == STATE_LOCKED) // executes only if pseudo has returned LOCKED
	{
		unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
		unsigned long time_now_micros 			= micros();
		bool slave_responded_correct_flag		= false;
		unsigned long waiting 	= 15000;  					

		digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

		do{

			*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer(USER_COMMAND, slave_response);
			
			if( (*CURRENT_STATE == META_FINISHED) )
			{
				Serial.print("STATE RECEIVED : "); Serial.println(*CURRENT_STATE);
				result = true;
				*finishMetaMode = true;
				slave_responded_correct_flag	= true;
			}
			else if ( (*CURRENT_STATE == META_REPEAT) )
			{
				Serial.print("STATE RECEIVED : "); Serial.println(*CURRENT_STATE);
				result = true;
				*finishMetaMode = false;
				slave_responded_correct_flag	= true;
			}
			else
			{
				result = false;
			}	
		Serial.println("edw");
		Serial.println(*CURRENT_STATE);

		}while( (!slave_responded_correct_flag) );

		digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin
	
	}
	else
	{
		result = false;
	}
	
	return result;
} // END FUNCTION: setGoalPositionMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::saveEEPROMsettingsMaster(int pseudoID, int ssPins[], volatile byte *CURRENT_STATE  )
{
	/*
	 *  Orders slave to lock only if is in position
	 *  Returns true only if locked achieved
	 */	
	bool result;

	if (*CURRENT_STATE == STATE_LOCKED) 
	{
		unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
		unsigned long time_now_micros 			= micros();
		bool slave_responded_correct_flag		= false;
		unsigned long eeprom_read_time_micros 	= 3500;  					// 3.5ms (3500μs) for read 

		digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

		do{
			Serial.print("Mphka saveEEPROMsettingsMaster(MUST BE 100)"); Serial.println(*CURRENT_STATE);
			*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_SAVE_EEPROM, slave_response);

			if( (*CURRENT_STATE == META_FINISHED) )
			{
				slave_responded_correct_flag = true;
				result = true;
			}
			else
			{
				result = false;
			}	
		//}while( (micros() < time_now_micros + wait_total_response_time_micros) && (!slave_responded_correct_flag) );
		}while( (!slave_responded_correct_flag) );

		digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	}
	else
	{
		result =  false;
	}
	

	return result;
} // END FUNCTION: lockPseudoMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::saveEEPROMsettingsSlave(volatile byte *CURRENT_STATE , byte * currentAbsPosPseudo_ci, byte * currentDirStatusPseudo)
{
	/*
	 *	Responds to continueMetaExecutionMaster
	 */
	bool result;
	Serial.print("State in saveEEPROMsettingsSlave"); Serial.println(*CURRENT_STATE);

	//if( (*CURRENT_STATE == STATE_LOCKED) )
	for (size_t one_time_counter = 0; one_time_counter < 1; one_time_counter++)
	{
		// 1. Save the dirPin status
		EEPROM.update(CD_EEPROM_ADDR, *currentDirStatusPseudo);
		Serial.println("[INFO]  SAVED	[	CD	] 	TO 	EEPROM");

		// 2. Save current absolute position in which pseudo is locked after metamorphosis
		EEPROM.update(CP_EEPROM_ADDR, *currentAbsPosPseudo_ci);
		Serial.println("[INFO]  SAVED	[	CP	] 	TO 	EEPROM");

		// 3. Save the last state of pseudo(always meta finished in order to save)
		EEPROM.update(CS_EEPROM_ADDR, META_FINISHED);
		Serial.println("[INFO]  SAVED	[	CS	] 	TO 	EEPROM");

		*CURRENT_STATE = META_FINISHED;

		result = true;
	}

	return result;

} // END FUNCTION: saveEEPROMsettingsSlave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::repeatMetaSlave( volatile byte *CURRENT_STATE)
{
	/*
	 *	Responds to continueMetaExecutionMaster/REPEAT -> discontnued by saveCurrentStateSlave
	 */
	bool result;

	if( (*CURRENT_STATE == STATE_LOCKED) )
	{	
		*CURRENT_STATE = META_REPEAT;

		result = true;
	}
	else
	{
		result = false;
	}
	
	return result;

} // END FUNCTION: saveEEPROMsettingsSlave

// =========================================================================================================== //

void PseudoSPIcommMetamorphicManipulator::readEEPROMsettingsSlave(int pseudoID, volatile byte *CURRENT_STATE , byte * currentAbsPosPseudo_ci,  byte *currentDirStatusPseudo, int *currentAbsPosPseudo)
{
	/*
	 *	This function is executed at setup() to initialize the global variables of slave
	 */

	// 1. Read the saved dirPin status
	*currentDirStatusPseudo = EEPROM.read(CD_EEPROM_ADDR);

	// 2. Read the saved absolute ci
	*currentAbsPosPseudo_ci = EEPROM.read(CP_EEPROM_ADDR);

	float step_angle = 0.000000f;
	EEPROM.get(STEP_ANGLE_ADDR, step_angle);    			

	float currentAbsPosPseudo_float = 0.000000f;
	currentAbsPosPseudo_float = step_angle * (*currentAbsPosPseudo_ci - 1) + min_pseudo_angle;

	*currentAbsPosPseudo = abs( 	round( currentAbsPosPseudo_float / ag)    );			

	// 3. Read the saved state
	*CURRENT_STATE = EEPROM.read(CS_EEPROM_ADDR);

	// Print the results in serial monitor
    Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.println("   ]   [READ EEPROM SETTINGS]   SUCCESS"); 
	Serial.println("-----------------------------------------------------------------------------------------------");
	Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT_STATE    ]    [   "); Serial.print(*CURRENT_STATE); Serial.println("   ]");
	Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT_POS_ci   ]    [   "); Serial.print(*currentAbsPosPseudo_ci); Serial.println("     ]");
	Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [  CURRENT_POS_angle ]    [   "); Serial.print(currentAbsPosPseudo_float); Serial.println("  ]");
	Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [  CURRENT_POS_steps ]    [   "); Serial.print(*currentAbsPosPseudo ); Serial.println("     ]");
	Serial.print("[   PSEUDO:"); Serial.print(pseudoID); Serial.print("   ]   [   CURRENT_DIR      ]    [   "); Serial.print(*currentDirStatusPseudo); Serial.println("     ]");
	Serial.println("-----------------------------------------------------------------------------------------------");

} // END FUNCTION: readEEPROMsettingsSlave

// =========================================================================================================== //

void PseudoSPIcommMetamorphicManipulator::statusLEDblink( int number_of_blinks, unsigned long blink_for_ms)
{
	for (int blink_counter = 0; blink_counter < number_of_blinks; blink_counter++)
	{
		 unsigned long time_now_millis = millis();

		digitalWrite(statusLED_Pin,HIGH);
		while(millis() < time_now_millis + blink_for_ms){}                 
		digitalWrite(statusLED_Pin,LOW);
	}
} // END FUNCTION: statusLEDblink

// =========================================================================================================== //

void PseudoSPIcommMetamorphicManipulator::txrxLEDSblink( int number_of_blinks, unsigned long blink_for_ms)
{
	for (int blink_counter = 0; blink_counter < number_of_blinks; blink_counter++)
	{
		 unsigned long time_now_millis = millis();

		digitalWrite(TXled_Pin,HIGH); digitalWrite(RXled_Pin,HIGH);
		while(millis() < time_now_millis + blink_for_ms){}                 
		digitalWrite(TXled_Pin,LOW); digitalWrite(RXled_Pin,LOW);
	}
} // END FUNCTION: txrxLEDSblink

// =========================================================================================================== //

void PseudoSPIcommMetamorphicManipulator::setupEEPROMslave( int newID, float max_angle_limit, float min_angle_limit, float pseudoStepAngle)
{
	/*
	 *  USER EXECUTES ONLY FOR ROBOT CONFIGURATION! 
	 *  NEVER USE INSIDE LOOP!
	 *  EVERY WRITE TAKES AN AVERAGE OF 3.3 moveSlave
	 *  EEPROM AREA HAS AN AVERAGE OF 100.000 cycles per single location
	 */

	// Set pseudojoint ID
	EEPROM.write(ID_EEPROM_ADDR, newID);

	// Set min/max angle limits
	EEPROM.put(MAX_POS_LIM_ADDR, max_angle_limit);
	EEPROM.put(MIN_POS_LIM_ADDR, min_angle_limit);

	// Set step angle (must oblije to joint mechanical design!)
	EEPROM.put(STEP_ANGLE_ADDR, pseudoStepAngle);

	// Set current state of pseudos
	EEPROM.write(CS_EEPROM_ADDR, STATE_LOCKED);

	// Set current position and direaction at setup
	EEPROM.write(CP_EEPROM_ADDR, 7);
	EEPROM.write(CD_EEPROM_ADDR, LOW);

} // END FUNCTION: setupEEPROMslave

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::setHomePositionMaster(int pseudoID, int ssPins[], volatile byte *CURRENT_STATE )
{
	/*
	 *  Orders slave to go to Home position
	 *  Returns true only if in position achieved
	 */	
	bool result;

	if ( *CURRENT_STATE == STATE_UNLOCKED )
	{
		unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
		unsigned long time_now_micros 			= micros();
		bool slave_responded_correct_flag		= false;

		digitalWrite(ssPins[pseudoID-1], LOW);								// enable Pseudo Slave Select pin

		do{

			*CURRENT_STATE = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_HOME, slave_response);


			if( (*CURRENT_STATE == IN_POSITION) )
			{
				slave_responded_correct_flag = true;
				result = true;
			}
			else
			{
				result = false;
			}

		//}while( (micros() < time_now_micros + wait_total_response_time_micros) && (!slave_responded_correct_flag) );
		}while( (!slave_responded_correct_flag) );

		digitalWrite(ssPins[pseudoID-1], HIGH);								// disable Pseudo Slave Select pin

	}
	else
	{
		result = false;
	}
	
	return result;
} // END FUNCTION: setHomePositionMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::setHomePositionSlave(volatile byte *CURRENT_STATE ,int *currentAbsPosPseudo, byte *currentAbsPosPseudo_ci, byte *currentDirStatusPseudo, volatile bool *homingHallActivated, volatile bool *limitHallActivated){

	bool result;

	byte MOTOR_DIRECTION = *currentDirStatusPseudo;
	int homing_calibration_steps;
	bool HOMING_PSEUDO = true;

	unsigned long homing_stepping_delay = 500;

	Serial.print("[   PSEUDO:"); Serial.print(_pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(HOMING); Serial.println("   ]");          

// commented since function is used only on demand 
	//digitalWrite(RELAY_lock_Pin, LOW);      							
	//digitalWrite(RELAY_lock_Pin2, LOW);

	if( (*CURRENT_STATE == STATE_UNLOCKED)  )
	{

		while (HOMING_PSEUDO)
		{                                                                        
			time_now_micros = micros();

			// Step motor
			digitalWrite(stepPin_NANO, HIGH);
			while(micros() < time_now_micros + homing_stepping_delay){}   	// wait approx. [μs]
			digitalWrite(stepPin_NANO, LOW);

	// next if's are commented since ISR is used
	///*
			if( digitalRead(pseudoLimitSwitch_Pin) == HIGH )
			{
				*limitHallActivated = true;
			}

			if( digitalRead(hallSwitch_Pin) == HIGH )
			{
				*homingHallActivated = true;
			}
	//*/
			if (*limitHallActivated)
			{
				Serial.println("MIN/MAX LIMIT HALL SENSOR ACTIVATED");

				// Change DIR Pin status
				digitalWrite(dirPin_NANO, !MOTOR_DIRECTION);
				MOTOR_DIRECTION = !MOTOR_DIRECTION;
				*limitHallActivated = false;
			}

			if (*homingHallActivated)
			{
				Serial.println("HOME HALL SENSOR ACTIVATED");

				for (size_t homing_calibration_steps = 0; homing_calibration_steps < HOMING_CALIBRATION_LIMIT; homing_calibration_steps++)
				{
					time_now_micros = micros();
					digitalWrite(stepPin_NANO, HIGH);
					while(micros() < time_now_micros + homing_stepping_delay){}   	// wait approx. [μs]
					digitalWrite(stepPin_NANO, LOW);
					Serial.println(homing_calibration_steps);
				}
				
				HOMING_PSEUDO = false;
				*homingHallActivated = false;
			}

		}

		//digitalWrite(RELAY_lock_Pin, HIGH);
		//digitalWrite(RELAY_lock_Pin2, HIGH);

		Serial.print("[   PSEUDO:"); Serial.print(_pseudoID); Serial.print("   ]   [   CURRENT STATUS:"); Serial.print(STATE_IN_POSITION_STRING); Serial.println("   ]");          

		*currentAbsPosPseudo 	= 0;
		*currentAbsPosPseudo_ci = 7; 										// for 15deg angle step only
		*CURRENT_STATE 			= IN_POSITION;
		*currentDirStatusPseudo = MOTOR_DIRECTION;

		EEPROM.update(CP_EEPROM_ADDR, *currentAbsPosPseudo_ci);
		Serial.println("[INFO]  SAVED	[	CP	] 	TO 	EEPROM");

		result = true;
	}
	else
	{
		result = false;
	}

	return result;
} // END FUNCTION: 

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::readCurrentAnatomyMaster(int pseudoID, int ssPins[], volatile byte *CURRENT_Ci, volatile byte *CURRENT_Ci_IDENTITY)
{
	/*
	 *  Reads for the current Ci of the pseudo connected in the ssPin specified
	 *  Returns true only if pseudo's Ci is successfully received
	 */	
	bool result;

	unsigned long slave_response 			= SLAVE_RESPONSE_TIME;  	// how much waits inside transfer function
	unsigned long time_now_micros 			= micros();
	bool slave_responded_correct_flag		= false;
	unsigned long eeprom_read_time_micros 	= 3500;  					// 3.5ms (3500μs) for read 

	digitalWrite(ssPins[pseudoID-1], LOW);				// enable Pseudo Slave Select pin

	do{
	
		//CURRENT_ANATOMY[pseudoID-1] = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_GIVE_CP, 5000);
		*CURRENT_Ci_IDENTITY = PseudoSPIcommMetamorphicManipulator::singleByteTransfer((byte) CMD_GIVE_CP, slave_response);

		switch (*CURRENT_Ci_IDENTITY)
		{
		case slave_is_at_c1:
			*CURRENT_Ci = c1;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c2:
			*CURRENT_Ci = c2;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c3:
			*CURRENT_Ci = c3;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c4:
			*CURRENT_Ci = c4;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c5:
			*CURRENT_Ci = c5;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c6:
			*CURRENT_Ci = c6;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c7:
			*CURRENT_Ci = c7;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c8:
			*CURRENT_Ci = c8;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c9:
			*CURRENT_Ci = c9;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c10:
			*CURRENT_Ci = c10;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c11:
			*CURRENT_Ci = c11;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c12:
			*CURRENT_Ci = c12;
			slave_responded_correct_flag = true;
			break;
		case slave_is_at_c13:
			*CURRENT_Ci = c13;
			slave_responded_correct_flag = true;
			break;

		default:
			*CURRENT_Ci = wrong_ci;
			//Serial.println("WRONG Ci inside readCurrentAnatomyMaster!");
			slave_responded_correct_flag = false;
			break;
		}

	}while( (!slave_responded_correct_flag) );

	digitalWrite(ssPins[pseudoID-1], HIGH);				// disable Pseudo Slave Select pin

	if( (*CURRENT_Ci >= c1)  && (*CURRENT_Ci  <= c13))
	{
		//Serial.print("ci given by slave inside readCurrentAnatomyMaster:"); Serial.println(*CURRENT_Ci);
		result = true;
	}
	else
	{
		result = false;
	}

	return result;	
} // END FUNCTION: readCurrentAnatomyMaster

// =========================================================================================================== //

bool PseudoSPIcommMetamorphicManipulator::readCurrentAnatomySlave(volatile byte *CURRENT_Ci, volatile byte *CURRENT_Ci_IDENTITY )
{
	/*
	 *  This is the response from readCurrentStateMaster.
	 *  Slave after each metamorphosis execution saves CS to EEPROM memory
	 *  Each time a meta mode starts/repeats reads the last state of slave
	 *  Reading CS only expects 3 appropriate returns!!! Otherwise errors!!!
	 */

	bool result;

	*CURRENT_Ci = EEPROM.read(CP_EEPROM_ADDR);
	Serial.print("I read this current ci from my eeprom:"); Serial.println(*CURRENT_Ci);

/*
	if( (*CURRENT_Ci >= c1)  && (*CURRENT_Ci <= c13))
	{
		result = true;
	}
	else
	{
		result = false;
	}
*/
	switch (*CURRENT_Ci)
	{
		case c1:
			*CURRENT_Ci_IDENTITY = slave_is_at_c1;
			result = true;
			break;
		case c2:
			*CURRENT_Ci_IDENTITY = slave_is_at_c2;
			result = true;
			break;
		case c3:
			*CURRENT_Ci_IDENTITY = slave_is_at_c3;
			result = true;
			break;
		case c4:
			*CURRENT_Ci_IDENTITY = slave_is_at_c4;
			result = true;
			break;
		case c5:
			*CURRENT_Ci_IDENTITY = slave_is_at_c5;
			result = true;
			break;
		case c6:
			*CURRENT_Ci_IDENTITY = slave_is_at_c6;
			result = true;
			break;
		case c7:
			*CURRENT_Ci_IDENTITY = slave_is_at_c7;
			result = true;
			break;
		case c8:
			*CURRENT_Ci_IDENTITY = slave_is_at_c8;
			result = true;
			break;
		case c9:
			*CURRENT_Ci_IDENTITY = slave_is_at_c9;
			result = true;
			break;
		case c10:
			*CURRENT_Ci_IDENTITY = slave_is_at_c10;
			result = true;
			break;
		case c11:
			*CURRENT_Ci_IDENTITY = slave_is_at_c11;
			result = true;
			break;
		case c12:
			*CURRENT_Ci_IDENTITY = slave_is_at_c12;
			result = true;
			break;
		case c13:
			*CURRENT_Ci_IDENTITY = slave_is_at_c13;
			result = true;
			break;

		default:
			*CURRENT_Ci_IDENTITY = IS_TALKING;
			result = false;
			break;
	}

	Serial.print("I sent this ci identity to my master:"); Serial.println(*CURRENT_Ci_IDENTITY);

	return result;
} // END FUNCTION: readCurrentAnatomySlave