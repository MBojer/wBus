/*
Version 0.2
*/

// --------------------------------------------- From MBWire ---------------------------------------------

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include "twi.h"
}



// --------------------------------------------- WBus ---------------------------------------------

#include "Arduino.h"
#include "WBus.h"

// --------------------------------------------- WBus ---------------------------------------------

WBus::WBus(int I2C_Device_ID, int Max_Queue_Length, bool Log_To_Serial, long Serial_Speed) {

	_Log_To_Serial = Log_To_Serial;
	_Serial_Speed = Serial_Speed;

	_Max_Queue_Length = Max_Queue_Length;

	_Device_ID = I2C_Device_ID;

} // End Marker for WBus

// Initialize Class Variables //////////////////////////////////////////////////

uint8_t WBus::rxBuffer[BUFFER_LENGTH];
uint8_t WBus::rxBufferIndex = 0;
uint8_t WBus::rxBufferLength = 0;

uint8_t WBus::txAddress = 0;
uint8_t WBus::txBuffer[BUFFER_LENGTH];
uint8_t WBus::txBufferIndex = 0;
uint8_t WBus::txBufferLength = 0;

uint8_t WBus::transmitting = 0;
void (*WBus::user_onRequest)(void);
void (*WBus::user_onReceive)(int);


// --------------------------------------------- From MBWire ---------------------------------------------

// Public Methods //////////////////////////////////////////////////////////////

void WBus::begin(void)
{
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  twi_init();
}

void WBus::begin(uint8_t address)
{
  twi_setAddress(address);
  twi_attachSlaveTxEvent(onRequestService);
  twi_attachSlaveRxEvent(onReceiveService);
  begin();
}

void WBus::begin(int address)
{
  begin((uint8_t)address);
}

uint8_t WBus::requestFrom(uint8_t address, uint8_t quantity)
{
  // clamp to buffer length
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  // perform blocking read into buffer
  uint8_t read = twi_readFrom(address, rxBuffer, quantity);
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}

uint8_t WBus::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity);
}

void WBus::beginTransmission(uint8_t address)
{
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void WBus::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

uint8_t WBus::endTransmission(void)
{
  // transmit buffer (blocking)
  int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t WBus::write(uint8_t data)
{
  if(transmitting){
  // in master transmitter mode
    // don't bother if buffer is full
    if(txBufferLength >= BUFFER_LENGTH){
      setWriteError();
      return 0;
    }
    // put byte in tx buffer
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // update amount in buffer
    txBufferLength = txBufferIndex;
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(&data, 1);
  }
  return 1;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t WBus::write(const uint8_t *data, size_t quantity)
{
  if(transmitting){
  // in master transmitter mode
    for(size_t i = 0; i < quantity; ++i){
      write(data[i]);
    }
  }else{
  // in slave send mode
    // reply to master
    twi_transmit(data, quantity);
  }
  return quantity;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int WBus::available(void)
{
  return rxBufferLength - rxBufferIndex;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int WBus::read(void)
{
  int value = -1;

  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// must be called in:
// slave rx event callback
// or after requestFrom(address, numBytes)
int WBus::peek(void)
{
  int value = -1;

  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void WBus::flush(void)
{
  // XXX: to be implemented.
}

// behind the scenes function that is called when data is received
void WBus::onReceiveService(uint8_t* inBytes, int numBytes)
{
  // don't bother if user hasn't registered a callback
  if(!user_onReceive){
    return;
  }
  // don't bother if rx buffer is in use by a master requestFrom() op
  // i know this drops data, but it allows for slight stupidity
  // meaning, they may not have read all the master requestFrom() data yet
  if(rxBufferIndex < rxBufferLength){
    return;
  }
  // copy twi rx buffer into local read buffer
  // this enables new reads to happen in parallel
  for(uint8_t i = 0; i < numBytes; ++i){
    rxBuffer[i] = inBytes[i];
  }
  // set rx iterator vars
  rxBufferIndex = 0;
  rxBufferLength = numBytes;
  // alert user program
  user_onReceive(numBytes);
}

// behind the scenes function that is called when data is requested
void WBus::onRequestService(void)
{
  // don't bother if user hasn't registered a callback
  if(!user_onRequest){
    return;
  }
  // reset tx buffer iterator vars
  // !!! this will kill any pending pre-master sendTo() activity
  txBufferIndex = 0;
  txBufferLength = 0;
  // alert user program
  user_onRequest();
}

// sets function called on slave write
void WBus::onReceive( void (*function)(int) )
{
  user_onReceive = function;
}

// sets function called on slave read
void WBus::onRequest( void (*function)(void) )
{
  user_onRequest = function;
}

void WBus::pullup(bool Activate) {
  if (Activate == true) {
    digitalWrite(SDA, 1);
    digitalWrite(SCL, 1);
  }
  else {
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 0);
  }
}


// --------------------------------------------- WBus ---------------------------------------------


void WBus::Boot_Message() {

	if (_Log_To_Serial == true) {

		if (_Serial_Speed != 0) {
			Serial.begin(_Serial_Speed);
		}

		Serial.println("");
		Serial.println("Including WBus v0.2");

	}

} // End marker for Boot_Message


void WBus::I2C_BUS_Error(int Error_Number) {

	if (Error_Number == 1) { // represents an Error not useing the address just to be safe
		Serial.println("I2C Error 1: Data too long to fit in transmit buffer");
	}

	else if (Error_Number == 2) { // represents an Error not useing the address just to be safe
		Serial.println("I2C Error 2: Received NACK on transmit of address");
	}

	else if (Error_Number == 3) { // represents an Error not useing the address just to be safe
		Serial.println("I2C Error 3: Received NACK on transmit of data");
	}

	else if (Error_Number == 4) { // represents an Error not useing the address just to be safe
		Serial.println(String("I2C BUS error on address: ") + String(_Device_ID));
	}

	else if (Error_Number == 5) { // represents an Error not useing the address just to be safe
		Serial.println("I2C Error Mode Acrive: I2C Bus - Timeout while trying to become Bus Master");
	}

	else if (Error_Number == 6) { // represents an Error not useing the address just to be safe
		Serial.println("I2C Error Mode Acrive: I2C Bus - Timeout while waiting for data to be sent");
	}

	else { // Above 6 is reserved for later errors
		Serial.println("I2C Error Mode Acrive: Error number " + Error_Number);
	}

	return;

} // End Marker - I2C_Error_Print


int WBus::Device_ID_Check() {

	String test_string = "1337"; // CHANGE ME
	String I2C_BUS_Responce;

	begin(100);

	beginTransmission(100);  // broadcast to all
	write(test_string.c_str());
	I2C_BUS_Responce = endTransmission();

	if (I2C_BUS_Responce != "") { // CHANGE ME
		Serial.println(I2C_BUS_Responce);
	}

	return 1;
  //
	// if (_Device_ID_Check_OK == 0) { // Check not done
  //
	// 	if (_Device_ID < 10 || _Device_ID > 99) {
	// 		Serial.println("ERROR: Devide ID not between 10 and 99");
	// 		_Device_ID_Check_OK = 2; // 2 = Error
	// 		return 2; // 2 = Error
	// 	}
  //
	// 	Wire.begin();
  //
	// 	int I2C_BUS_Responce;
  //
	// 	Wire.beginTransmission(_Device_ID);
	// 	I2C_BUS_Responce = Wire.endTransmission();
  //
	// 	Serial.println(I2C_BUS_Responce); // REMOVE ME
  //
	// 	if (I2C_BUS_Responce != 0) {
	// 		Serial.println("ERROR: Devide ID in use");
	// 		_Device_ID_Check_OK = 2; // 2 = Error
	// 		Serial.println("Marker");
	// 		return 2; // 2 = Error
	// 	}
  //
	// 	_Device_ID_Check_OK = 1; // 1 = Check done and passed
	// 	return 1; // 1 = Check done and passed
	// }
  //
	// else if (_Device_ID_Check_OK == 1) { // Check done and passed
	// 	return 1; // 1 = Check done and passed
	// }
  //
	// return 2; // 2 = Error

} // END MARKER - Device_ID



void WBus::Broadcast(String Broadcast_String) {

	/* ------------------------------ Broadcast ------------------------------
	Version 0.1
	Broadcasts information to the I2C network
	*/

	int I2C_BUS_Responce;

	beginTransmission (0);  // broadcast to all
	write(Broadcast_String.c_str());
	I2C_BUS_Responce = endTransmission();

	if (I2C_BUS_Responce == 0) { // REMOVE ME
		Serial.println(String("Send: ") + String(Broadcast_String)); // REMOVE ME
	} // REMOVE ME


	if (I2C_BUS_Responce != 0) {
		I2C_BUS_Error(I2C_BUS_Responce);
	}


} // End marker for Broadcast



void WBus::Queue_Push(String Push_String, bool Add_To_Front_Of_Queue) {

	if (_Queue_Is_Empthy == true) {
		_Queue_String = Push_String + ";";
		_Queue_Length = 1;
		_Queue_Is_Empthy = false;
	}


	else {
		_Queue_String = _Queue_String + Push_String + ";";
		_Queue_Length++;
		_Queue_Is_Empthy = false;
	}

	if (_Queue_Length >= _Max_Queue_Length) {
		Serial.println("ERROR: Max_Queue_Length reached, clearing command queue"); // CHANGE ME - Add I2C ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		_Queue_String = ";";
		_Queue_Length = 0;
		_Queue_Is_Empthy = true;
		// CHANGE ME - Add I2C ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
	}

} // End Marker for Push



String WBus::Queue_Pop() {

			String Pop_String;


			if (_Queue_Is_Empthy == true) {
				return ";";
			}


			else {

				Pop_String = _Queue_String.substring(0, _Queue_String.indexOf(";"));

				_Queue_String.remove(0, _Queue_String.indexOf(";") + 1);

				_Queue_Length--;


				if (_Queue_String.length() <= 3 || _Queue_Length < 1) {
					_Queue_String = ";";
					_Queue_Length = 0;
					_Queue_Is_Empthy = true;
				}


				return Pop_String;

			} // End Marker for Else

} // End Marker for POP



String WBus::Queue_Peek() {

			if (_Queue_String == ";") {
				return ";";
			}

			else {
				return _Queue_String.substring(0, _Queue_String.indexOf(";"));
			}

}// End Marker for Peek



String WBus::Queue_Peek_Queue() {
			return _Queue_String;
}



int WBus::Queue_Length() {
			return _Queue_Length;
}



int WBus::Queue_Is_Empthy() {
			return _Queue_Is_Empthy;
}



void WBus::Queue_Clear() {
			_Queue_String = ";";
			_Queue_Length = 0;
			_Queue_Is_Empthy = true;
}



String WBus::Queue_Search_Peek(String Search_String) {

			if (_Queue_String.indexOf(Search_String) >= 0) {

				String Search_Peek_String = _Queue_String.substring(0, _Queue_String.indexOf(Search_String) + Search_String.length());

				if (Search_Peek_String.indexOf(";") <= 0) {
					return Search_Peek_String;
				}

				else {
					Search_Peek_String = Search_Peek_String.substring(Search_Peek_String.lastIndexOf(";") + 1, Search_Peek_String.length());
					return Search_Peek_String;
				}
			}

			return ";";
}



String WBus::Queue_Search_Pop(String Search_String, bool Delete_All_Matches) {
			if (_Queue_String.indexOf(Search_String) >= 0) {

				String Search_Pop_String = _Queue_String.substring(0, _Queue_String.indexOf(Search_String) + Search_String.length());


				if (Search_Pop_String.indexOf(";") <= 0) {

					_Queue_String = _Queue_String.substring(Search_Pop_String.length() + 1);
				}


				else {
					Search_Pop_String = Search_Pop_String.substring(Search_Pop_String.lastIndexOf(";") + 1, Search_Pop_String.length());

					_Queue_String = _Queue_String.substring(0, _Queue_String.indexOf(Search_Pop_String)) + _Queue_String.substring(_Queue_String.indexOf(Search_Pop_String) + Search_Pop_String.length() + 1 , _Queue_String.length());
				}

				_Queue_Length--;


				if (Delete_All_Matches == true) {
					_Queue_String.replace(Search_Pop_String + ";", "");

					_Queue_Length = _Queue_String.indexOf(";") + 1;
				}


				if (_Queue_String.length() <= 3 || _Queue_Length < 1) {
					_Queue_String = ";";
					_Queue_Length = 0;
					_Queue_Is_Empthy = true;
				}

				return Search_Pop_String;

			}

			return ";";
}



void WBus::Error_Blink(int Number_Of_Blinks) {

			/* ------------------------------ Blink ------------------------------
			Version 0.1
			Blinks :-P
			*/

			pinMode(LED_BUILTIN, OUTPUT);

			int Blink_Timer = 500;
			int Blink_Timer_Marker = 75;

			for (int x = 1; x < Number_Of_Blinks + 1; x++) {

				digitalWrite(LED_BUILTIN, HIGH);
				delay(Blink_Timer);
				digitalWrite(LED_BUILTIN, LOW);
				delay(Blink_Timer);
			}

			delay(Blink_Timer);

			for (int x = 1; x < 4; x++) {

				digitalWrite(LED_BUILTIN, HIGH);
				delay(Blink_Timer_Marker);
				digitalWrite(LED_BUILTIN, LOW);
				if (x < 3) { // Done to make the Serial activity blink apear as the 3rd end or error blink
					delay(Blink_Timer_Marker);
				}
			}


}
