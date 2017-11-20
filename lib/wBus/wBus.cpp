/*

Version 0.1
NOTE: Remember to change version number in Boot_Message

*/

// --------------------------------------------- For TWI (Two Wire Interface) ---------------------------------------------
extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
  #include "twi.h"
}

// --------------------------------------------- WBus ---------------------------------------------

#include "Arduino.h"
#include "WBus.h"

#include "MemoryFree.h" // REMOVE ME


// --------------------------------------------- Setup ---------------------------------------------

WBus::WBus(int I2C_Device_ID, bool I2C_Internal_Pullup, int Max_Queue_Length, bool Log_To_Serial, long Serial_Speed) {

  _Device_ID = I2C_Device_ID;
  _I2C_Internal_Pullup = I2C_Internal_Pullup;

  _Max_Queue_Length = Max_Queue_Length;

  _Log_To_Serial = Log_To_Serial;
  _Serial_Speed = Serial_Speed;

  pullup(_I2C_Internal_Pullup); // Sets the PullUp Resistors

  begin(_Device_ID);


} // End Marker for WBus


// --------------------------------------------- Two Wire Interface ---------------------------------------------

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

void WBus::begin(void) {
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  twi_init();
}

void WBus::begin(uint8_t address) {
  twi_setAddress(address);
  twi_attachSlaveTxEvent(onRequestService);
  twi_attachSlaveRxEvent(onReceiveService);
  begin();
}

void WBus::begin(int address) {
  begin((uint8_t)address);
}

void WBus::beginTransmission(uint8_t address) {
  // indicate that we are transmitting
  transmitting = 1;
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

void WBus::beginTransmission(int address) {
  beginTransmission((uint8_t)address);
}

int WBus::broadcast(String Broadcast_String) {

  /* ------------------------------ Broadcast ------------------------------
  Version 0.1
  Broadcasts information to the I2C network
  */

  int I2C_BUS_Responce;

  for (int x = 0; x < 3; x++) {

    beginTransmission(0);  // broadcast to all
    write(Broadcast_String.c_str());
    I2C_BUS_Responce = endTransmission();

    if (I2C_BUS_Responce == 0) { // REMOVE ME
      Serial.println(String("Send: ") + String(Broadcast_String)); // REMOVE ME
      return 0;
    } // REMOVE ME


    else if (I2C_BUS_Responce != 0) { // Returns if 0  -  0 = No error
      if (_Log_To_Serial == true) {
        Serial.println(I2C_BUS_Error_To_Text(I2C_BUS_Responce));
      }
      return I2C_BUS_Responce;
    }

    else { // REMOVE ME
      Serial.println("Retransmitting: " + Broadcast_String); // REMOVE ME
    } // REMOVE ME

    Serial.println("x: " + String(x));
    delay(250);
  }

  return 0;
} // End marker for Broadcast

uint8_t WBus::endTransmission(void) {
  // transmit buffer (blocking)
  int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;
  return ret;
}

uint8_t WBus::requestFrom(uint8_t address, uint8_t quantity) {
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

uint8_t WBus::requestFrom(int address, int quantity) {
  return requestFrom((uint8_t)address, (uint8_t)quantity);
}

size_t WBus::write(uint8_t data) {
  // must be called in:
  // slave tx event callback
  // or after beginTransmission(address)
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

size_t WBus::write(const uint8_t *data, size_t quantity) {
  // must be called in:
  // slave tx event callback
  // or after beginTransmission(address)
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

int WBus::available(void) {
  // must be called in:
  // slave rx event callback
  // or after requestFrom(address, numBytes)
  return rxBufferLength - rxBufferIndex;
}

int WBus::read(void) {
  int value = -1;

  // get each successive byte on each call
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

int WBus::peek(void) {
  // must be called in:
  // slave rx event callback
  // or after requestFrom(address, numBytes)
  int value = -1;

  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void WBus::flush(void) {
  // XXX: to be implemented.
}

void WBus::onReceive( void (*function)(int) ) {
  // sets function called on slave write
  user_onReceive = function;
}

void WBus::onReceiveService(uint8_t* inBytes, int numBytes) {
  // behind the scenes function that is called when data is received
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

void WBus::onRequest( void (*function)(void) ) {
  // sets function called on slave read
  user_onRequest = function;
}

void WBus::onRequestService(void) {
  // behind the scenes function that is called when data is requested
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

void WBus::pullup(bool Activate) { // Enable/Disable Internal PullUp Resistors
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

int WBus::Device_ID_Check() {

  /*
  0 = Not Done
  1 = Check done and passed
  2 = Error
  3 = Waiting for reply

  */


  if (_Device_ID_Check_OK == 0) { // 0 = Not done
    _Device_ID_Check_Millis_Start_At = millis() + _Device_ID_Check_Millis_Interval;
    if (broadcast(String(_Device_ID) + "DD") != 0) {
      _Device_ID_Check_Error_Counter--;
    }
    _Device_ID_Check_Checks_Left--;
    _Device_ID_Check_OK = 3;
    return 3;
  }

  else if (_Device_ID_Check_OK == 1) { // 1 = Done and OK

    if (_Queue_Device_ID_Check_Hit == true) {
      /*
      Command_Queue_Push have put (Device_ID)"DD" (Device ID check request) in the Queue.

      Checking if the device id matches this units Device ID and if so replied (Device_ID)"DD"

      (Device_ID)"DD" will make the other device enter error state and change ID to "110"
      */

      if (Queue_Search_Pop((String(_Device_ID) + "DD"), true) != ";") { // Another device send duplicate Device ID
        if (broadcast(String(_Device_ID) + "DD") != 0) {
          _Device_ID_Check_Error_Counter--;
        }

        // CHANGE ME - ADD I2C Error Braodcast on duplicate hit
      }

      else { // Clears the Queue of DD
        Serial.println("Clear queue"); // REMOVE ME
        Queue_Search_Pop("DD", true); // Clears the queue for device id check requests
      } // Clear queue for Device ID requests

      _Queue_Device_ID_Check_Hit = false;
      return 1;
    } // END MARKER - if (_Queue_Device_ID_Check_Hit == true)

    return 1;
  } // END MARKER - else if (_Device_ID_Check_OK == 1)

  else if (_Device_ID_Check_OK == 2) { // 2 = Failed
    return 2;
  }

  else { // 3 = Waiting for reply

    if (_Queue_Device_ID_Check_Hit == true) { // Queue_Push added "DD" to the queue, "DD" Symbolizes and Device_ID check

      if (Queue_Search_Pop((String(_Device_ID) + "DD"), true) != ";") { // Device ID Check failed going to error state
        _Device_ID_Check_OK = 2;
        _I2C_Bus_Error = 3; // if _I2C_Bus_Error = 1 Queue_Push will not add any data
        Queue_Clear();
        Serial.println("ERROR: Duplicate Device ID found, going into Error Mode");
        // begin(110); // CHAMGE ME - working here
        // Serial.println("Changinb to BUS address 110"); // CHAMGE ME
        return 2;
      }

    } // END MARKER - if (_Queue_Device_ID_Check_Hit == true)

    else if (_Device_ID_Check_Checks_Left <= 0) { // Done no reply on "DD" assuming device id unique

      if (_Device_ID_Check_Error_Counter <= 0) {
        _Device_ID_Check_OK = 2;
        _I2C_Bus_Error = 2; // if _I2C_Bus_Error = 1 Queue_Push will not add any data
        Queue_Clear();
        Serial.println("ERROR: All broadcasts failed, assuming I2C bus is down, entering error mode");
        return 2;
      }

      Serial.println("Device ID: " + String(_Device_ID) + " Check Compleate, ID not in use");
      _Device_ID_Check_OK = 1;
      Queue_Search_Pop("DD", true); // Clears the queue for any DD messages
      return 1;
    }

    else { // No reply broadcasting device ID again

      if (_Device_ID_Check_Millis_Start_At <= millis()) {
        _Device_ID_Check_Millis_Start_At = millis() + _Device_ID_Check_Millis_Interval;
        if (broadcast(String(_Device_ID) + "DD") != 0) {
          _Device_ID_Check_Error_Counter--;
        }
        _Device_ID_Check_Checks_Left--;
      }

      return 3;
      }

  } // END MARKER - else

} // END MARKER - Device_ID

int WBus::I2C_BUS_Error() {
  return _I2C_Bus_Error;
}

String WBus::I2C_BUS_Error_To_Text(int Error_Number) {

  String Error_Text;

  if (Error_Number == 1) { // represents an Error not useing the address just to be safe
    Error_Text = "I2C Error 1: Data too long to fit in transmit buffer";
  }

  else if (Error_Number == 2) { // represents an Error not useing the address just to be safe
    Error_Text = "I2C Error 2: Received NACK on transmit of address";
  }

  else if (Error_Number == 3) { // represents an Error not useing the address just to be safe
    Error_Text = "I2C Error 3: Received NACK on transmit of data";
  }

  else if (Error_Number == 4) { // represents an Error not useing the address just to be safe
    Error_Text = "I2C BUS error on address: " + String(_Device_ID);
  }

  else if (Error_Number == 5) { // represents an Error not useing the address just to be safe
    Error_Text = "I2C Error Mode Acrive: I2C Bus - Timeout while trying to become Bus Master";
  }

  else if (Error_Number == 6) { // represents an Error not useing the address just to be safe
    Error_Text = "I2C Error Mode Acrive: I2C Bus - Timeout while waiting for data to be sent";
  }

  else { // Above 6 is reserved for later errors
    Error_Text = "I2C Error Mode Acrive: Error number " + Error_Number;
  }

  if (Blink_LED(true) == 0) { // Blinks the Error LED once when one of the abve mentione errors occures
    Blink_LED_Start(1);
  }

  if (_Log_To_Serial == true) {
    Serial.println(Error_Text);
  }

  return Error_Text;

} // End Marker - I2C_Error_Print




// --------------------------------------------- Blink LED ---------------------------------------------
/*
Version 0.1
Blinks :-P
*/

int WBus::Blink_LED(bool Read_Value_Only) { // Blinks the onboard LED to indicate errors

  if (Read_Value_Only == true) {
    return _Blink_LED_Blinks_Left;
  }

  if (_Blink_LED_Blinks_Left == 0) {
      return 0;
  } // END MARKER - if (_Blink_LED_Blinks_Left == 0)

  if (_Blink_LED_Millis_Start_At <= millis()) {

    if (digitalRead(_Blink_LED_Pin) == LOW) { // LED ON
      _Blink_LED_Millis_Start_At = millis() + _Blink_LED_Millis_Interval;
      digitalWrite(_Blink_LED_Pin, HIGH);
      return _Blink_LED_Blinks_Left;
    }

    else { // LED OFF
      _Blink_LED_Millis_Start_At = millis() + _Blink_LED_Millis_Interval;
      digitalWrite(_Blink_LED_Pin, LOW);

      _Blink_LED_Blinks_Left--;

      if (_Blink_LED_Blinks_Left == 0) {
        _Blink_LED_Millis_Start_At = millis() + _Blink_LED_Millis_Interval_Break;
      }

      return _Blink_LED_Blinks_Left;
    }
  } // END MARKER - if (_Blink_LED_Millis_Start_At <= millis()) {

  return _Blink_LED_Blinks_Left;
} // END MARKER - Blink_LED(bool Read_Value_Only)

void WBus::Blink_LED_Start(int Number_Of_Blinks) {
  Blink_LED_Start(Number_Of_Blinks, LED_BUILTIN);
} // END MARKER - Blink_LED_Start

void WBus::Blink_LED_Start(int Number_Of_Blinks, int LED_Pin) {

  if (_Blink_LED_Millis_Start_At >= millis()) { // Dooing nothing, waiting on time between blinks to pass
    return;
  }

  digitalWrite(_Blink_LED_Pin, LOW);

  _Blink_LED_Pin = LED_Pin;

  _Blink_LED_Blinks_Left = Number_Of_Blinks;

  _Blink_LED_Millis_Start_At = millis();

  pinMode(_Blink_LED_Pin, OUTPUT);

  Blink_LED(false);

} // END MARKER - Blink_LED_Start

void WBus::Blink_LED_Stop() {
  _Blink_LED_Blinks_Left = 0;
  digitalWrite(_Blink_LED_Pin, LOW);
} // END MARKER - Blink_LED_Stop



// --------------------------------------------- Queue ---------------------------------------------

void WBus::Queue_Push(String Push_String, bool Add_To_Front_Of_Queue) {

  if (_I2C_Bus_Error != 0) { // Error on I2C bus disabling Queue_Push
    return;
  }

  if (Push_String.indexOf((String(_Device_ID) + "DD")) <= 0) {
    _Queue_Device_ID_Check_Hit = true;
  }

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
    _Queue_Device_ID_Check_Hit = false;
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

} // End Marker for Peek

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



// --------------------------------------------- MISC ---------------------------------------------

void WBus::Boot_Message() { // Displays a boot message if included

  if (_Log_To_Serial == true && (Serial)) {

    if (_Serial_Speed != 0) {
      Serial.begin(_Serial_Speed);
    }

    Serial.println("");
    Serial.println("Including wBus v0.1");
    Serial.println("Using BUS Address: " + String(_Device_ID));
  }
} // End marker for Boot_Message
