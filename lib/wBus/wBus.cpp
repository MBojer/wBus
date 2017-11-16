/*
Version 0.2
*/

#include "Arduino.h"
#include "wBus.h"

#include <MBWire.h>


wBus::wBus(int I2C_Device_ID, int Max_Queue_Length, bool Log_To_Serial, long Serial_Speed) {

	_Log_To_Serial = Log_To_Serial;
	_Serial_Speed = Serial_Speed;

	_Max_Queue_Length = Max_Queue_Length;

	_Device_ID = I2C_Device_ID;

} // End Marker for wBus



void wBus::Boot_Message() {

	if (_Log_To_Serial == true) {

		if (_Serial_Speed != 0) {
			Serial.begin(_Serial_Speed);
		}

		Serial.println("");
		Serial.println("Including wBus v0.2");

	}

} // End marker for Boot_Message


void wBus::I2C_BUS_Error(int Error_Number) {

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


int wBus::Device_ID_Check() {

	if (_Device_ID_Check_OK == 0) { // Check not done

		if (_Device_ID < 10 || _Device_ID > 99) {
			Serial.println("ERROR: Devide ID not between 10 and 99");
			_Device_ID_Check_OK = 2; // 2 = Error
			return 2; // 2 = Error
		}

		Wire.begin();

		int I2C_BUS_Responce;

		Wire.beginTransmission(_Device_ID);
		I2C_BUS_Responce = Wire.endTransmission();

		Serial.println(I2C_BUS_Responce); // REMOVE ME

		if (I2C_BUS_Responce != 0) {
			Serial.println("ERROR: Devide ID in use");
			_Device_ID_Check_OK = 2; // 2 = Error
			Serial.println("Marker");
			return 2; // 2 = Error
		}

		_Device_ID_Check_OK = 1; // 1 = Check done and passed
		return 1; // 1 = Check done and passed
	}

	else if (_Device_ID_Check_OK == 1) { // Check done and passed
		return 1; // 1 = Check done and passed
	}

	return 2; // 2 = Error

} // END MARKER - Device_ID



void wBus::Broadcast(String Broadcast_String) {

	/* ------------------------------ Broadcast ------------------------------
	Version 0.1
	Broadcasts information to the I2C network
	*/

	int I2C_BUS_Responce;

	Wire.beginTransmission (0);  // broadcast to all
	Wire.write(Broadcast_String.c_str());
	I2C_BUS_Responce = Wire.endTransmission();

	if (I2C_BUS_Responce == 0) { // REMOVE ME
		Serial.println(String("Send: ") + String(Broadcast_String)); // REMOVE ME
	} // REMOVE ME


	if (I2C_BUS_Responce != 0) {
		I2C_BUS_Error(I2C_BUS_Responce);
	}


} // End marker for Broadcast



void wBus::Queue_Push(String Push_String, bool Add_To_Front_Of_Queue) {

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



String wBus::Queue_Pop() {

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



String wBus::Queue_Peek() {

			if (_Queue_String == ";") {
				return ";";
			}

			else {
				return _Queue_String.substring(0, _Queue_String.indexOf(";"));
			}

}// End Marker for Peek



String wBus::Queue_Peek_Queue() {
			return _Queue_String;
}



int wBus::Queue_Length() {
			return _Queue_Length;
}



int wBus::Queue_Is_Empthy() {
			return _Queue_Is_Empthy;
}



void wBus::Queue_Clear() {
			_Queue_String = ";";
			_Queue_Length = 0;
			_Queue_Is_Empthy = true;
}



String wBus::Queue_Search_Peek(String Search_String) {

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



String wBus::Queue_Search_Pop(String Search_String, bool Delete_All_Matches) {
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



void wBus::Error_Blink(int Number_Of_Blinks) {

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
