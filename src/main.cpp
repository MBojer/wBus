#include <Arduino.h>

#include <WBus.h>

int Loop_Delay = 500;

unsigned int Counter;

String I2C_Receive_Data;

int I2C_Address = 11;

int I2C_BUS_Responce;

WBus wBus(I2C_Address, 20, true, 115200);


void Broadcast(String Broadcast_String) {

	/* ------------------------------ Broadcast ------------------------------

	Version 0.1

	Broadcasts information to the I2C network

	*/

	int I2C_BUS_Responce;

	wBus.beginTransmission (0);  // broadcast to all
	wBus.write(Broadcast_String.c_str());
	I2C_BUS_Responce = wBus.endTransmission();

	if (I2C_BUS_Responce == 0) { // REMOVE ME
		Serial.println(String("Send: ") + String(Broadcast_String)); // REMOVE ME
	} // REMOVE ME


	if (I2C_BUS_Responce != 0) {
    Serial.println(String("ERROR: ") + String(I2C_BUS_Responce)); // REMOVE ME
	}


} // End marker for Broadcast

void I2C_Receive(int HowMany) {

  while (wBus.available()) {
    I2C_Receive_Data += (char)wBus.read();
  }

} //End marker for I2C_Receive


void setup() {
	Serial.begin(115200);
	Serial.println("Boot Start");

	// wBus.Device_ID_Check();


	wBus.begin(I2C_Address);
	wBus.pullup(true);
	TWAR = (I2C_Address << 1) | 1;
	wBus.onReceive(I2C_Receive);


  Serial.println("Boot Done");
	Broadcast("Mega1 Boot Done");
	delay(1000);
	Broadcast("DD");
}

void loop() {
  Counter++;
  Serial.println("Counter: " + String(Counter));


  if (I2C_Receive_Data == "DD") {
    Broadcast("DX");
    Serial.println("DX");
    I2C_Receive_Data = "";
  }

  else if (I2C_Receive_Data != "") {
    Serial.println("Reviced: " + I2C_Receive_Data);
		I2C_Receive_Data = "";
  }

	if (Counter % 7 == 0) {
		Broadcast("Mega1: " + String(Counter));
	}



  delay(Loop_Delay);
}
