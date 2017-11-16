#include <Arduino.h>

#include <MBWire.h>

int Loop_Delay = 500;

unsigned int Counter;

String I2C_Receive_Data;

int I2C_Address = 12;

int I2C_BUS_Responce;


void Broadcast(String Broadcast_String) {

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
    Serial.println(String("ERROR: ") + String(I2C_BUS_Responce)); // REMOVE ME
	}


} // End marker for Broadcast

void I2C_Receive(int HowMany) {

  while (Wire.available()) {
    I2C_Receive_Data += (char)Wire.read();
  }

} //End marker for I2C_Receive


void setup() {

	// Wire.pullup(true);

  Wire.begin(I2C_Address);
  TWAR = (I2C_Address << 1) | 1;
  Wire.onReceive(I2C_Receive);

  Serial.begin(115200);
  Serial.println("Boot Start");
  Serial.println("Boot Done");
	Broadcast("Mega2 Boot Done");
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
		Broadcast("Mega2: " + String(Counter));
	}



  delay(Loop_Delay);
}
