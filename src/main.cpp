#include <Arduino.h>

#include <WBus.h>

int Loop_Delay = 500;

unsigned int Counter;

String I2C_Receive_Data;

int I2C_Address = 11;

int I2C_BUS_Responce;

WBus wBus(I2C_Address, true, 20, true, 115200);


void I2C_Receive(int HowMany) {

  while (wBus.available()) {
    I2C_Receive_Data += (char)wBus.read();
  }

} //End marker for I2C_Receive


void setup() {
	Serial.begin(115200);
	Serial.println("Boot Start");

	wBus.Boot_Message();

	// wBus.Device_ID_Check();


	TWAR = (I2C_Address << 1) | 1;
	wBus.onReceive(I2C_Receive);


  Serial.println("Boot Done");
	wBus.broadcast("Mega1 Boot Done");
	delay(1000);
	}

void loop() {
  Counter++;
  Serial.println("Counter: " + String(Counter));

  if (I2C_Receive_Data != "") {
    Serial.println("Reviced: " + I2C_Receive_Data);
		I2C_Receive_Data = "";
  }

	if (Counter % 7 == 0) {
		wBus.broadcast("Mega1: " + String(Counter));
	}



  delay(Loop_Delay);
}
