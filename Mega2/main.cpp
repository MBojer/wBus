#include <Arduino.h>

#include <WBus.h>

int Loop_Delay = 500;

unsigned int Counter;

String I2C_Receive_Data;

int I2C_Address = 11;

int I2C_BUS_Responce;




WBus wBus(I2C_Address, true, 20, true, 115200, Loop_Delay);


void I2C_Receive(int HowMany) {

  while (wBus.available()) {
    I2C_Receive_Data += (char)wBus.read();
  }

wBus.Queue_Push(I2C_Receive_Data, false);

} //End marker for I2C_Receive


void setup() {
  Serial.begin(115200);
  Serial.println("Boot Start");

  randomSeed(analogRead(0)); // CHANGE ME
  unsigned int Boot_Delay = random(0, 5000); // CHANGE ME
  Serial.println("Random start delay: " + String(Boot_Delay) + " milliseconds"); // CHANGE ME
  delay(Boot_Delay);
  delay(5000);

  wBus.Boot_Message();

  TWAR = (I2C_Address << 1) | 1;
  wBus.onReceive(I2C_Receive);

  wBus.Device_ID_Check();

  Serial.println("Boot Done");
  delay(1000);
}

void loop() {
  Counter++;
  // Serial.println("Counter: " + String(Counter));

  wBus.Device_ID_Check();

  if (I2C_Receive_Data != "") {
    Serial.println("Reviced: " + I2C_Receive_Data);
    I2C_Receive_Data = "";
  }

  if (Counter % 7 == 0) {
    wBus.broadcast("Mega2: " + String(Counter));
  }



  delay(Loop_Delay);
}
