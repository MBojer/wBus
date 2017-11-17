#include <Arduino.h>

#include <WBus.h>

int Loop_Delay = 500;

unsigned int Counter;

int I2C_Address = 11;

int I2C_BUS_Responce;




WBus wBus(I2C_Address, true, 20, true, 115200, Loop_Delay);

void Error_Mode_Local() {
  if (wBus.Device_ID_Check() == 2) {
    wBus.Blink_LED(5);
  }
}
String I2C_Receive_Data;

void I2C_Receive(int HowMany) {


  while (wBus.available()) {
    I2C_Receive_Data += (char)wBus.read();
  }

    Serial.println("Reviced: " + I2C_Receive_Data); // REMOVE ME

  // wBus.Queue_Push(I2C_Receive_Data, false);

//  I2C_Receive_Data = ""; // REMOVE ME
} //End marker for I2C_Receive


void setup() {
  Serial.begin(115200);
  Serial.println("Boot Start");

  randomSeed(analogRead(0)); // CHANGE ME
  unsigned int Boot_Delay = random(0, 5000); // CHANGE ME
  Serial.println("Random start delay: " + String(Boot_Delay) + " milliseconds"); // CHANGE ME
  delay(Boot_Delay);
  Serial.println("After delay");


  wBus.Boot_Message();

  TWAR = (I2C_Address << 1) | 1;
  wBus.onReceive(I2C_Receive);

  // wBus.Device_ID_Check();

  delay(10000); // CHANGE ME - Migh not be needed later

  Serial.println("Boot Done");
}

void loop() {
  Counter++;
  // Serial.println("Counter: " + String(Counter));


  if (I2C_Receive_Data == "11DD") {
    Serial.println("Marker - 11DD");
    wBus.broadcast("11DD");
    I2C_Receive_Data = "";
  }

  else {
    I2C_Receive_Data = "";
  }




  if (Counter == 30) {
    wBus.broadcast("11DD");
  }



  if (Counter % 7 == 0) {
    wBus.broadcast("Mega2: " + String(Counter));
  }



  delay(Loop_Delay);
}
