

#include <Arduino.h>

#include <WBus.h>

#include <MemoryFree.h> // REMOVE ME

String I2C_Receive_Data;

int I2C_Address = 11;


WBus wBus(I2C_Address, true, 20, true, 115200);


void I2C_Receive(int HowMany) {

  while (wBus.available()) {
    I2C_Receive_Data += (char)wBus.read();
  }

  wBus.Queue_Push(I2C_Receive_Data, false);

} //End marker for I2C_Receive


void Error_Mode(void) {
  if (wBus.Blink_LED(true) == 0) {
    wBus.Blink_LED_Start(wBus.Blink_LED_Number_Of_Blinks());
  }
} // END MARKER - Error Mode



void setup() {
  Serial.begin(115200);
  Serial.println("Boot Start");
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());


  wBus.Boot_Message();

  TWAR = (I2C_Address << 1) | 1;
  wBus.onReceive(I2C_Receive);


  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
  Serial.println("Boot Done");
}

void loop() {
  wBus.Device_ID_Check();

  if (String(millis()).indexOf("000") <= 0) {
    Serial.print("freeMemory()=");
    Serial.println(freeMemory());
  }

}
