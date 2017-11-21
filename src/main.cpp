#include <Arduino.h>

#include <WBus.h>

#include <MemoryFree.h> // REMOVE ME

int I2C_Address = 12;

unsigned long Message_Trigger_At = 1500;

WBus wBus(I2C_Address, true, 20, true, 115200);

void I2C_Receive(int HowMany) {
  String I2C_Receive_Data;

  while (wBus.available()) {
    I2C_Receive_Data += (char)wBus.read();
  }

  wBus.Queue_Push(I2C_Receive_Data, false);

  Serial.print("Resived: "); // REMOVE ME
  Serial.println(I2C_Receive_Data); // REMOVE ME

} //End marker for I2C_Receive


void Error_Mode(void) {

  wBus.zzzZZZ();

  if (wBus.Blink_LED(true) == 0) {
    wBus.Blink_LED_Start(wBus.I2C_BUS_Error());
  }

  if (Message_Trigger_At < millis()) {

    Message_Trigger_At = millis() + 1500;
    Serial.println("ERROR MODE");

    delay(2000);
  }

} // END MARKER - Error Mode


void setup() {
  wBus.Boot_Message();
  Serial.println("Boot Start");

  wBus.begin(I2C_Address);

  TWAR = (I2C_Address << 1) | 1;
  wBus.onReceive(I2C_Receive);

  Serial.println("Boot Done");
}


void loop() {
  wBus.Device_ID_Check();
  wBus.Blink_LED(false);

  if (wBus.I2C_BUS_Error() != 0 && 3) {
    Error_Mode();
  }

  // if (Message_Trigger_At < millis()) {
  //   Message_Trigger_At = millis() + 1500;
  //   Serial.println("OK");
  //   Serial.println(wBus.Queue_Peek_Queue());
  // }




}
