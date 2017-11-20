#include <Arduino.h>

#include <WBus.h>

#include <MemoryFree.h> // REMOVE ME

int I2C_Address = 11;

unsigned long Message_Trigger_At = 1500;

WBus wBus(I2C_Address, true, 20, true, 115200);

void I2C_Receive(int HowMany) {
  String I2C_Receive_Data; // UNCOMMENT ME LATER - Used to make sure that the I2C_Receive_Data gets cleared every time

  while (wBus.available()) {
    I2C_Receive_Data += (char)wBus.read();
  }

  wBus.Queue_Push(I2C_Receive_Data, false);

  Serial.print("Resived: ");
  Serial.println(I2C_Receive_Data);
  I2C_Receive_Data = "";

} //End marker for I2C_Receive


void Error_Mode(void) {
  if (wBus.Blink_LED(true) == 0) {
    wBus.Blink_LED_Start(wBus.I2C_BUS_Error());
  }

  if (Message_Trigger_At > millis()) {
    Message_Trigger_At = millis() + 1500;
    Serial.println("ERROR MODE");
  }

} // END MARKER - Error Mode


void setup() {
  Serial.begin(115200);
  Serial.println("Boot Start");

  wBus.begin(I2C_Address);

  TWAR = (I2C_Address << 1) | 1;
  wBus.onReceive(I2C_Receive);

  Serial.println("Boot Done");
}


void loop() {
  wBus.Device_ID_Check();
  wBus.Blink_LED(false);

  if (wBus.I2C_BUS_Error() != 0) {
    Error_Mode();
  }



}
