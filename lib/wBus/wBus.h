/*
------------------------------ I2C Net ------------------------------
Version 0.1
Network for made to run on the I2C bus using broadcast
*/

#ifndef WBus_h
  #define WBus_h

  #include "Arduino.h"
  #include <inttypes.h>
  #include "Stream.h"
  #define BUFFER_LENGTH 32




  class WBus : public Stream {

    public:


      // --------------------------------------------- Setup ---------------------------------------------

      WBus(int I2C_Device_ID, bool I2C_Internal_Pullup, int Max_Queue_Length, bool Log_To_Serial, long Serial_Speed, int Loop_Delay);


      // --------------------------------------------- Wire Functions ---------------------------------------------

      void begin();
      void begin(uint8_t);
      void begin(int);
      void beginTransmission(uint8_t);
      void beginTransmission(int);
      int broadcast(String Broadcast_String);
      uint8_t endTransmission(void);
      uint8_t requestFrom(uint8_t, uint8_t);
      uint8_t requestFrom(int, int);
      virtual size_t write(uint8_t);
      virtual size_t write(const uint8_t *, size_t);
      virtual int available(void);
      virtual int read(void);
      virtual int peek(void);
  	  virtual void flush(void);
      void onReceive( void (*)(int) );
      void onRequest( void (*)(void) );
      void pullup(bool Activate);

      using Print::write; // Change ME / REMOVE ME


      // --------------------------------------------- WBus ---------------------------------------------

      int Device_ID_Check();
      void I2C_BUS_Error(int Error_Number);


      // --------------------------------------------- Queue ---------------------------------------------

      void Queue_Push(String Push_String, bool Add_To_Front_Of_Queue);
      String Queue_Pop();
      String Queue_Peek();
      String Queue_Peek_Queue();
      int Queue_Length();
      int Queue_Is_Empthy();
      void Queue_Clear();
      String Queue_Search_Peek(String Search_String);
      String Queue_Search_Pop(String Search_String, bool Delete_All_Matches);


      // --------------------------------------------- Misc ---------------------------------------------

      void Boot_Message();
      void Blink_LED(int Number_Of_Blinks, int LED_Pin = 13);
      int _Script_Loop_Delay = false;



    private:
      // --------------------------------------------- Two Wire Interface ---------------------------------------------

      static uint8_t rxBuffer[BUFFER_LENGTH];
      static uint8_t rxBufferIndex;
      static uint8_t rxBufferLength;

      static uint8_t txAddress;
      static uint8_t txBuffer[BUFFER_LENGTH];
      static uint8_t txBufferIndex;
      static uint8_t txBufferLength;

      static uint8_t transmitting;
      static void (*user_onRequest)(void);
      static void (*user_onReceive)(int);
      static void onRequestService(void);
      static void onReceiveService(uint8_t*, int);

      // --------------------------------------------- I2C Bus ---------------------------------------------
      int _Device_ID;
      bool _I2C_Internal_Pullup;
      bool _I2C_Bus_Error = false; // 0 = OK  -  1 = Error


      // --------------------------------------------- Serial ---------------------------------------------
      int _Log_To_Serial;
      long _Serial_Speed;


      // --------------------------------------------- Device ID Checkd ---------------------------------------------
      int _Device_ID_Check_OK = 0; // 0 = Not done  -  1 = Done and OK  -  2 = Failed  -  3 = Waiting for reply
      int _Device_ID_Check_OK_Counter; // Used for Device ID check at boot to make sure the other device have time to send
      // int _Queue_Device_ID_Check_Hit = false; // Look below int defined there just here as a reminder
      bool _Queue_Device_ID_Check_Hit;


      // --------------------------------------------- I2C Command Queue ---------------------------------------------
      int _Max_Queue_Length;
      int _Queue_Length = 0;
      bool _Queue_Is_Empthy = true;
      String _Queue_String = ";"; // ";" is used as seperators betweens the string in the queue

  };

#endif
