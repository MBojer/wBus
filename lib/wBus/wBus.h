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
      WBus(int I2C_Device_ID, bool I2C_Internal_Pullup, int Max_Queue_Length, bool Log_To_Serial, long Serial_Speed);


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
      int I2C_BUS_Error();
      String I2C_BUS_Error_To_Text(int Error_Number);


      // --------------------------------------------- Blink LED ---------------------------------------------
      int Blink_LED(bool Read_Value_Only);
      void Blink_LED_Start(int Number_Of_Blinks);
      void Blink_LED_Start(int Number_Of_Blinks, int LED_Pin);
      void Blink_LED_Stop();


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
      int _I2C_Bus_Error = 0; // 0 = OK  -  All other numbers is the number of Error Bliks, see man.txt


      // --------------------------------------------- Serial ---------------------------------------------
      int _Log_To_Serial;
      long _Serial_Speed;


      // --------------------------------------------- Device ID Checkd ---------------------------------------------
      int _Device_ID_Check_OK = 0; // 0 = Not done  -  1 = Done and OK  -  2 = Failed  -  3 = Waiting for reply

      #define _Device_ID_Check_Checks_Default 6
      int _Device_ID_Check_Checks_Left = _Device_ID_Check_Checks_Default; // A check consists of broadcasting the units Device ID and then checking for a 2 secound to see if you get a reply
      int _Device_ID_Check_Error_Counter = _Device_ID_Check_Checks_Default;

      bool _Queue_Device_ID_Check_Hit;

      unsigned long _Device_ID_Check_Millis_Start_At = 0;
      unsigned long _Device_ID_Check_Millis_Interval = 2500;

      unsigned long _Device_ID_Check_Millis_Retry_At = 0;
      unsigned long _Device_ID_Check_Millis_Retry_Interval = 7500; // CHANGE ME - To a usefull number like 30 min (1800000)


      // --------------------------------------------- I2C Command Queue ---------------------------------------------
      int _Max_Queue_Length;
      int _Queue_Length = 0;
      bool _Queue_Is_Empthy = true;
      String _Queue_String = ";"; // ";" is used as seperators betweens the string in the queue
      // int _Queue_Device_ID_Check_Hit = false; // Look below int defined there just here as a reminder


      // --------------------------------------------- Blink LED ---------------------------------------------
      int _Blink_LED_Blinks_Left = 0;

      unsigned long _Blink_LED_Millis_Start_At = 0;
      unsigned long _Blink_LED_Millis_Interval = 500;
      unsigned long _Blink_LED_Millis_Interval_Break = 1500;

      int _Blink_LED_Pin;



  };

#endif
