/*
------------------------------ I2C Net ------------------------------
Version 0.1
Network for made to run on the I2C bus using broadcast
*/

#ifndef wBus_h
  #define wBus_h 

  // From MWWire (WSWire)

  #include <inttypes.h>
  #include "Stream.h"
  #define BUFFER_LENGTH 32

  // End Marker MBWire (WSWire)


  #include "Arduino.h"

  class wBus : public Stream {

    public:
      // --------------------------------------------- From MBWire ---------------------------------------------

      void begin();
      void begin(uint8_t);
      void begin(int);
      void beginTransmission(uint8_t);
      void beginTransmission(int);
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

      // --------------------------------------------- wBus ---------------------------------------------

      wBus(int I2C_Device_ID, int Max_Queue_Length, bool Log_To_Serial, long Serial_Speed);
      void Boot_Message();
      int Device_ID_Check();
      void I2C_BUS_Error(int Error_Number);
      void Broadcast(String Broadcast_String);

      // --------------------------------------------- I2C Command Queue ---------------------------------------------

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

      void Error_Blink(int Number_Of_Blinks);




    private:
      // --------------------------------------------- From MBWire ---------------------------------------------

      static uint8_t rxBuffer[];
      static uint8_t rxBufferIndex;
      static uint8_t rxBufferLength;

      static uint8_t txAddress;
      static uint8_t txBuffer[];
      static uint8_t txBufferIndex;
      static uint8_t txBufferLength;

      static uint8_t transmitting;
      static void (*user_onRequest)(void);
      static void (*user_onReceive)(int);
      static void onRequestService(void);
      static void onReceiveService(uint8_t*, int);

      // --------------------------------------------- General ---------------------------------------------

      int _Device_ID;

      int _Log_To_Serial;
      long _Serial_Speed;


      // --------------------------------------------- Device ID Checkd ---------------------------------------------

      int _Device_ID_Check_OK = 0;


      // --------------------------------------------- I2C Command Queue ---------------------------------------------

      int _Max_Queue_Length;
      int _Queue_Length = 0;
      bool _Queue_Is_Empthy = true;
      String _Queue_String = ";"; // ";" is used as seperators betweens the string in the queue

  };

#endif
