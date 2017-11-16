/*
------------------------------ I2C Net ------------------------------
Version 0.1
Network for made to run on the I2C bus using broadcast
*/

#ifndef wBus_h
#define wBus_h

#include "Arduino.h"


class wBus {

public:

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
