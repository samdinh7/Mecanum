#include <Arduino.h>
#include "MyMotor.h"
#include "MyEncoder.h"
#include "MyPID.h"
#include "MySerial.h"

void setup()
{
  Init_Motor();
  Init_Encoder();
  Init_PID();
  Init_Serial();
}

void loop()
{
  Get_Angle();
  Compute_PID(); 
  Run_Motor();
  SerialDataPrint();
  SerialDataWrite();
}
