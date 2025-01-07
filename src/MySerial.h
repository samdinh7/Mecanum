#include <Arduino.h>

unsigned long Serial_time = 0; // time in us
double th1_ref;
// ====================================================================================
void Init_Serial()
{
  Serial.begin(115200);
  while (!Serial)
    ;
}
// ====================================================================================
void SerialDataPrint()
{
  if (micros() - Serial_time >= 10000)
  {
    Serial_time = micros();
    // For MATLAB
    Serial.print(Serial_time / 10000);
    Serial.print(",");
    Serial.print(MOT1_cmd);
    Serial.print(",");
    Serial.print(th1_ref);
    Serial.print(",");
    Serial.print(th1);
    Serial.println();

    // For Teleplot
    // Serial.println(Serial_time / 10000);
    // Serial.print(">MOT1_cmd:");
    // Serial.println(MOT1_cmd);
    // Serial.print(">th1:");
    // Serial.println(th1);
    // Serial.print(">th1_ref:");
    // Serial.println(th1_ref);
    // Serial.print(">kp:");
    // Serial.println(kp);
    // Serial.print(">ki:");
    // Serial.println(ki);
    // Serial.print(">kd:");
    // Serial.println(kd);
  }
}
// ====================================================================================
void SerialDataWrite()
{
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      switch (received_chars[0])
      {
      case 'a':
        received_chars.remove(0, 1);
        th1_ref = received_chars.toFloat();
        break;
      case 'q':
        received_chars.remove(0, 1);
        kp = received_chars.toFloat();
        break;
      case 'w':
        received_chars.remove(0, 1);
        ki = received_chars.toFloat();
        break;
      case 'e':
        received_chars.remove(0, 1);
        kd = received_chars.toFloat();
        break;
      default:
        break;
      }
      received_chars = "";
    }
  }
}