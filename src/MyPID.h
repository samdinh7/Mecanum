#include <Arduino.h>
#include <PID_v1.h>

extern double th1_ref; 
double kp = 10.0, ki = 5.0, kd = 0.05;

// Input, output, reference
PID MOT1_PID(&th1, &MOT1_cmd, &th1_ref, kp, ki, kd, DIRECT);

// ================================================================
// Function Definition
// ================================================================
void Init_PID()
{
  MOT1_PID.SetMode(AUTOMATIC);
  MOT1_PID.SetOutputLimits(-255, 255);
  MOT1_PID.SetSampleTime(10);
}
// ================================================================
void Compute_PID()
{
  MOT1_PID.SetTunings(kp, ki, kd);
  MOT1_PID.Compute();
}
// ================================================================
