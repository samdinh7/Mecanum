#include <Arduino.h>


#define ENC0_A 4    // MOT 1A
#define ENC0_B 16   // MOT 1B
#define ENC1_A 13    // MOT 1A
#define ENC1_B 14   // MOT 1B
#define ENC2_A 22    // MOT 1A
#define ENC2_B 23   // MOT 1B
#define ENC3_A 32    // MOT 1A
#define ENC3_B 35   // MOT 1B
#define ENC_RES 330 // Encoder resolution*Gearbox ratio: 11*30

volatile long int cnt0 = 0; // Volatile as it changed during interrupt
double th0 = 0;             // Position angle in degrees
volatile long int cnt1 = 0; // Volatile as it changed during interrupt
double th1 = 0;             // Position angle in degrees
volatile long int cnt2 = 0; // Volatile as it changed during interrupt
double th2 = 0;             // Position angle in degrees
volatile long int cnt3 = 0; // Volatile as it changed during interrupt
double th3 = 0;             // Position angle in degrees



//================================================================================
void readEncoder0()
{
    int b = digitalRead(ENC0_B);
    cnt0 = (b > 0) ? (cnt0 + 1) : (cnt0 - 1);
}
void readEncoder1()
{
    int b = digitalRead(ENC1_B);
    cnt1 = (b > 0) ? (cnt1 + 1) : (cnt1 - 1);
}
void readEncoder2()
{
    int b = digitalRead(ENC2_B);
    cnt2 = (b > 0) ? (cnt2 + 1) : (cnt2 - 1);
}
void readEncoder3()
{
    int b = digitalRead(ENC3_B);
    cnt3 = (b > 0) ? (cnt3 + 1) : (cnt3 - 1);
}

void Init_Encoder()
{
    pinMode(ENC0_A, INPUT);
    pinMode(ENC0_B, INPUT);
    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT);
    pinMode(ENC2_A, INPUT);
    pinMode(ENC2_B, INPUT);
    pinMode(ENC3_A, INPUT);
    pinMode(ENC3_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(ENC0_A), readEncoder0, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), readEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC3_A), readEncoder3, RISING);


}

void Get_Angle()
{
    th0 = cnt0 * 360 / ENC_RES; // Conversion between encoder count and degree
    th1 = cnt1 * 360 / ENC_RES; // Conversion between encoder count and degree
    th2 = cnt2 * 360 / ENC_RES; // Conversion between encoder count and degree
    th3 = cnt3 * 360 / ENC_RES; // Conversion between encoder count and degree
}