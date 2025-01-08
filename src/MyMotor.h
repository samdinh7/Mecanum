#include <Arduino.h>

#define MOT0_A 18 // MOT 0A
#define MOT0_B 17 // MOT 0B
#define MOT1_A 26 // MOT 1A
#define MOT1_B 27 // MOT 1B
#define MOT2_A 19 // MOT 2A
#define MOT2_B 21 // MOT 2B
#define MOT3_A 25 // MOT 3A
#define MOT3_B 33 // MOT 3B

#define MOT1_Channel 0 // MOT 1 channel

#define PWM_FREQ 10000 // PWM Frequency: 10kHz
#define PWM_RES 8      // PWM resolution 255

double MOT1_cmd = 0; // MOT1 command [-255; 255]

//============================================================
void Init_Motor()
{
    pinMode(MOT0_A, OUTPUT);
    pinMode(MOT0_B, OUTPUT);
    pinMode(MOT1_A, OUTPUT);
    pinMode(MOT1_B, OUTPUT);
    pinMode(MOT2_A, OUTPUT);
    pinMode(MOT2_B, OUTPUT);
    pinMode(MOT3_A, OUTPUT);
    pinMode(MOT3_B, OUTPUT);
    analogWrite(MOT0_A, 0); 
    digitalWrite(MOT0_B, LOW);
    analogWrite(MOT1_A, 0); 
    digitalWrite(MOT1_B, LOW);
    analogWrite(MOT2_A, 0); 
    digitalWrite(MOT2_B, LOW);
    analogWrite(MOT3_A, 0); 
    digitalWrite(MOT3_B, LOW);
    ledcSetup(MOT1_Channel, PWM_FREQ, PWM_RES);
}
//============================================================
void Send_PWM(int PINA, int PINB, double mot_cmd, int channel)
{
    // Reverse direction, if the mot_cmd is from [-255; 0]
    if (mot_cmd < 0)
    {
        ledcAttachPin(PINB, channel);
        ledcDetachPin(PINA);
        digitalWrite(PINA, LOW);
        ledcWrite(channel, abs(mot_cmd));
    }
    // Forward direction, if the mot_cmd is from [0; 255]; 
    else 
    {
        ledcAttachPin(PINA, channel);
        ledcDetachPin(PINB);
        digitalWrite(PINB, LOW);
        ledcWrite(channel, abs(mot_cmd));
    }
}

//============================================================
void Run_Motor()
{
    Send_PWM(MOT1_A, MOT1_B, MOT1_cmd, MOT1_Channel);
}
