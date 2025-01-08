#include <Arduino.h>
#include "MyMotor.h"
#include "MyEncoder.h"
#include "MyPID.h"
#include "MySerial.h"
#include "HardwareSerial.h"



void setup()
{

    Serial.begin(115200);  // Initialize serial communication at 115200 baud rate
    Serial.print("Hello"); // Print
    Init_Motor();
    Init_Encoder();
    Init_PID();
    Init_Serial();
    delay(2000);
}

uint32_t pretime = 0;
float tsamp = 0.05; // Sampling time in seconds
float Nu_Round = 1;   // Desired number of rotations
int PWM = 240;         // PWM value for motor control
int PWM_Max = 255;    // Maximum PWM value
int Error_Angle = 5;  // Acceptable error in degrees
volatile long int prePulse_Encoder0 = 0;
volatile long int nowPulse_Encoder0 = 0;
volatile long int prePulse_Encoder1 = 0;
volatile long int nowPulse_Encoder1 = 0;
volatile long int prePulse_Encoder2 = 0;
volatile long int nowPulse_Encoder2 = 0;
volatile long int prePulse_Encoder3 = 0;
volatile long int nowPulse_Encoder3 = 0;
int count = 0;

float Omega_M0 = 0; //rpm
float Omega_M1 = 0;
float Omega_M2 = 0;
float Omega_M3 = 0;


	float e0[] = { 0.0, 0.0, 0.0 };
	float Kp0 = 0.2;	//0.5
	float Ki0 = 4.3;	//12.3
	float Kd0 = 0.0;
	float integral0 = 0.0;
	float derivative0 = 0.0;
	int pid0 = 0.00;
    float setpoint0 = 0;
    
	float e1[] = { 0.0, 0.0, 0.0 };
	float Kp1 = 0.2;	//0.5
	float Ki1 = 4.3;	//12.3
	float Kd1 = 0.0;
	float integral1 = 0.0;
	float derivative1 = 0.0;
	int pid1 = 0.00;
    float setpoint1 = 0;

	float e2[] = { 0.0, 0.0, 0.0 };
	float Kp2 = 0.2;	//0.5
	float Ki2 = 4.3;	//12.3
	float Kd2 = 0.0;
	float integral2 = 0.0;
	float derivative2 = 0.0;
	int pid2 = 0.00;
    float setpoint2 = 0;

	float e3[] = { 0.0, 0.0, 0.0 };
	float Kp3 = 0.2;	//0.5
	float Ki3 = 4.3;	//12.3
	float Kd3 = 0.0;
	float integral3 = 0.0;
	float derivative3 = 0.0;
	int pid3 = 0.00;
    float setpoint3 = 0;
    float R = 0.045;
    float lx = 0.122;
    float ly = 0.145;
    float Vx = 0;
    float Vy = 5;
    float W = 0.0;

void loop()
{
    uint32_t nowtime = millis();            // Get the current time in milliseconds
    uint32_t deltatime = nowtime - pretime; // Calculate the elapsed time
    if (deltatime >= tsamp * 1000)
    {                        // Check if the sampling time has passed
        setpoint0 = (1/R)*(Vx-Vy-(lx+ly)*W);
        setpoint1 = (1/R)*(Vx+Vy+(lx+ly)*W);
        setpoint2 = (1/R)*(Vx+Vy-(lx+ly)*W);
        setpoint3 = (1/R)*(Vx-Vy+(lx+ly)*W);
        nowPulse_Encoder0 = cnt0;
        nowPulse_Encoder1 = cnt1;
        nowPulse_Encoder2 = cnt2;
        nowPulse_Encoder3 = cnt3;

        Omega_M0 = ((float) (nowPulse_Encoder0 - prePulse_Encoder0) / 330* (1.00 / (tsamp))) * 60;
        Omega_M1 = ((float) (nowPulse_Encoder1 - prePulse_Encoder1) / 330* (1.00 / (tsamp))) * 60;
        Omega_M2 = ((float) (nowPulse_Encoder2 - prePulse_Encoder2) / 330* (1.00 / (tsamp))) * 60;
        Omega_M3 = ((float) (nowPulse_Encoder3 - prePulse_Encoder3) / 330* (1.00 / (tsamp))) * 60;



        e0[1] = setpoint0 - Omega_M0;	//setpoints[0]
			integral0 += e0[1] * tsamp;
			derivative0 = (e0[1] - e0[0]) / tsamp;
			e0[0] = e0[1];
			pid0 = Kp0 * e0[1] + Ki0 * integral0 + Kd0 * derivative0;
			if (pid0 >255 )
				pid0 = 255;
			if (pid0 < -255)
				pid0 = -255;

        e1[1] = setpoint1 - Omega_M1;	//setpoints[0]
			integral1 += e1[1] * tsamp;
			derivative1 = (e1[1] - e1[0]) / tsamp;
			e1[0] = e1[1];
			pid1 = Kp1 * e1[1] + Ki1 * integral1 + Kd1 * derivative1;
			if (pid1 >255 )
				pid1 = 255;
			if (pid1 < -255)
				pid1 = -255;

        e2[1] = setpoint2 - Omega_M2;	//setpoints[0]
			integral2 += e2[1] * tsamp;
			derivative2 = (e2[1] - e2[0]) / tsamp;
			e2[0] = e2[1];
			pid2 = Kp2 * e2[1] + Ki2 * integral2 + Kd2 * derivative2;
			if (pid2 >255 )
				pid2 = 255;
			if (pid2 < -255)
				pid2 = -255;

        e3[1] = setpoint3 - Omega_M3;	//setpoints[0]
			integral3 += e3[1] * tsamp;
			derivative3 = (e3[1] - e3[0]) / tsamp;
			e3[0] = e3[1];
			pid3 = Kp3 * e3[1] + Ki3 * integral3 + Kd3 * derivative3;
			if (pid3 >255 )
				pid3 = 255;
			if (pid3 < -255)
				pid3 = -255;
    if(pid0 > 0){
        analogWrite(MOT0_A, pid0); 
        digitalWrite(MOT0_B, LOW);
    }
    else if(pid0 < 0){
        analogWrite(MOT0_A, PWM_Max + pid0); 
        digitalWrite(MOT0_B, HIGH);
    }

    if(pid1 > 0){
        analogWrite(MOT1_A, pid1); 
        digitalWrite(MOT1_B, LOW);
    }
    else if(pid1 < 0){
        analogWrite(MOT1_A, PWM_Max + pid1); 
        digitalWrite(MOT1_B, HIGH);
    }

    if(pid2 > 0){
        analogWrite(MOT2_A, pid2); 
        digitalWrite(MOT2_B, LOW);
    }
    else if(pid2 < 0){
        analogWrite(MOT2_A, PWM_Max + pid2); 
        digitalWrite(MOT2_B, HIGH);
    }       

    if(pid3 > 0){
        analogWrite(MOT3_A, pid3); 
        digitalWrite(MOT3_B, LOW);
    }
    else if(pid3 < 0){
        analogWrite(MOT3_A, PWM_Max + pid3); 
        digitalWrite(MOT3_B, HIGH);
    }       

  

        count++;
        if(count >= 20){
        // Serial.println(Omega_M1);
        Vx = 0;
        Vy = 0;
        W = 0;
        count = 20;
        }
        pretime = nowtime;
        prePulse_Encoder0 = nowPulse_Encoder0;
        prePulse_Encoder1 = nowPulse_Encoder1;
        prePulse_Encoder2 = nowPulse_Encoder2;
        prePulse_Encoder3 = nowPulse_Encoder3;

    }
}




        // Serial.println(th0); // Print the angle to the serial monitor
        // Serial.println(th1); // Print the angle to the serial monitor
        // Serial.println(th2); // Print the angle to the serial monitor
        // Serial.println(th3); // Print the angle to the serial monitor
        // Serial.println(deltatime); // Print the angle to the serial monitor