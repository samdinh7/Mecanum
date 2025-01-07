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
}

uint32_t pretime = 0;
uint32_t tsamp = 0.1; // Sampling time in seconds
float Nu_Round = 1;   // Desired number of rotations
int PWM = 240;         // PWM value for motor control
int PWM_Max = 255;    // Maximum PWM value
int Error_Angle = 5;  // Acceptable error in degrees

void twoRotation()
{
    if (th1 <= Nu_Round * 360)
    {                             // If the angle is less than the target angle minus the error
        analogWrite(MOT1_A, PWM_Max); // Set motor to move forward
        digitalWrite(MOT1_B, LOW);
        Serial.println("Moving forward");
    }
    else
    {
        analogWrite(MOT1_A, 0); // Stop the motor
        digitalWrite(MOT1_B, LOW);
        delay(1000);             // Wait for 1 second
        Nu_Round = Nu_Round + 1; // Increase the number of rotations by 1
    }
}

void counterClockwiseRotate()
{
    analogWrite(MOT1_A, PWM); // Set motor to move forward
    digitalWrite(MOT1_B, LOW);
}

void clockwiseRotate()
{
    analogWrite(MOT1_A, PWM_Max - PWM); // Stop the motor
    digitalWrite(MOT1_B, HIGH);
    Serial.println("Moving clockwise");
}

void stopMotor()
{
    analogWrite(MOT1_A, 0); // Stop the motor
    digitalWrite(MOT1_B, LOW);
    Serial.println("Stop");
}

void loop()
{
    uint32_t nowtime = millis();            // Get the current time in milliseconds
    uint32_t deltatime = nowtime - pretime; // Calculate the elapsed time

    if (deltatime >= tsamp * 1000)
    {                        // Check if the sampling time has passed
        Get_Angle();         // Read the encoder angle
        Serial.println(th1); // Print the angle to the serial monitor

        if (th1 < Nu_Round * 360)
        {
            counterClockwiseRotate();
        }
         else if (th1< 1440)
        {
            Nu_Round += 1;
        }
        else{
            stopMotor();
        }
    }
}