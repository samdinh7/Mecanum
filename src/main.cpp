#include <Arduino.h>
#include "MyMotor.h"
#include "MyEncoder.h"
#include "MyPID.h"
#include "MySerial.h"
#include "HardwareSerial.h"
#include "HX711.h"
#include "soc/rtc.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
const char *ssid = "Be Hai";        // Replace with your Wi-Fi SSID
const char *password = "333666999"; // Replace with your Wi-Fi password
#define DT 2                        // Data pin connected to GPIO 21
#define SCK 15                      // Clock pin connected to GPIO 22
uint32_t reading = 0;
HX711 scale;
typedef enum Robot_State
{
    None,
    Triangle,
    SendWeight
};
Robot_State SpecialState = None;
AsyncWebServer server(80);
float Mass = 0;
// WebApp HTML with corrected layout and commands
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32-CAM Robot</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 0;
      padding: 0;
      background-color: #f4f4f9;
    }
    h1 {
      margin: 20px 0;
    }
    #video-stream {
      width: 100%;
      max-width: 480px;
      height: auto;
      border-radius: 10px;
      margin-bottom: 20px;
    }
    .container {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      grid-gap: 10px;
      justify-items: center;
      margin: 0 auto;
      max-width: 300px;
    }
    .container button {
      padding: 10px;
      font-size: 24px;
      cursor: pointer;
      background-color: #3949ab;
      color: white;
      border: none;
      border-radius: 5px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2);
    }
    .container button:hover {
      background-color: #5c6bc0;
    }
    .forward { grid-column: 2; }
    .left { grid-column: 1; }
    .stop { grid-column: 2; font-size: 18px; }
    .right { grid-column: 3; }
    .backward { grid-column: 2; }
    .diag135 { grid-column: 1; grid-row: 1; } /* Up-left */
    .diag45 { grid-column: 3; grid-row: 1; }  /* Up-right */
    .diag225 { grid-column: 1; grid-row: 3; } /* Down-left */
    .diag315 { grid-column: 3; grid-row: 3; } /* Down-right */
  </style>
</head>
<body>
  <h1>ESP32-CAM Robot</h1>
  <img id="video-stream" src="http://192.168.1.13:81/stream" alt="Video Stream" style="transform: rotate(180deg);">
  <div class="container">
    <button class="diag135" onclick="sendCommand('/diag135')">&#x2196;</button> <!-- Up-left -->
    <button class="forward" onclick="sendCommand('/forward')">&#x2191;</button> <!-- Up -->
    <button class="diag45" onclick="sendCommand('/diag45')">&#x2197;</button> <!-- Up-right -->
    <button class="left" onclick="sendCommand('/left')">&#x2190;</button> <!-- Left -->
    <button class="stop" onclick="sendCommand('/stop')">STOP</button> <!-- Stop -->
    <button class="right" onclick="sendCommand('/right')">&#x2192;</button> <!-- Right -->
    <button class="diag225" onclick="sendCommand('/diag225')">&#x2199;</button> <!-- Down-left -->
    <button class="backward" onclick="sendCommand('/backward')">&#x2193;</button> <!-- Down -->
    <button class="diag315" onclick="sendCommand('/diag315')">&#x2198;</button> <!-- Down-right -->
    <button class="" onclick="sendCommand('/rotateleft')">Rotate left</button> 
    <button class="" onclick="sendCommand('/path')">Set Path</button> 
    <button class="" onclick="sendCommand('/rotateright')">Rotate right</button> 
  </div>
  <div>
  <h2>Weight: <span id="weight-value">0</span></h2>
  <button onclick="updateWeight()">Get Weight</button>
</div>

  <script>
    function sendCommand(endpoint) {
        fetch(endpoint)
            .then(response => response.text())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));
    }
    function updateWeight() {
    fetch('/weight')
        .then(response => response.text())
        .then(data => document.getElementById('weight-value').innerText = data)
        .catch(error => console.error('Error:', error));
}
  </script>
</body>
</html>
)rawliteral";

float Vx = 0;
float Vy = 0;
float W = 0.0;
void setup()
{

    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    // Serial.print("Hello"); // Print

    Init_Motor();
    Init_Encoder();
    Init_PID();
    Init_Serial();
    // delay(2000);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    int timeout = 30; // Timeout counter
    while (WiFi.status() != WL_CONNECTED && timeout > 0)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
        timeout--;
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("Failed to connect to WiFi");
    }
    scale.begin(DT, SCK);

    // Wait for the scale to stabilize
    // if (!scale.is_ready())
    // {
    //     Serial.println("HX711 not found.");
    //     while (true);
    // }
    Serial.println("HX711 initialized.");
    // long rawValue = scale.get_units();
    // Serial.println(rawValue);
    // Set calibration factor (you will need to adjust this value)
    scale.set_scale(-1616593.f); // Adjust this to your specific load cell
    scale.tare();                // Reset the scale to zero

    Serial.println("Calibration done.");
    // Initialize motors
    Init_Motor();

    // Serve WebApp
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/html", index_html); });

    //   Define HTTP endpoints for motor control
    server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 3.5;
        Vy = 0;
        W = 0;
        // Run_Motor();
        request->send(200, "text/plain", "Moving Forward"); });

    server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = -3.5;
        Vy = 0;
        W = 0;
        // Run_Motor();
        request->send(200, "text/plain", "Moving Backward"); });

    server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 0;
        Vy = 3.5;
        W = 0;
        // Run_Motor();
        request->send(200, "text/plain", "Moving Left"); });

    server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 0;
        Vy = -3.5;
        W = 0;
        request->send(200, "text/plain", "Moving Right"); });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 0;
        Vy = 0;
        W = 0;
        request->send(200, "text/plain", "Stopped"); });

    server.on("/diag45", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 2.5;
        Vy = -2.5;
        W = 0;
        request->send(200, "text/plain", "Moving 45°"); });

    server.on("/diag135", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 2.5;
        Vy = 2.5;
        W = 0;
        request->send(200, "text/plain", "Moving 135°"); });

    server.on("/diag225", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = -2.5;
        Vy = 2.5;
        W = 0;
        request->send(200, "text/plain", "Moving 225°"); });

    server.on("/diag315", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = -2.5;
        Vy = -2.5;
        W = 0;
        request->send(200, "text/plain", "Moving 315°"); });

    server.on("/rotateleft", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 0;
        Vy = 0;
        W = -15;
        request->send(200, "text/plain", "Moving 315°"); });

    server.begin();
    server.on("/rotateright", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        Vx = 0;
        Vy = 0;
        W = 15;
        request->send(200, "text/plain", "Moving 315°"); });
    server.on("/path", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    SpecialState = Triangle;
        request->send(200, "text/plain", "Moving triangle"); });

    server.on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request)
              {
              if (request->hasParam("value")) {
                String value = request->getParam("value")->value();
                int speed = value.toInt();
                Serial.print("Speed set to: ");
                Serial.println(speed);
                // Update motor speed variable here
                // Example: motorSpeed = speed;
                request->send(200, "text/plain", "Speed updated");
              } else {
                request->send(400, "text/plain", "Missing speed value");
              } });

    server.on("/weight", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    if (scale.is_ready()) {
        float weight = abs(scale.get_units(10)); // Average of 10 readings
        Serial.print("Weight measured: ");
        Serial.println(weight);
        request->send(200, "text/plain", String(weight, 2)); // Send weight with 2 decimal points
    } else {
        request->send(500, "text/plain", "HX711 not ready"); // Handle HX711 not ready error
    } });

    server.begin();
}

uint32_t pretime = 0;
float tsamp = 0.05;  // Sampling time in seconds
float Nu_Round = 1;  // Desired number of rotations
int PWM = 240;       // PWM value for motor control
int PWM_Max = 255;   // Maximum PWM value
int Error_Angle = 5; // Acceptable error in degrees
volatile long int prePulse_Encoder0 = 0;
volatile long int nowPulse_Encoder0 = 0;
volatile long int prePulse_Encoder1 = 0;
volatile long int nowPulse_Encoder1 = 0;
volatile long int prePulse_Encoder2 = 0;
volatile long int nowPulse_Encoder2 = 0;
volatile long int prePulse_Encoder3 = 0;
volatile long int nowPulse_Encoder3 = 0;
int count = 0;

float Omega_M0 = 0; // rpm
float Omega_M1 = 0;
float Omega_M2 = 0;
float Omega_M3 = 0;

float e0[] = {0.0, 0.0, 0.0};
float Kp0 = 0.5; // 0.5
float Ki0 = 2.3; // 12.3
float Kd0 = 0.0;
float integral0 = 0.0;
float derivative0 = 0.0;
int pid0 = 0.00;
float setpoint0 = 0;

float e1[] = {0.0, 0.0, 0.0};
float Kp1 = 0.5; // 0.5
float Ki1 = 2.3; // 12.3
float Kd1 = 0.0;
float integral1 = 0.0;
float derivative1 = 0.0;
int pid1 = 0.00;
float setpoint1 = 0;

float e2[] = {0.0, 0.0, 0.0};
float Kp2 = 0.5; // 0.5
float Ki2 = 2.3; // 12.3
float Kd2 = 0.0;
float integral2 = 0.0;
float derivative2 = 0.0;
int pid2 = 0.00;
float setpoint2 = 0;

float e3[] = {0.0, 0.0, 0.0};
float Kp3 = 0.5; // 0.5
float Ki3 = 2.3; // 12.3
float Kd3 = 0.0;
float integral3 = 0.0;
float derivative3 = 0.0;
int pid3 = 0.00;
float setpoint3 = 0;
float R = 0.045;
float lx = 0.122;
float ly = 0.145;

int count_Triangle = 0;
int time_Limit = 40;
void loop()
{
    uint32_t nowtime = millis();            // Get the current time in milliseconds
    uint32_t deltatime = nowtime - pretime; // Calculate the elapsed time
    if (deltatime >= tsamp * 1000)
    { // Check if the sampling time has passed
        switch (SpecialState)
        {
        case Triangle:
        {
            if (count_Triangle < time_Limit)
            {
                Vx = 2.5;
                Vy = -2.5;
                W = 0;
                count_Triangle++;
                if (count_Triangle == time_Limit - 1)
                {
                    Vx = 0;
                    Vy = 0;
                    W = 0;
                }
            }
            else if (count_Triangle >= time_Limit && count_Triangle < time_Limit * 2)
            {
                Vx = -2.5;
                Vy = -2.5;
                W = 0;
                count_Triangle++;
                if (count_Triangle == time_Limit*2 - 1)
                {
                    Vx = 0;
                    Vy = 0;
                    W = 0;
                }
            }
            else if (count_Triangle >= time_Limit * 2 && count_Triangle < time_Limit * 3.5)
            {
                Vx = 0;
                Vy = 3.5;
                W = 0;
                count_Triangle++;
            }
            else
            {
                Vx = 0;
                Vy = 0;
                W = 0;
                SpecialState = None;
                count_Triangle = 0;
            }
        }
        }
        setpoint0 = (1 / R) * (Vx - Vy - (lx + ly) * W);
        setpoint1 = (1 / R) * (Vx + Vy + (lx + ly) * W);
        setpoint2 = (1 / R) * (Vx + Vy - (lx + ly) * W);
        setpoint3 = (1 / R) * (Vx - Vy + (lx + ly) * W);
        nowPulse_Encoder0 = cnt0;
        nowPulse_Encoder1 = cnt1;
        nowPulse_Encoder2 = cnt2;
        nowPulse_Encoder3 = cnt3;

        Omega_M0 = ((float)(nowPulse_Encoder0 - prePulse_Encoder0) / 330 * (1.00 / (tsamp))) * 60;
        Omega_M1 = ((float)(nowPulse_Encoder1 - prePulse_Encoder1) / 330 * (1.00 / (tsamp))) * 60;
        Omega_M2 = ((float)(nowPulse_Encoder2 - prePulse_Encoder2) / 330 * (1.00 / (tsamp))) * 60;
        Omega_M3 = ((float)(nowPulse_Encoder3 - prePulse_Encoder3) / 330 * (1.00 / (tsamp))) * 60;

        e0[1] = setpoint0 - Omega_M0; // setpoints[0]
        integral0 += e0[1] * tsamp;
        derivative0 = (e0[1] - e0[0]) / tsamp;
        e0[0] = e0[1];
        pid0 = Kp0 * e0[1] + Ki0 * integral0 + Kd0 * derivative0;
        if (pid0 > 255)
            pid0 = 255;
        if (pid0 < -255)
            pid0 = -255;

        e1[1] = setpoint1 - Omega_M1; // setpoints[0]
        integral1 += e1[1] * tsamp;
        derivative1 = (e1[1] - e1[0]) / tsamp;
        e1[0] = e1[1];
        pid1 = Kp1 * e1[1] + Ki1 * integral1 + Kd1 * derivative1;
        if (pid1 > 255)
            pid1 = 255;
        if (pid1 < -255)
            pid1 = -255;

        e2[1] = setpoint2 - Omega_M2; // setpoints[0]
        integral2 += e2[1] * tsamp;
        derivative2 = (e2[1] - e2[0]) / tsamp;
        e2[0] = e2[1];
        pid2 = Kp2 * e2[1] + Ki2 * integral2 + Kd2 * derivative2;
        if (pid2 > 255)
            pid2 = 255;
        if (pid2 < -255)
            pid2 = -255;

        e3[1] = setpoint3 - Omega_M3; // setpoints[0]
        integral3 += e3[1] * tsamp;
        derivative3 = (e3[1] - e3[0]) / tsamp;
        e3[0] = e3[1];
        pid3 = Kp3 * e3[1] + Ki3 * integral3 + Kd3 * derivative3;
        if (pid3 > 255)
            pid3 = 255;
        if (pid3 < -255)
            pid3 = -255;

        if (Vx == 0 && Vy == 0 && W == 0)
        {
            pid0 = 0;
            pid1 = 0;
            pid2 = 0;
            pid3 = 0;
        }

        if (pid0 >= 0)
        {
            analogWrite(MOT0_A, pid0);
            digitalWrite(MOT0_B, LOW);
        }
        else if (pid0 < 0)
        {
            analogWrite(MOT0_A, PWM_Max + pid0);
            digitalWrite(MOT0_B, HIGH);
        }

        if (pid1 >= 0)
        {
            analogWrite(MOT1_A, pid1);
            digitalWrite(MOT1_B, LOW);
        }
        else if (pid1 < 0)
        {
            analogWrite(MOT1_A, PWM_Max + pid1);
            digitalWrite(MOT1_B, HIGH);
        }

        if (pid2 >= 0)
        {
            analogWrite(MOT2_A, pid2);
            digitalWrite(MOT2_B, LOW);
        }
        else if (pid2 < 0)
        {
            analogWrite(MOT2_A, PWM_Max + pid2);
            digitalWrite(MOT2_B, HIGH);
        }

        if (pid3 >= 0)
        {
            analogWrite(MOT3_A, pid3);
            digitalWrite(MOT3_B, LOW);
        }
        else if (pid3 < 0)
        {
            analogWrite(MOT3_A, PWM_Max + pid3);
            digitalWrite(MOT3_B, HIGH);
        }

        // count++;
        // if(count >= 20){
        // // Serial.println(Omega_M1);
        // Vx = 0;
        // Vy = 0;
        // W = 0;
        // count = 20;
        // }
        pretime = nowtime;
        prePulse_Encoder0 = nowPulse_Encoder0;
        prePulse_Encoder1 = nowPulse_Encoder1;
        prePulse_Encoder2 = nowPulse_Encoder2;
        prePulse_Encoder3 = nowPulse_Encoder3;
        count++;
        if (count >= 10)
        {
            if (scale.is_ready())
            {
                // Read and print the weight
                // float weight = abs(scale.get_units(10)); // Average of 10 readings
                // Serial.print("Weight: ");
                // Serial.print(weight, 2); // Print with 2 decimal points
                // Serial.println(" kg");
            }
            else
            {
                // Serial.println("HX711 not ready.");
            }

            // delay(1000); // Update every second
            count = 0;
        }
    }
}

// Serial.println(th0); // Print the angle to the serial monitor
// Serial.println(th1); // Print the angle to the serial monitor
// Serial.println(th2); // Print the angle to the serial monitor
// Serial.println(th3); // Print the angle to the serial monitor
// Serial.println(deltatime); // Print the angle to the serial monitor