// #include <Arduino.h>
// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include "MyMotor.h"
// #include "MySerial.h"

// const char *ssid = "Be Hai";   // Replace with your Wi-Fi SSID
// const char *password = "333666999"; // Replace with your Wi-Fi password

// AsyncWebServer server(80);

// // WebApp HTML with corrected layout and commands
// const char index_html[] PROGMEM = R"rawliteral(
// <!DOCTYPE html>
// <html>
// <head>
//   <title>ESP32-CAM Robot</title>
//   <meta name="viewport" content="width=device-width, initial-scale=1.0">
//   <style>
//     body {
//       font-family: Arial, sans-serif;
//       text-align: center;
//       margin: 0;
//       padding: 0;
//       background-color: #f4f4f9;
//     }
//     h1 {
//       margin: 20px 0;
//     }
//     #video-stream {
//       width: 100%;
//       max-width: 480px;
//       height: auto;
//       border-radius: 10px;
//       margin-bottom: 20px;
//     }
//     .container {
//       display: grid;
//       grid-template-columns: repeat(3, 1fr);
//       grid-gap: 10px;
//       justify-items: center;
//       margin: 0 auto;
//       max-width: 300px;
//     }
//     .container button {
//       padding: 10px;
//       font-size: 24px;
//       cursor: pointer;
//       background-color: #3949ab;
//       color: white;
//       border: none;
//       border-radius: 5px;
//       box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2);
//     }
//     .container button:hover {
//       background-color: #5c6bc0;
//     }
//     .forward { grid-column: 2; }
//     .left { grid-column: 1; }
//     .stop { grid-column: 2; font-size: 18px; }
//     .right { grid-column: 3; }
//     .backward { grid-column: 2; }
//     .diag135 { grid-column: 1; grid-row: 1; } /* Up-left */
//     .diag45 { grid-column: 3; grid-row: 1; }  /* Up-right */
//     .diag225 { grid-column: 1; grid-row: 3; } /* Down-left */
//     .diag315 { grid-column: 3; grid-row: 3; } /* Down-right */
//   </style>
// </head>
// <body>
//   <h1>ESP32-CAM Robot</h1>
//   <img id="video-stream" src="http://Your_ESP32_CAM_IP:81/stream" alt="Video Stream">
//   <div class="container">
//     <button class="diag135" onclick="sendCommand('/diag135')">&#x2196;</button> <!-- Up-left -->
//     <button class="forward" onclick="sendCommand('/forward')">&#x2191;</button> <!-- Up -->
//     <button class="diag45" onclick="sendCommand('/diag45')">&#x2197;</button> <!-- Up-right -->
//     <button class="left" onclick="sendCommand('/left')">&#x2190;</button> <!-- Left -->
//     <button class="stop" onclick="sendCommand('/stop')">STOP</button> <!-- Stop -->
//     <button class="right" onclick="sendCommand('/right')">&#x2192;</button> <!-- Right -->
//     <button class="diag225" onclick="sendCommand('/diag225')">&#x2199;</button> <!-- Down-left -->
//     <button class="backward" onclick="sendCommand('/backward')">&#x2193;</button> <!-- Down -->
//     <button class="diag315" onclick="sendCommand('/diag315')">&#x2198;</button> <!-- Down-right -->
//   </div>

//   <script>
//     function sendCommand(endpoint) {
//         fetch(endpoint)
//             .then(response => response.text())
//             .then(data => console.log(data))
//             .catch(error => console.error('Error:', error));
//     }
//   </script>
// </body>
// </html>
// )rawliteral";

// void setup()
// {
//   Serial.begin(115200);

//   // Connect to Wi-Fi
//   WiFi.begin(ssid, password);
//   int timeout = 30; // Timeout counter
//   while (WiFi.status() != WL_CONNECTED && timeout > 0)
//   {
//     delay(1000);
//     Serial.println("Connecting to WiFi...");
//     timeout--;
//   }
//   if (WiFi.status() == WL_CONNECTED)
//   {
//     Serial.println("WiFi connected");
//     Serial.print("IP Address: ");
//     Serial.println(WiFi.localIP());
//   }
//   else
//   {
//     Serial.println("Failed to connect to WiFi");
//   }

//   // Initialize motors
//   Init_Motor();

//   // Serve WebApp
//   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
//             { request->send_P(200, "text/html", index_html); });

//   // Define HTTP endpoints for motor control
//   server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = 150;
//         MOT2_cmd = 150;
//         MOT3_cmd = 150;
//         MOT4_cmd = 150;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving Forward"); });

//   server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = -150;
//         MOT2_cmd = -150;
//         MOT3_cmd = -150;
//         MOT4_cmd = -150;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving Backward"); });

//   server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = -150;
//         MOT2_cmd = 150;
//         MOT3_cmd = 150;
//         MOT4_cmd = -150;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving Left"); });

//   server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = 150;
//         MOT2_cmd = -150;
//         MOT3_cmd = -150;
//         MOT4_cmd = 150;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving Right"); });

//   server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = 0;
//         MOT2_cmd = 0;
//         MOT3_cmd = 0;
//         MOT4_cmd = 0;
//         Run_Motor();
//         request->send(200, "text/plain", "Stopped"); });

//   server.on("/diag45", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = 150;
//         MOT2_cmd = 0;
//         MOT3_cmd = 0;
//         MOT4_cmd = 150;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving 45°"); });

//   server.on("/diag135", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = 0;
//         MOT2_cmd = 150;
//         MOT3_cmd = 150;
//         MOT4_cmd = 0;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving 135°"); });

//   server.on("/diag225", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = -150;
//         MOT2_cmd = 0;
//         MOT3_cmd = 0;
//         MOT4_cmd = -150;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving 225°"); });

//   server.on("/diag315", HTTP_GET, [](AsyncWebServerRequest *request)
//             {
//         MOT1_cmd = 0;
//         MOT2_cmd = -150;
//         MOT3_cmd = -150;
//         MOT4_cmd = 0;
//         Run_Motor();
//         request->send(200, "text/plain", "Moving 315°"); });

//   server.begin();
// }

// void loop()
// {
//   // Nothing to do here, everything is handled by the server
// }