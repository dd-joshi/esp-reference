/*
  ESP32 + ThingSpeak IoT Example

  This program reads some data and sends it to ThingSpeak.

  To use IoT, you only need to:

  1. Enter your Wi-Fi name
  2. Enter your Wi-Fi password
  3. Enter your ThingSpeak Write API Key
  4. Use SEND_IOT(data) to send any value
*/


#include "EasyIoT.h"


// =====================================================
// STEP 1: ENTER YOUR IoT DETAILS
// =====================================================

// Your Wi-Fi name
#define WIFI_SSID "YOUR_WIFI_NAME"

// Your Wi-Fi password
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Your ThingSpeak Write API Key
#define THINGSPEAK_KEY "YOUR_THINGSPEAK_WRITE_API_KEY"


void setup()
{
  Serial.begin(115200);


  // ===================================================
  // STEP 2: START IoT
  // ===================================================
  //
  // This connects the ESP32 to Wi-Fi and ThingSpeak.
  // Use it once inside setup().
  //

  IOT_BEGIN(WIFI_SSID, WIFI_PASSWORD, THINGSPEAK_KEY);
}


void loop()
{
  // ===================================================
  // YOUR NORMAL SENSOR CODE
  // ===================================================
  //
  // This can be ANY sensor:
  //
  // Analog sensor
  // Temperature sensor
  // Distance sensor
  // Light sensor
  // I2C sensor
  // etc.
  //
  // EasyIoT does not care where the data comes from.
  // ===================================================


  int data = analogRead(34);


  // Show data on Serial Monitor
  Serial.print("Sensor Data: ");
  Serial.println(data);


  // ===================================================
  // STEP 3: SEND DATA TO IoT
  // ===================================================
  //
  // Put the value you want to send inside SEND_IOT().
  //
  // Examples:
  //
  // SEND_IOT(temperature);
  // SEND_IOT(distance);
  // SEND_IOT(light);
  //
  // Here we are sending the variable "data".
  //

  SEND_IOT(data);


  delay(100);
}
