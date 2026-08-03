#ifndef EASY_IOT_H
#define EASY_IOT_H

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

/*
  ======================================================
  EasyIoT.h — Simple ThingSpeak Sender for ESP32
  ======================================================

  This file automatically handles:

    1. Connecting the ESP32 to Wi-Fi
    2. Reconnecting if Wi-Fi is disconnected
    3. Sending sensor data to ThingSpeak
    4. Maintaining the required upload interval

  Students normally do not need to change this file.

  Use only these two commands in the main program:

    IOT_BEGIN(WIFI_SSID, WIFI_PASSWORD, THINGSPEAK_KEY);

    SEND_IOT(data);

  SEND_IOT() can send int, float and other numeric values.
*/


// ======================================================
// ThingSpeak server address
// ======================================================

static const char* IOT_SERVER =
    "https://api.thingspeak.com/update";


// ======================================================
// Time between two uploads
//
// ThingSpeak does not allow data to be sent continuously.
// EasyIoT waits 20 seconds between uploads automatically.
// ======================================================

static const unsigned long IOT_UPLOAD_INTERVAL = 20000;


// ======================================================
// Internal EasyIoT variables
//
// These variables remember the Wi-Fi details and API key
// received from IOT_BEGIN().
//
// Students do not need to use these variables directly.
// ======================================================

static const char* _iot_ssid = nullptr;
static const char* _iot_password = nullptr;
static const char* _iot_api_key = nullptr;


// These variables control the upload timing.
static unsigned long _iot_last_upload = 0;
static bool _iot_upload_attempted = false;


// ======================================================
// IOT_BEGIN()
//
// Connects the ESP32 to Wi-Fi.
//
// Use this command once inside setup():
//
// IOT_BEGIN(WIFI_SSID, WIFI_PASSWORD, THINGSPEAK_KEY);
// ======================================================

inline void IOT_BEGIN(const char* ssid,
                      const char* password,
                      const char* apiKey)
{
    // Remember the information provided by the student.
    _iot_ssid = ssid;
    _iot_password = password;
    _iot_api_key = apiKey;


    // Put the ESP32 Wi-Fi into Station Mode.
    // Station Mode allows it to connect to a router
    // or mobile hotspot.
    WiFi.mode(WIFI_STA);

    // Automatically reconnect if Wi-Fi is lost.
    WiFi.setAutoReconnect(true);

    // Do not permanently save Wi-Fi information
    // inside the ESP32 memory.
    WiFi.persistent(false);


    Serial.print("Connecting to WiFi");

    WiFi.begin(_iot_ssid, _iot_password);

    unsigned long startTime = millis();


    // Wait for Wi-Fi connection.
    // Stop trying after 20 seconds so the ESP32
    // does not remain stuck here forever.
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startTime < 20000)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();


    // Show the Wi-Fi connection result.
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi Connected!");

        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("WiFi connection failed.");
        Serial.println("Check Wi-Fi name and password.");
    }
}


// ======================================================
// INTERNAL FUNCTION: _iot_reconnect()
//
// Checks whether Wi-Fi is still connected.
//
// If Wi-Fi is disconnected, EasyIoT tries to reconnect
// for up to 10 seconds.
//
// Students do not need to call this function.
// ======================================================

inline bool _iot_reconnect()
{
    // Wi-Fi is already connected.
    if (WiFi.status() == WL_CONNECTED)
        return true;


    Serial.println("WiFi disconnected. Reconnecting...");

    WiFi.disconnect();
    WiFi.begin(_iot_ssid, _iot_password);

    unsigned long startTime = millis();


    // Try reconnecting for up to 10 seconds.
    while (WiFi.status() != WL_CONNECTED &&
           millis() - startTime < 10000)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();


    // Return true when reconnection is successful.
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi reconnected.");
        return true;
    }


    // Return false when reconnection fails.
    Serial.println("WiFi reconnection failed.");
    return false;
}


// ======================================================
// SEND_IOT()
//
// Sends one numeric value to ThingSpeak Field 1.
//
// Examples:
//
// SEND_IOT(temperature);
// SEND_IOT(distance);
// SEND_IOT(light);
// SEND_IOT(sensorValue);
//
// This function may be placed directly inside loop().
// EasyIoT automatically sends only once every 20 seconds.
// ======================================================

template <typename T>
inline void SEND_IOT(T value)
{
    // Send the first value immediately.
    //
    // After the first upload attempt, wait 20 seconds
    // before allowing another upload.
    if (_iot_upload_attempted &&
        millis() - _iot_last_upload < IOT_UPLOAD_INTERVAL)
    {
        return;
    }


    // Remember when this upload attempt started.
    //
    // This also prevents repeated requests if the API key
    // is wrong or the ThingSpeak server rejects the data.
    _iot_upload_attempted = true;
    _iot_last_upload = millis();


    // Check the Wi-Fi connection before uploading.
    if (!_iot_reconnect())
    {
        Serial.println("IoT Upload Failed: No WiFi");
        return;
    }


    // Create a secure internet connection.
    WiFiClientSecure secureClient;

    // Use a simple HTTPS connection suitable for
    // classroom and educational projects.
    secureClient.setInsecure();


    // Create the HTTP request.
    HTTPClient http;


    // Start the connection with ThingSpeak.
    if (!http.begin(secureClient, IOT_SERVER))
    {
        Serial.println(
            "IoT Upload Failed: Could not start HTTPS"
        );

        return;
    }


    // Stop waiting if the server does not respond
    // within 10 seconds.
    http.setConnectTimeout(10000);
    http.setTimeout(10000);


    // Tell ThingSpeak what type of data is being sent.
    http.addHeader(
        "Content-Type",
        "application/x-www-form-urlencoded"
    );


    // Prepare the message for ThingSpeak.
    //
    // It contains:
    //   - The Write API Key
    //   - The value for Field 1
    String payload = "api_key=";
    payload += _iot_api_key;
    payload += "&field1=";
    payload += String(value);


    Serial.print("Sending to ThingSpeak: ");
    Serial.println(value);


    // Send the data to ThingSpeak.
    int httpCode = http.POST(payload);

    String response = "";


    // Read the reply received from ThingSpeak.
    if (httpCode > 0)
    {
        response = http.getString();
        response.trim();
    }


    // HTTP code 200 and a non-zero response means that
    // ThingSpeak successfully stored the data.
    //
    // The response contains the new entry number.
    if (httpCode == HTTP_CODE_OK && response != "0")
    {
        Serial.print("IoT Data Sent Successfully");
        Serial.print(" | Entry Number: ");
        Serial.println(response);
    }

    // ThingSpeak returns 0 when it rejects the update.
    //
    // Common causes:
    //   - Incorrect Write API Key
    //   - Data sent too frequently
    else if (httpCode == HTTP_CODE_OK &&
             response == "0")
    {
        Serial.println("ThingSpeak rejected the data.");
        Serial.println(
            "Check the Write API Key and upload interval."
        );
    }

    // The server responded, but an HTTP error occurred.
    else if (httpCode > 0)
    {
        Serial.print("ThingSpeak HTTP Error: ");
        Serial.println(httpCode);

        Serial.print("Server Response: ");
        Serial.println(response);
    }

    // The ESP32 could not connect to the server.
    else
    {
        Serial.print("Connection Error: ");
        Serial.println(
            HTTPClient::errorToString(httpCode)
        );
    }


    // Close the internet connection and release memory.
    http.end();
}

#endif
