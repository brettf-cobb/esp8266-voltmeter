/*
  Brett Fernandes, based on

      https://RandomNerdTutorials.com/esp8266-nodemcu-mqtt-publish-bme680-arduino/
  AND
      https://www.engineersgarage.com/nodemcu-battery-voltage-monitor/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>

#include "arduino_secrets.h"
// -- Define these in arduino_secrets.h:
// WIFI_SSID
// WIFI_PASSWORD
// MQTT_USER
// MQTT_PASS


// #include "config12v.h"
#include "config24v.h"
// -- Define the following settings
// MQTT_PUB_VOLT
//    The mqtt topic to publish to
// VOLTAGE_REFERENCE
//    Internal voltage reference for ADC
//    Calculated by applying the input voltage (as read by a multimeter) (V), and the read ADC value for this voltage (ADC) in the following formula:
//      VOLTAGE_REFERENCE = ADC / (1023 * V)
// DIVIDER_FACTOR
//     The input (VIN) to output (VOUT) ratio of the voltage divider circuit
//     DIVIDER_FACTOR = VIN / VOUT


// MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 68, 102)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last publish
unsigned long previousVoltMillis = 0; // Stores last voltage update time
unsigned int voltReadingCount = 0; // Stores number of times voltage was added to voltReadingTotal since update
float voltReadingTotal = 0; // Stores the voltReadingTotal for use in average calculation
float currentVoltage = 0;

const long interval = 60000; // Publish frequency
const long voltageReadingInterval = 14000; // voltage reading frequency
const int voltageReadingsPerLoop = 20; // amount of ADC readings to use per voltage reading

void getVoltageReadings() {
  float rawValue=0.0;
  float voltageForIteration = 0;
  float rawVoltageReadings[20] = {};

  unsigned long lastReading = millis();

  unsigned int i = 0;
  while (i < voltageReadingsPerLoop) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReading >= 10) { //No delay() to prevent inaccurate readings
      float analogInput = analogRead(A0);
      rawVoltageReadings[i] = analogInput;
      lastReading = currentMillis;
      i += 1;
    }
  }

  // Sort Ascending
  quickSort(rawVoltageReadings, 0, voltageReadingsPerLoop - 1);

  unsigned int valueCount = 0;
  // split readings into n chuncks and use innermost chunck values for calculation (median)
  unsigned int chunkSize = voltageReadingsPerLoop/5;
  for (unsigned int i=(chunkSize * 3) - 1; i < (chunkSize*4)-1; i++) {
    rawValue=rawValue + rawVoltageReadings[i];
    valueCount=valueCount+1;
  }

  voltageForIteration=rawValue/(float)valueCount;
  voltageForIteration=(float)voltageForIteration/1023 * VOLTAGE_REFERENCE;
  float factor = DIVIDER_FACTOR;

  voltReadingCount = voltReadingCount + 1;
  voltReadingTotal = voltReadingTotal + (float)(voltageForIteration * factor);
  currentVoltage = voltReadingTotal / voltReadingCount;
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(9600);
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();

  // every X number of milliseconds, read the voltage and average it into all voltage readings between publishes
  if (currentMillis - previousVoltMillis >= voltageReadingInterval) {
    getVoltageReadings();
    // save the last time a voltage reading was taken
    previousVoltMillis = currentMillis;
  }

  // Every X number of milliseconds, publish a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // reset the counters for the next iteration of voltage readings until publish
    voltReadingCount = 0;
    voltReadingTotal = 0;

    Serial.println(currentVoltage);
    mqttClient.publish(MQTT_PUB_VOLT, 1, true, String(currentVoltage).c_str());
  }
}

void quickSort(float output[], int lowerIndex, int higherIndex) {
 int i = lowerIndex;
 int j = higherIndex;
 int pivot = output[lowerIndex + (higherIndex - lowerIndex) / 2];
 while (i <= j) {
   while (output[i] < pivot) {
     i++;
   }
   while (output[j] > pivot) {
     j--;
   }
   if (i <= j) {
     exchangeNumbers(output, i, j);
     i++;
     j--;
   }
 }
 if (lowerIndex < j)
   quickSort(output, lowerIndex, j);
 if (i < higherIndex)
   quickSort(output, i, higherIndex);
}

void exchangeNumbers(float output[], int i, int j) {
 int temp = output[i];
 output[i] = output[j];
 output[j] = temp;
}
