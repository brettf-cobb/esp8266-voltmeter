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
// Define these in arduino_secrets.h:
// #define WIFI_SSID
// #define WIFI_PASSWORD
// #define MQTT_USER
// #define MQTT_PASS

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 68, 102)
#define MQTT_PORT 1883
#define MQTT_PUB_VOLT "/esp/voltmeter_12v_01/voltage"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last publish
unsigned long previousVoltMillis = 0; // Stores last voltage update
unsigned int voltReadingCount = 0; // Stores number of times voltage was added to voltReadingTotal since update
float voltReadingTotal = 0; // Stores the voltReadingTotal for use in average calculation
float currentVoltage = 0;

const long interval = 60000;
const long voltageReadingInterval = 14000;
const int voltageReadingsPerLoop = 20;

void getVoltageReadings() {
  float rawValue=0.0;
  float voltageForIteration = 0;
  float rawVoltageReadings[20] = {};
  /////////////////////////////////////Battery Voltage//////////////////////////////////  
  for(unsigned int i=0;i<voltageReadingsPerLoop;i++){
    float analogInput = analogRead(A0);
    rawVoltageReadings[i] = analogInput;         //Read analog Voltage
    delay(10);                                    //ADC stable
  }

  // Sort Ascending
  quickSort(rawVoltageReadings, 0, voltageReadingsPerLoop - 1);

  unsigned int valueCount = 0;
  // split readings into n chuncks and use innermost chunck values for calculation
  unsigned int chunkSize = voltageReadingsPerLoop/5;
  for (unsigned int i=(chunkSize * 3) - 1; i < (chunkSize*4)-1; i++) {
    rawValue=rawValue + rawVoltageReadings[i];
    valueCount=valueCount+1;
  }
  
  voltageForIteration=(float)rawValue/valueCount;
  voltageForIteration=(float)voltageForIteration/1024 * 3.30;
  float factor = 4.89651;

  voltReadingCount = voltReadingCount + 1;
  voltReadingTotal = voltReadingTotal + (float)(voltageForIteration * factor);
  currentVoltage = voltReadingTotal / voltReadingCount;
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

  // We read the voltage every 10000ms of the publish period to get a nice average
  if (currentMillis - previousVoltMillis >= voltageReadingInterval) {
    getVoltageReadings();
    previousVoltMillis = currentMillis;
  }
  
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // reset the counters for the next iteration of voltage readings until publish
    voltReadingCount = 0;
    voltReadingTotal = 0;

    Serial.println(currentVoltage);
    
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_VOLT, 1, true, String(currentVoltage).c_str());
    Serial.println("Published");
  }
}
