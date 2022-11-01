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

const long interval = 5000;        // Interval at which to publish sensor readings

void getVoltageReadings() {
  float Vvalue=0.0;
  float voltageForIteration = 0;
  /////////////////////////////////////Battery Voltage//////////////////////////////////  
  for(unsigned int i=0;i<20;i++){
    Vvalue=Vvalue+analogRead(A0);         //Read analog Voltage
    delay(5);                             //ADC stable
  }
  
  voltageForIteration=(float)Vvalue/20.0;
  voltageForIteration=(float)voltageForIteration/1024 * 3.300;
  float factor = 5.042;
  if (voltageForIteration < 2.25) {
    factor = 5.089;
  }

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

  // We read the voltage every 500ms of the publish period to get a nice average
  if (currentMillis - previousMillis >= 500) {
    getVoltageReadings();
  }
  
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // get the latest readings of voltage integrated into the average
    getVoltageReadings();
    // reset the counters for the next iteration of voltage readings until publish
    voltReadingCount = 0;
    voltReadingTotal = 0;

    Serial.println(currentVoltage);
    
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_VOLT, 1, true, String(currentVoltage).c_str());
    Serial.println("Published");
  }
}
