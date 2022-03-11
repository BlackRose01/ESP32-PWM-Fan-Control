/*
 * SOURCES
 *  - https://github.com/stefanthoss/esp8266-fan-control
 *  - https://github.com/jinie/arduino_PwmFanController
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESPmDNS.h>
#include <Adafruit_SHT31.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>

#define DNS_NAME "pflanzi-vitrine-fan-1"
#define WIFI_SSID ""
#define WIFI_PASS ""

#define MQTT_SERVER "192.168.0.2"
#define MQTT_PORT 1883
#define MQTT_TOPIC "pflanzi/vitrine"

#define SENSOR_TEMP_TOL 2.0
#define MAX_TEMP 24
#define MIN_TEMP 18
#define MAX_HUM 70
#define MIN_HUM 40

#define PWM_PIN 17
#define RPM_PIN 16
#define MIN_FAN_SPEED_PERCENT 0

#define FAN_SLEEP_START_HOUR 22
#define FAN_SLEEP_STOP_HOUR 7

#define LOOP_DELAY 20

bool enableHeater = false;
int counter = 0;
int tmp_speed = 100;
bool tmp_speed_changed = true;
bool ignore_delay = false;

WiFiClient net;
WiFiUDP ntpUDP;
PubSubClient mqtt(net);
Adafruit_SHT31 sht31 = Adafruit_SHT31();
NTPClient timeClient(ntpUDP);

/**
 * MQTT Callback function for fan speed
 */
void mqttCbSpeed(byte* payload, unsigned int length) {
  String speed = "";
  
  for (int i = 0; i < length; i++) {
    speed = speed + String((char)payload[i]);
  }
  
  tmp_speed = speed.toInt();

  if (tmp_speed > 100) {
    tmp_speed = 100;
  } else if (tmp_speed < 0) {
    tmp_speed = 0;
  }
  
  tmp_speed_changed = true;
}

/**
 * MQTT Callback function for fan speed
 */
void mqttCbIgnoreDelay(byte* payload, unsigned int length) {
  char a = (char)payload[0];

  if (a == '1') {
    ignore_delay = true;
  } else {
    ignore_delay = false;
  }
}

/**
 * Callback for incoming MQTT messages
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (topic == "pflanzi/vitrine/speed") {
     mqttCbSpeed(payload, length);
  } else if (topic == "pflanzi/vitrine/delay") {
    mqttCbIgnoreDelay(payload, length);
  }
}

/**
 * Reconnect to MQTT Broker
 */
void reconnectMqtt() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print(".");
    // Attempt to connect
    if (!mqtt.connect("esp_fan_1")) {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

  mqtt.subscribe("pflanzi/vitrine/speed");
  mqtt.subscribe("pflanzi/vitrine/delay");
}

/**
 * Setup
 */
void setup() {
  // Open Serial
  Serial.begin(9600);

  // Connect WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println("\t[finished]");

  // Configure NTP
  Serial.print("Configure NTP");
  timeClient.begin();
  Serial.println("\t\t[finished]");

  // Register DNS name
  Serial.print("Configure MDNS");
  while(!MDNS.begin(DNS_NAME)) {
    Serial.println("Error setting up MDNS responder!");
    delay(1000);
  }
  Serial.println("\t\t[finished]");

  // Configure OTA
  Serial.print("Configure OTA");
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(DNS_NAME);
  ArduinoOTA.setPasswordHash("ba4a46b504499d817cd51eefc3a1c7be6a957292");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();
  
  Serial.println("\t\t[finished]");


  // Connect MQTT
  Serial.print("Configure MQTT");
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  reconnectMqtt();
  Serial.println("\t\t[finished]");

  // Configure SHT31
  sht31.begin();

  // Configure PWM
  pinMode(PWM_PIN, OUTPUT);
  pinMode(RPM_PIN, INPUT);

  ledcSetup(0, 25000, 8);
  ledcAttachPin(PWM_PIN, 0);
}

/**
 * Returns the speed of the fan
 */
int getFanSpeedRpm() {
  int highTime = pulseIn(RPM_PIN, HIGH);
  int lowTime = pulseIn(RPM_PIN, LOW);
  int period = highTime + lowTime;
  if (period == 0) {
    return 0;
  }
  float freq = 1000000.0 / (float)period;
  return (freq * 60.0) / 2.0; // two cycles per revolution
}

/**
 * Set the speed of the fan
 */
void setFanSpeedPercent(int p) {
  int value = (p / 100.0) * 255;
  ledcWrite(0, value);
}

/**
 * Calculate the new fan speed depending on temperature and humidity
 */
int calcFanSpeed(double currTemp, double currHum) {
  double percTemp = 0.0;
  double percHum = 0.0;

  // Calc temperature
  if (currTemp < MIN_TEMP) {
    percTemp = 0;
  } else if (currTemp > MAX_TEMP) {
    percTemp = 100;
  } else {
    percTemp = (100 - MIN_FAN_SPEED_PERCENT) * (currTemp - MIN_TEMP) / (MAX_TEMP - MIN_TEMP) + MIN_FAN_SPEED_PERCENT;
  }

  // Calc humidity
  if (currHum < MIN_HUM) {
    percHum = 0;
  } else if (currHum > MAX_HUM) {
    percHum = 100;
  } else {
    percHum = (100 - MIN_FAN_SPEED_PERCENT) * (currHum - MIN_HUM) / (MAX_HUM - MIN_HUM) + MIN_FAN_SPEED_PERCENT;
  }

  // Returns the mean value of temperature and humidity
  return ((percHum + percTemp) / 2);
}

/**
 * Publish a MQTT message
 */
void mqPublish(const char * topic, String message) {
  char buf[MQTT_MAX_PACKET_SIZE];
  memset(buf, 0, sizeof(buf));
  message.toCharArray(buf, sizeof(buf));

  mqtt.publish(topic, buf, false);
}

/**
 * Build the JSON message
 */
String buildMessage(double t, double h, int speed) {
  String data = "{";
  data += "\"temperature\": ";
  data += String(t);
  data += ", \"humidity\": ";
  data += String(h);
  data += ", \"speed\": ";
  data += String(speed);
  data += ", \"heater_enabled\": ";
  data += String(enableHeater);
  data += ", \"timestamp\": ";
  data += timeClient.getEpochTime();
  data += "}";

  return data;
}

/**
 * Check if fan should be in standby (turned off)
 */
bool isStandby() {
  // get current hour
  int currHour = timeClient.getHours();

  // check if current hour is in standby time range
  if (currHour >= FAN_SLEEP_START_HOUR || currHour < FAN_SLEEP_STOP_HOUR) {
    return true;
  } else {
    return false;
  }
}
 
void loop() {
  // Update NTP
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }

  // Handle OTA update
  ArduinoOTA.handle();

  // Handle MQTT subscribtion
  if (!mqtt.connected()) {
    reconnectMqtt();
  }
  mqtt.loop();

  // Get current temperature and humidity
  float currTemp = sht31.readTemperature() - SENSOR_TEMP_TOL;
  float currHum = sht31.readHumidity();

  // Check if fan shoudl be in standby and update fan speed
  if (!isStandby()) {
    setFanSpeedPercent(calcFanSpeed(currTemp, currHum));
  } else {
    setFanSpeedPercent(0);
  }
  
  // Get current fan speed and publish MQTT message with current measurements
  int speed = getFanSpeedRpm() * 1.1;
  mqPublish(MQTT_TOPIC, buildMessage(currTemp, currHum, speed));

  if (tmp_speed_changed) {
    setFanSpeedPercent(tmp_speed);
    tmp_speed_changed = false;
  }

  // Activate heater on SHT31 after 30 loops
  if (counter >= 10) {
    enableHeater = !enableHeater;
    sht31.heater(enableHeater);
    counter = 0;
  }

  counter++;

  if (!ignore_delay) {
    delay(LOOP_DELAY * 1000);
  }
}