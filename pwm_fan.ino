/*
 * SOURCES
 *  - https://github.com/stefanthoss/esp8266-fan-control
 *  - https://github.com/jinie/arduino_PwmFanController
 */

#include <Arduino.h>
#include <Wire.h>
// #include <Adafruit_SHT31.h>
#include <ESP8266WiFiMulti.h>
#include <PubSubClient.h>
#include <WiFiClient.h>

#define SHT31_ADDRESS   0x44

#define WIFI_SSID ""
#define WIFI_PASS ""

#define MQTT_SERVER "192.168.0.2"
#define MQTT_PORT 1883
#define MQTT_TOPIC "pflanzi/vitrine"

#define MAX_TEMP 24
#define MIN_TEMP 18
#define MAX_HUM 70
#define MIN_HUM 40

#define PWM_PIN 5
#define RPM_PIN 4
#define MIN_FAN_SPEED_PERCENT 0

bool enableHeater = false;
int counter = 0;
int tmp_speed = 100;
bool tmp_speed_changed = true;

ESP8266WiFiMulti wifiMulti;
WiFiClient net;
PubSubClient mqtt(net);
// Adafruit_SHT31 sht31 = Adafruit_SHT31();

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println();

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

void reconnectMqtt() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect("esp_fan_1")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }

  mqtt.subscribe("pflanzi/vitrine/speed");
}

void setup() {
  Serial.begin(9600);

  // Connect WiFi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASS);
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
  }

  // Connect MQTT
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  reconnectMqtt();

  // Configure SHT31
  // sht31.begin(SHT31_ADDRESS);
  
  pinMode(PWM_PIN, OUTPUT);
  pinMode(RPM_PIN, INPUT);
}

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

void setFanSpeedPercent(int p) {
  int value = (p / 100.0) * 255;
  analogWrite(PWM_PIN, value);
}

int calcFanSpeed(double currTemp, double currHum) {
  double percTemp = 0.0;
  double percHum = 0.0;

  if (currTemp < MIN_TEMP) {
    percTemp = 0;
  } else if (currTemp > MAX_TEMP) {
    percTemp = 100;
  } else {
    percTemp = (100 - MIN_FAN_SPEED_PERCENT) * (currTemp - MIN_TEMP) / (MAX_TEMP - MIN_TEMP) + MIN_FAN_SPEED_PERCENT;
  }

  if (currHum < MIN_HUM) {
    percHum = 0;
  } else if (currHum > MAX_HUM) {
    percHum = 100;
  } else {
    percHum = (100 - MIN_FAN_SPEED_PERCENT) * (currHum - MIN_HUM) / (MAX_HUM - MIN_HUM) + MIN_FAN_SPEED_PERCENT;
  }

  return ((percHum + percTemp) / 2);
}

void mqPublish(const char * topic, String message) {
  char buf[MQTT_MAX_PACKET_SIZE];
  memset(buf, 0, sizeof(buf));
  message.toCharArray(buf, sizeof(buf));

  mqtt.publish(topic, buf, false);
}

String buildMessage(double t, double h, int speed) {
  unsigned long timestamp = 0L;
  
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
  data += String(timestamp);
  data += "}";

  return data;
}

void devices() {
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
 
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
 
void loop() {
  mqtt.loop();
  //devices();
  float currTemp = 0.0; // sht31.readTemperature();
  float currHum = 0.0; // sht31.readHumidity();
  int speed = getFanSpeedRpm() * 1.1;
  // setFanSpeedPercent(tmp_speed);
  mqPublish(MQTT_TOPIC, buildMessage(currTemp, currHum, speed));

  if (tmp_speed_changed) {
    setFanSpeedPercent(tmp_speed);
    tmp_speed_changed = false;
  }

  /*
  if (counter >= 30) {
    enableHeater = !enableHeater;
    sht31.heater(enableHeater);
    counter = 0;
  }

  counter++;
  */

  //delay(1000);
}
