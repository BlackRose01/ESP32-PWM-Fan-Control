# ESP32-PWM-Fan-Control (WiP)

A small script to control and monitor a 4-pin PWN fan via MQTT. The microcontroller sends the temperature, humidity, fan speed (RPM) and measure timestamp to the MQTT Broker. The speed of the fan (0% - 100%) will be automatically set by the measured temperature and humidity. It will calculate a percent value between a predefined MIN and MAX and afterwards get the mean value of both values.

## Bill of Material
* ESP32 with WLAN (ESP32 D1 Mini)
* Wires
* 2.2 kOhm Resistor
* +5V 4-pin PWM fan (e.g. Noctua NF-12 5V PWM)
* SHT31-D (e.g. Adafruit SHT31-D)

## Wire Plan
A detailed plan how the sensores and devices will be connected can be found in the PDF file in this repository.

## Sources
[Whitepaper Noctua PWM Specifications](https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf)\
[Example PWM Fan Control](https://github.com/jinie/arduino_PwmFanController/blob/master/PwmFanController/PwmFanController.ino)\
[Another Example PWM Fan Control](https://github.com/stefanthoss/esp8266-fan-control)