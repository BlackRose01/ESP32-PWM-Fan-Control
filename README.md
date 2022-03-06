# ESP32-PWM-Fan-Control (WiP)

A small script to control and monitor a 4-pin PWN fan via MQTT. The microcontroller sends the temperature, humidity, fan speed (RPM) and measure timestamp to the MQTT Broker. The speed of the fan (0% - 100%) will be automatically set by the measured temperature and humidity. It will calculate a percent value between a predefined MIN and MAX and afterwards get the mean value of both values.

> Currently Work in Progess (WiP) due to missing microcontroller. First I tried it with an ESP8266 (Wemos D1 Mini and Wemos D1 Mini Pro) but the I2C bus does not recognize connected sensors after the first loop.

## Bill of Material
* ESP32 with WLAN
* Wires
* 2.2 kOhm Resistor
* +5V 4-pin PWM fan (e.g. Noctua NF-12 5V PWM)
* SHT31 (e.g. Adafruit SHT31)

## Wire Plan
A detailed plan how the sensores and devices will be connected can be found in the PDF file in this repository.

## Sources
[Whitepaper Noctua PWM Specifications](https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf)\
[Example PWM Fan Control](https://github.com/jinie/arduino_PwmFanController/blob/master/PwmFanController/PwmFanController.ino)\
[Another Example PWM Fan Control](https://github.com/stefanthoss/esp8266-fan-control)