# Face-Unlock-Security-System-Esp32-Cam
<!-- BHJ Hlatshwayo -->
<!-- Final Year TUT Computer Science Project -->
To set up this project you will need the following equipment.
1 x Esp32 Cam Ai Thinker 
1 x Pir Sensor
1 x Servo Motor
1 x Red LED 
1 x Green LED 
1 x FTDI Basic USB to TTL Programmer
4 x Female to Female connector
8 x Female to Male connector
2 x Male to Male connector

Circuit board connection on image in documentation

1. Download latest Arduino IDE from https://www.arduino.cc/en/software.
2. Connect the following API's and Libraries from sketch -> include library -> manage libraries in Arduino IDE:
- uTLGBotLib
- UniversalTelegramBot
- Firebase Arduino Client Library for ESP8266 and Esp32
- Esp32WifiManager
- ESP32Servo
- ESP32 Rest Client
- ArduinoJson
3. Project connection:
- Connect GND from Esp32 Cam to breadboard using 1 female to male connector
- Connect 5v from Esp32 Cam to breadboard female to male connector
- Connect female to female from GND  to IO0 on Esp32 Cam
- Connect GND from Pir sensor, Servor Motor, Red and Green LED to GND on breadboard from Esp32 cam using female to male connector for Pir sensor and male to male for the Servo motor
- Connect 5v from Pir sensor, Servor Motor, 5v on breadboard from Esp32 cam using female to male connector and male to male for the Servo motor
- Connect Pir sensor input to GPIO15 on Esp32 Cam using female to female connector
- Connect Servor Motor input to GPIO14 on Esp32 Cam using male to female connector
- Connect Red LED input to GPIO12 on Esp32 Cam using female to female connector
- Connect Green LED input to GPIO12 on Esp32 Cam using female to female connector
- Use female to female connector to connect 5v to FTDI. U0R to TX, U0T to RX
4. Upload code from Arduino IDE to Esp32 from FTDI usb connection


