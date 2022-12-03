//Junior Workspace

#include <Arduino.h> //library for arduino functions
#include <WiFi.h> //library allows an Arduino board to connect to the internet.
#include <WiFiClientSecure.h> //provides Client SSL to ESP32
// Disable brownout problems
#include "soc/soc.h" 
#include "soc/rtc_cntl_reg.h"

#include "esp_camera.h"//Esp32 Cam library
#include <UniversalTelegramBot.h>//library for using Telegram Bot API
#include <ArduinoJson.h>//a C++ JSON library for Arduino and IoT
#include <ESP32Servo.h>//library for enabling and using the servo motor
#define CAMERA_MODEL_AI_THINKER//Use ESP32 cam
#include "camera_pins.h" //declare camera pins

//I had to define 2 dummy servos.
//So that ESP32Servo API does not interfere with pwm(Pulse Width Modulation) 
//channel and timer used by esp32 camera.
#define DUMMY_SERVO1_PIN 18
#define DUMMY_SERVO2_PIN 19

//defining pins for servo, LED, and PIR sensor
#define SERVOR_PIN 14
#define FLASH_LED_GREEN 13
#define FLASH_LED_RED 12
#define PIR_SENSOR 15

int state = LOW;//state of pir sensor, LOW meaning nothing is detected at first
int val = 0;//set value for pir snsor to 0

//dummy servo declaration
Servo dummyServo1;
Servo dummyServo2;

//servo more declaration
Servo myservo;

//set constant variable for wifi name and password
const char* ssid = "OPPO_HLATSHWAYO_5G";
const char* password = "P@ssword1";

// Initialize Telegram BOT 
String BOTtoken = "5653276233:AAFDFA7VMfWdQeikB1dpO-BHIm4ppy3_cW0";
String CHAT_ID = "1149907942";
bool sendPhoto = false;
boolean matchFace = false;

//call method to start esp32 cam server
void startCameraServer();
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);


bool flashState = LOW;

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

//Telegram connection starts here
void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  //Check if chat_id matches
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    //Code to turn servo to open the door
    if (text == "/unlock") {
      String msg = "Door Unlocked";
      digitalWrite(FLASH_LED_GREEN, true);//turn green led on
      digitalWrite(FLASH_LED_RED, false);//turn red led off
      myservo.write(60);
      bot.sendMessage(CHAT_ID, msg, "");
      Serial.println("Door Unlocked");
      delay(6000);
      msg = "Door Locked";
      digitalWrite(FLASH_LED_RED, true);
      digitalWrite(FLASH_LED_GREEN, false);
      myservo.write(180);
      bot.sendMessage(CHAT_ID, msg, "");
      Serial.println("Door Locked");
    }
    //code to turn servo to close the door
    if (text == "/lock") {
      String msg = "Door Locked";
      digitalWrite(FLASH_LED_RED, true);
      digitalWrite(FLASH_LED_GREEN, false);
      myservo.write(180);
      bot.sendMessage(CHAT_ID, msg, "");
      Serial.println("Door Locked");
    }
    //code to take and send photo to telegram
    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo request");
    }
  }
}

//method to send photo to telegram
String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";//set telegram api variable
  String getAll = "";
  String getBody = "";

  //check if camera has started
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  Serial.println("Connect to " + String(myDomain));

  //check clientTCP connection 
  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");

    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000; 
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}
//Telegram connection end 

//setup method 
void setup() {

  // set servo pins
  dummyServo1.attach(DUMMY_SERVO1_PIN);
  dummyServo2.attach(DUMMY_SERVO2_PIN);
  myservo.attach(SERVOR_PIN, 500, 2400);
  //start servo at 180 to represent 
  myservo.write(180);

  //setting pir sensor pin
  pinMode(PIR_SENSOR, INPUT);

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Set LED as output
  pinMode(FLASH_LED_GREEN, OUTPUT);
  pinMode(FLASH_LED_RED, OUTPUT);
  digitalWrite(FLASH_LED_GREEN, false);
  digitalWrite(FLASH_LED_RED, true);

  //set message that will be sent to be able to use the options
  String welcome = "Welcome, Junior!\n";
  welcome += "Click on one of the following options to interact with the Door System!\n";
  welcome += "/photo : see who is at the door\n";
  welcome += "/unlock : unlock door\n";
  welcome += "/lock : lock door\n";
  bot.sendMessage(CHAT_ID, welcome, "");


  //Arduino configarations 
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  //Wifi connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org

  //check wifi status
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  //Check if facial recognition matchesin order to unlock the door 
  if (matchFace == true) {
    myservo.write(60);
    Serial.print("DOOR UNLOCKED");
    delay(6000);
    myservo.write(180);
    Serial.print("DOOR LOCKED");
  }

  //check if motion is detected and send Alert to user on telegram
  val = digitalRead(PIR_SENSOR);
  if (val == HIGH) {
    delay(10000);
    String welcome = "ALERT!!!!\n";
    welcome += "SOMEONE HAS BEEN DETECTED AT THE DOOR!\n";
    welcome += "/photo : see who is at the door\n";
    welcome += "/unlock : unlock door\n";
    welcome += "/lock : lock door\n";
    bot.sendMessage(CHAT_ID, welcome, "");
    if (state == LOW) {
      Serial.println("Motion detected!");
      state = HIGH;
    }
  }
  else {
    if (state == HIGH) {
      Serial.println("Motion stopped!");
      state = LOW;     
    }
  }

  //send photo to telegram 
  if (sendPhoto) {
    Serial.println("Preparing photo");
    sendPhotoTelegram();
    sendPhoto = false;
  }

  //reset telegram bot to be able get multiple messages
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }

}
