
/***************************************************
  HUSKYLENS An Easy-to-use AI Machine Vision Sensor
  <https://www.dfrobot.com/product-1922.html>

***************************************************
  This example shows how to play with line tracking.

  Created 2020-03-13
  By [Angelo qiao](Angelo.qiao@dfrobot.com)

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
****************************************************/

/***********Notice and Trouble shooting***************
  1.Connection and Diagram can be found here
  <https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_23>
  2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

// Libraries
#include "SoftwareSerial.h"
#include "PIDConfig.h"
#include <string.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "analogWrite.h"
#include <Wire.h>
#include <LiquidCrystal.h>
#include "AiEsp32RotaryEncoder.h"

// pin definitions
//Define L298N pin mappings
#define IN1 19
#define IN2 18

#define encoderY 16
#define encoderW 17
// #define SDA 23
// #define SCL 22

// LCD screen pins
#define RS 4
#define EN 5
#define D7 21
#define D6 33
#define D5 15
#define D4 27

#define mainButton 32
#define mainButtonLight 25
#define reset 26

#define ROTARY_ENCODER_A_PIN 22
#define ROTARY_ENCODER_B_PIN 14
#define ROTARY_ENCODER_BUTTON_PIN 23
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4


#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE 1
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"

#define ZUMO_FAST 255


// state machine set up
// state 0 default machine follow mode; state 1 manual override
int state;

// 0 for choosing speed, 1 for choosing time
int manualSelectionState;
bool displayUpdate = true;
bool networkEnabled = false;
bool autoComplete = false;

// setting PWM properties ----------------------------
const int MAX_PWM_VOLTAGE = 255;
int NOM_PWM_VOLTAGE = 150;

#include <WiFi.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "credentials.h"
#include "analogWrite.h"
#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif


/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER "io.adafruit.com"

// Using port 8883 for MQTTS
#define AIO_SERVERPORT 8883

/************ Global State (you don't need to change this!) ******************/

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'userCommand' for subscribing.
Adafruit_MQTT_Subscribe userCommand = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ME126");

//instead of changing here, rather change numbers above
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);


// LCD screen
// custom characters doc
//https :forums.parallax.com/uploads/attachments/40445/40177.pdf
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
byte hourglass_down[8] = {
  0b00000,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b01010,
  0b10001,
  0b11111
};

byte hourglass_up[8] = {
  0b00000,
  0b11111,
  0b10001,
  0b01010,
  0b00100,
  0b01110,
  0b11111,
  0b11111
};

byte smiley[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000
};
// byte wifi[8] = {
//   0b00000,
//   0b01110,
//   0b10001,
//   0b00100,
//   0b01010,
//   0b00000,
//   0b00100,
//   0b00000
// };

byte wifi[8] = {
  0b00000,
  0b01110,
  0b10001,
  0b00100,
  0b01010,
  0b00000,
  0b00100,
  0b00000
};

byte heart[8] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};


#define hourglassFlashTime 1500
int hourglassTimeComp = 0;

// io.adafruit.com root CA
const char *adafruitio_root_ca =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
  "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
  "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
  "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
  "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
  "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
  "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
  "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
  "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
  "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
  "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
  "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
  "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
  "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
  "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
  "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
  "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
  "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
  "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
  "-----END CERTIFICATE-----\n";

/****************************** Feeds ***************************************/

// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
// Adafruit_MQTT_Publish test = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/test");





// Motor info
ESP32Encoder encoder;
// ESP32Encoder rotary;
int omegaSpeed = 0;
int omegaDes = 0;
int omegaMax = 18;  // CHANGE THIS VALUE TO YOUR MEASURED MAXIMUM SPEED
int dir = 1;

//int Kp = 80;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
//int Ki = 2;
int pError = 0;

//Setup interrupt variables ----------------------------
volatile int count = 0;  // encoder count
volatile int rotaryCount = 0;
volatile boolean buttonBufferComplete = false;  // check timer interrupt 1
volatile bool deltaT = false;         // check timer interrupt 2
volatile int shakeMinuteCounter = 0;
int shakeMinutePrevCounter = 0;
volatile bool mainButtonIsPressed = false;
volatile bool rotaryButtonIsPressed = false;
int totalInterrupts = 0;  // counts the number of triggering of the alarm

#define speedRampTime 2000
int speedRampTimeComp = 0;

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t Task1;
TaskHandle_t Task2;

uint32_t x = 0;


int userInputSpeedLevel = 1;  // initial setting medium, 0 for low, 2 for high
int userInputTime = 1;        // preset time, in mins

//Initialization ------------------------------------

void IRAM_ATTR isr() {  // the function to be called when interrupt is triggered
  mainButtonIsPressed = true;
  timerRestart(timer0);
}

void IRAM_ATTR rotaryIsr() {
  rotaryButtonIsPressed = true;
}
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  buttonBufferComplete = true;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
  timerStop(timer0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  count = encoder.getCount();
  encoder.clearCount();


  deltaT = true;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR onTime2() {
  portENTER_CRITICAL_ISR(&timerMux2);
  shakeMinuteCounter--;
  portEXIT_CRITICAL_ISR(&timerMux2);
}

void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}


void setup() {
  Serial.begin(115200);

  state = 0;
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(mainButtonLight, OUTPUT);
  pinMode(mainButton, INPUT_PULLDOWN);
  pinMode(reset, INPUT_PULLDOWN);
  // pinMode(rotaryBTN, INPUT_PULLDOWN);
  attachInterrupt(mainButton, isr, RISING);
  // attachInterrupt(rotaryBTN, rotaryIsr, RISING);

  Wire.begin(SDA, SCL);
  digitalWrite(13, HIGH);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  bool circleValues = true;
  // rotaryEncoder.setBoundaries(1, 60, circleValues); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without accelerateion you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it

  // create a new character
  lcd.createChar(0, hourglass_down);
  // create a new character
  lcd.createChar(1, hourglass_up);
  // create a new character
  lcd.createChar(2, smiley);
  lcd.createChar(3, wifi);
  lcd.createChar(4, heart);

 Serial.println();
  Serial.println();

  lcd.setCursor(3, 0);
  lcd.print("Welcome! ");
  lcd.write(2);
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");


  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
if (hotspot) {
WiFi.mode(WIFI_STA);
}

  WiFi.begin(WLAN_SSID, WLAN_PASS);

int connectTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - connectTime >= CONNECTION_TIMEOUT) {
      break;

  } 
  }

if (WiFi.status() == WL_CONNECTED) {
  Serial.println();


  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&userCommand);

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);
  // networkEnabled = true;
  
} else {
  networkEnabled = false;
  state = 1;
}

  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(100);
  //
  //
  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(100);
}


void Task1code(void *pvParameters) {

  // the setup function on command core
  commandCoreSetup();

  // the loop function on command core
  while (1) {
    if (digitalRead(reset) == HIGH) {
      ESP.restart();

    };
    if (!networkEnabled && state == 0) {

    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();
    }
    if (networkEnabled) {
      MQTT_connect();
    Adafruit_MQTT_Subscribe *subscription;
    subscription = mqtt.readSubscription();
    if (subscription == &userCommand) {
      Serial.print(F("Got: "));
      Serial.println((char *)userCommand.lastread);
      String setString = (char *)userCommand.lastread;
      int setMinutes = setString.toInt();
      startShakerCounting(setMinutes);
    }
  }


    if (rotaryEncoder.isEncoderButtonClicked()) {
      rotary_onButtonClick();
    }
    // Serial.println(rotaryCount);
  
      if (count > 0) {
      turnButtonLightOn();
    } else {
      turnButtonLightOff();
    }


    switch (state) {

      case 0:
        commandCoreStartUp();
        break;

      case 1:
        commandCoreIdle();
        break;

      case 2:
        commandCoreDrive();
        break;
    }

    // this is our 'wait for incoming subscription packets' busy subloop
    // try to spend your time here

    vTaskDelay(10);
  }


  vTaskDelete(NULL);
}


void commandCoreStartUp() {
  // if button or knob press, go to state 1
  if (mainButtonPressed()) {
    mainButtonIsPressed = false;
    state = 1;
  }


  // if google service, go to state 2
  if (shakeMinuteCounter > 0) {
    state = 2;
    speedRampTimeComp = millis();
  }
}

void commandCoreIdle() {

  if (rotaryButtonIsPressed && mainButtonPressed()) {
    ESP.restart();
    
  }
  // if google service calls, go to state 2
  if (shakeMinuteCounter > 0) {
    state = 2;
    startShakerCounting(shakeMinuteCounter);
    speedRampTimeComp = millis();
    // displayUpdate = true;
  }
  if (mainButtonPressed()) {
    state = 2;
    speedRampTimeComp = millis();
    startShakerCounting(userInputTime);
    mainButtonIsPressed = false;
    // shakeMinuteCounter = userInputTime;
  }
  bool circleValues = true;
  switch (manualSelectionState) {
    case 0:  // speed selection
      circleValues = false;
      rotaryEncoder.setBoundaries(1, 3, circleValues);
      rotaryEncoder.disableAcceleration();

      if (rotaryEncoder.encoderChanged()) {
        rotaryCount = rotaryEncoder.readEncoder();
        displayUpdate = true;
      }
      break;
    case 1:  // time selection
      rotaryEncoder.setBoundaries(1, 61, circleValues);
      rotaryEncoder.setAcceleration(15);  //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration
      if (rotaryEncoder.encoderChanged()) {
        userInputTime = rotaryEncoder.readEncoder();
        displayUpdate = true;
      }
  }

  if (rotaryButtonIsPressed) {
    rotaryButtonIsPressed = false;
    Serial.println("rotaryButtonPressed");
    displayUpdate = true;
    if (manualSelectionState == 0) {
      manualSelectionState = 1;

    } else {
      manualSelectionState = 0;
    }
  }


  // // check time, if larger than 5 mins, turn off display
  // if (millis() - hourglassTimeComp >= 5000) {
  //   state = 0;
  //   lcd.clear();
  // }
}

void commandCoreDrive() {
  // if large button press, go back to state 1
  if (mainButtonPressed()) {
    // if (rotaryButtonIsPressed) {
  //  rotaryButtonIsPressed = false;
    mainButtonIsPressed = false;
    state = 1;
    displayUpdate = true;
    stopShakerCounting();
    // stopShakerCounting();
  }

  // if google service cancels, go to state 1
  if (shakeMinuteCounter == 0) {
    state = 1;
    displayUpdate = true;
    autoComplete = true;
    stopShakerCounting();
  }
}

void stopShakerCounting() {
  lcd.clear();
  // timerStop(timer2);
  portENTER_CRITICAL_ISR(&timerMux2);
  shakeMinuteCounter = 0;
  portEXIT_CRITICAL_ISR(&timerMux2);
  shakeMinutePrevCounter = 0;
  timerStop(timer2);
}

void startShakerCounting(int setMinutes) {
  lcd.clear();
  // displayUpdate
  portENTER_CRITICAL(&timerMux2);
  shakeMinuteCounter = setMinutes;
  portEXIT_CRITICAL(&timerMux2);
  shakeMinutePrevCounter = shakeMinuteCounter;
  timerRestart(timer2);
  displayUpdate = true;
}


void Task2code(void *parameter) {
  // motor core setup function
  motorCoreSetup();
  // Serial.println(state);

  // motor core loop function
  for (;;) {
    Serial.println(state);
  

    // motor core does modify state, only observe
    switch (state) {

      case 0:
        // sleeping
        motorCoreStartUp();
        break;

      case 1:
        // idle
        motorCoreIdle();
        break;

      case 2:
        // drive
        motorCoreDrive();
        break;
    }
    vTaskDelay(10);
  }
  vTaskDelete(NULL);
}

void motorCoreStartUp() {
  stopMoving();
  // display off function below
  // lcd.noDisplay();
  lcd.setCursor(3, 0);
  lcd.print("Welcome! ");
  lcd.write(2);
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
}

void motorCoreIdle() {
  stopMoving();
  if (autoComplete) {
    autoComplete = false;
    lcd.clear();
    int displayCompleteTime = millis();
    while (millis() - displayCompleteTime <= 2000) {
      lcd.setCursor(0,0);
      lcd.print("Shake complete!");
      lcd.write(4);
  lcd.setCursor(3,1);
  lcd.print("Hold on...");      
      
    }
  }
  if (displayUpdate) {
    lcd.clear();
    // lcd.display();
    displayUpdate = false;
    // display on
    lcd.setCursor(0, 0);
    lcd.print("Speed: ");

    // rotaryEncoder.setEncoderValue(1);
    switch (rotaryCount) {
      case 1:
        lcd.print("Low");
        break;
      case 2:
        lcd.print("Med");
        break;
      default:
        lcd.print("High");
        break;
    }
    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(userInputTime);
    lcd.print(" m");

    lcd.setCursor(14, 1);
    lcd.write(3);
    if(!networkEnabled) {
      lcd.print("x");
    }


    switch (manualSelectionState) {
      case 0:
        lcd.setCursor(7, 0);


        break;
      case 1:
        lcd.setCursor(6, 1);

        break;
    }


    // display text
  }
  lcd.blink();
}
void motorCoreDrive() {
  // display new set text
  // lcd.display();


  // if (shakeMinuteCounter == 0) {
  //   shakeMinuteCounter = userInputTime;
  // }
  int currentTime = millis();

  if (currentTime - speedRampTimeComp <= speedRampTime) {
    vDes = 14;
  } else {
    switch (rotaryCount) {
      case 1:
        vDes = 15;
        break;
      case 2:
        vDes = 17;
        break;
      case 3:
        vDes = MAX_VDES;
        break;
      default:
        vDes = NOM_VDES;
        break;
    }
  }
  // update screen once every minute
  if (shakeMinuteCounter - shakeMinutePrevCounter != 0) {
    lcd.clear();
    shakeMinutePrevCounter = shakeMinuteCounter;
  }
  // set the cursor to the top left
  lcd.setCursor(0, 0);
  lcd.write(2);  // write in the smiley face
  lcd.print(" Shaking");

  lcd.print("...");
lcd.setCursor(14, 0);
lcd.write(3);
if (!networkEnabled) {
lcd.print("x");  
}

  lcd.setCursor(0, 1);

  if (currentTime - hourglassTimeComp < hourglassFlashTime / 2) {
    lcd.write(byte(0));

  } else if (currentTime - hourglassTimeComp < hourglassFlashTime) {
    lcd.write(byte(1));
  }
  if (currentTime - hourglassTimeComp >= hourglassFlashTime) {
    hourglassTimeComp = currentTime;
  }

  lcd.print(" Time Rem: ");
  if (shakeMinuteCounter >= 60) {
    int displayTime = shakeMinuteCounter / 60;
    lcd.print(displayTime);

    lcd.setCursor(15, 1);
    lcd.print("h");
  } else {
    lcd.print(shakeMinuteCounter);
    lcd.setCursor(15, 1);
    lcd.print("m");
  }

  // set speed
  setSpeed(vDes);
}


// Motor driving core (core 1) code

void motorCoreSetup() {
  Serial.println("check 1");



  ESP32Encoder::useInternalWeakPullResistors = UP;  // Enable the weak pull up resistors
  encoder.attachHalfQuad(encoderY, encoderW);       // Attache pins for use as encoder pins
  encoder.setCount(0);                              // set starting count value after attaching
  // rotary.attachHalfQuad(rotaryA, rotaryB);       // Attache pins for use as encoder pins
  // rotary.setCount(0);                              // set starting count value after attaching

// button debounce 
  timer0 = timerBegin(0, 80, true);              // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true);  // edge (not level) triggered
  timerAlarmWrite(timer0, 200000, true);       // 200000 * 1 us = 200 ms, autoreload true
  timerAlarmEnable(timer0);                      // enable

  timer1 = timerBegin(1, 80, true);              // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true);  // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true);          // 10000 * 1 us = 10 ms, autoreload true
  timerAlarmEnable(timer1);                      // enable



  timer2 = timerBegin(2, 80, true);              // timer 2, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer2, &onTime2, true);  // edge (not level) triggered
  timerAlarmWrite(timer2, 60000000, true);       // 60000000 * 1 us = 60 s, autoreload true
  timerAlarmEnable(timer2);                      // enable
}








// driving functions

void stopMoving() {
  writeVoltage(0);
}

// command core code (core 0)
void commandCoreSetup() {

 

}



void loop() {
}


void setSpeed(int inputVDes) {
  if (deltaT) {

    portENTER_CRITICAL(&timerMux1);
    deltaT = false;
    portEXIT_CRITICAL(&timerMux1);

    int vDifference = inputVDes - count;
    int dError = vDifference - prevDifference;

    prevDifference = vDifference;
    iError += vDifference;


    if (abs(iError) >= sIMax) {
      if (iError < 0) {
        iError = -sIMax;
      } else {
        iError = sIMax;
      }
    }

    int D = calculateD(vDifference, iError, dError);

    writeVoltage(D);
  }
}

void writeVoltage(int D) {
  if (D >= MAX_PWM_VOLTAGE) {
    D = MAX_PWM_VOLTAGE;
  }
  if (D <= -MAX_PWM_VOLTAGE) {
    D = -MAX_PWM_VOLTAGE;
  }

  if (D > 0) {
    analogWrite(IN1, LOW);
    analogWrite(IN2, D);
  } else if (D < 0) {
    analogWrite(IN1, -D);
    analogWrite(IN2, LOW);
  } else {
    analogWrite(IN1, LOW);
    analogWrite(IN2, LOW);
  }
}

void printSpeed() {
  Serial.println(String() + F("speed is: ") + count);
}

int calculateD(int difference, int iError, int dError) {
  return Ksp * difference + Ksi * iError + Ksd * dError;
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  } 

networkEnabled = false;
  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 2 seconds...");
    mqtt.disconnect();
    delay(2000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1)
        ;
    }
  }

  Serial.println("MQTT Connected!");
  networkEnabled = true;
  state = 1;
}

void rotary_onButtonClick() {
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 150) {
    rotaryButtonIsPressed = false;
    return;
  }
  lastTimePressed = millis();
  rotaryButtonIsPressed = true;
  displayUpdate = true;
  Serial.print("button pressed ");
  Serial.print(millis());
  Serial.println(" milliseconds after restart");
}

bool mainButtonPressed() {
  if (buttonBufferComplete) {
    portENTER_CRITICAL_ISR(&timerMux0);
  buttonBufferComplete = false;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
    return mainButtonIsPressed;
  } else {
    return false;
  }
}

void turnButtonLightOn() {
digitalWrite(mainButtonLight, HIGH);  
}

void turnButtonLightOff() {
  digitalWrite(mainButtonLight, LOW);
}
