
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
int omegaSpeed = 0;
int omegaDes = 0;
int omegaMax = 18;  // CHANGE THIS VALUE TO YOUR MEASURED MAXIMUM SPEED
int dir = 1;

//int Kp = 80;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
//int Ki = 2;
int pError = 0;

//Setup interrupt variables ----------------------------
volatile int count = 0;                  // encoder count
volatile int sleepMinuteCounter = 0;  // check timer interrupt 1
volatile bool deltaT = false;            // check timer interrupt 2
volatile int shakeMinuteCounter = 0;
int totalInterrupts = 0;  // counts the number of triggering of the alarm
hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t Task1;
TaskHandle_t Task2;

uint32_t x = 0;


//Initialization ------------------------------------
void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  sleepMinuteCounter++;  // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
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



void setup() {
  Serial.begin(115200);

  state = 0;
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(13, OUTPUT);
  // pinMode(encoderY, INPUT);
  // pinMode(encoderW, INPUT);
  Wire.begin(SDA, SCL);
  digitalWrite(13, HIGH);
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

        // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    MQTT_connect();
    Adafruit_MQTT_Subscribe *subscription;
   subscription = mqtt.readSubscription(2000) ;
    if (subscription == &userCommand) {
      Serial.print(F("Got: "));
      Serial.println((char *)userCommand.lastread);
      String setString = (char *)userCommand.lastread;
      int setMinutes = setString.toInt();
      Serial.println("checkpoint 1");
      portENTER_CRITICAL(&timerMux2);
      shakeMinuteCounter = setMinutes;
      portEXIT_CRITICAL(&timerMux2);
      Serial.println("checkpoint 2");
      timerRestart(timer2);
    }
  

    switch (state) {

      case 0:
      core0Sleep();
      break;

      case 1:
        core0Idle();
        break;

        case 2:
        core0Drive();
        break;

    }

    // this is our 'wait for incoming subscription packets' busy subloop
    // try to spend your time here

    vTaskDelay(10);
  }


  vTaskDelete(NULL);
}


void core0Sleep() {
  // if button or knob press, go to state 1

  // if google service, go to state 2
  if (shakeMinuteCounter > 0) {
    state = 2;
  }

  
}

void core0Idle() {
   // if google service, go to state 2
  if (shakeMinuteCounter > 0) {
    state = 2;
  }

}

void core0Drive() {
  // if large button press, go back to state 1

  // if google service cancels, go to state 1
  if (shakeMinuteCounter = 0) {
    state = 1;
  }  

}


void Task2code(void *parameter) {
  // motor core setup function
  motorCoreSetup();

  // motor core loop function
  for (;;) {
    // switch (state) {
    //   case 0:
    //     state0MotorCore();
    //     break;
    //   case 1:
    //     state1MotorCore();
    //     break;
    // }
   
// writeVoltage(150);

    if (shakeMinuteCounter > 0) {
      setSpeed(vDes);
      
    } else {
      timerStop(timer2);

      // po  rtENTER_CRITICAL(&timerMux2);
      //  minuteCounter = 0;
      //   portEXIT_CRITICAL(&timerMux2);
      stopMoving();
      

      // Serial.println(String() + "speed is: "+ count);
    }
    Serial.println(String() + "speed is: " + count);
    // important so that the watchdog bug doesnt get triggered
    Serial.println(shakeMinuteCounter);
    vTaskDelay(10);
  }
  vTaskDelete(NULL);
}


// Motor driving core (core 1) code

void motorCoreSetup() {
  Serial.println("check 1");
  ESP32Encoder::useInternalWeakPullResistors = UP;  // Enable the weak pull up resistors
  encoder.attachHalfQuad(encoderY, encoderW);       // Attache pins for use as encoder pins
  encoder.setCount(0);                              // set starting count value after attaching

  timer0 = timerBegin(0, 80, true);              // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true);  // edge (not level) triggered
  timerAlarmWrite(timer0, 60000000, true);       // 60000000 * 1 us = 60 s, autoreload true
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


void stopMoving() {
  writeVoltage(0);
}

// command core code (core 0)
void commandCoreSetup() {
  Serial.println();
  Serial.println();


  
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  
  WiFi.begin(WLAN_SSID, WLAN_PASS);

  while (WiFi.status() != WL_CONNECTED) {
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&userCommand);

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);
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
}
