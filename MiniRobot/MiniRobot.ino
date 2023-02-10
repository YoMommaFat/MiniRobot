#include "VisiBot_lib.h"
#include "Motor_lib.h"
#include <ArduinoJson.h>        // Arduino IDE-ből "ArduinoJson by Benoit Blanchon"
// https://arduinojson.org/v6/how-to/do-serial-communication-between-two-boards/
#include <Arduino_FreeRTOS.h>   // Arduino IDE-ből "FreeRTOS by Richard Barry"
#include <Adafruit_NeoPixel.h>  // Arduino IDE-ből "Adafruit NeoPixel by Adafruit"

#define PAUSE_RCV 20
#define PAUSE_ENC 100
#define PAUSE_CLIFF 100
#define PAUSE_BUMP 100
#define PAUSE_DIST 500
#define PAUSE_PWR 100
#define PAUSE_BLINK 500
#define USART Serial2 // Node-RED serial port
//#define USART Serial

Led led1(LED1);
Led led2(LED2);
RotEnc_L encL;
RotEnc_R encR;
Adafruit_NeoPixel strip(LED_COUNT, RGB_LED, NEO_GRB + NEO_KHZ800);

void setup() {
  mainPowerOn(); // Turn on self power hold (if JP1 is not used)
  auxPowerOn(); // Turn on power for Raspberry Pi
  analogReference(INTERNAL2V56);
  USART.begin(115200);
  Serial.begin(115200);
  motorInit();      // motor pins In/Out
  slowDecayLow();   // Set the decay mode of the motors

  // More custom functions can be found in RGBWstrandtest example sketch
  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.setBrightness(0);   // Set BRIGHTNESS (max = 255)
  strip.clear();            // Turn OFF all pixels
  strip.show();             // Show

  // pointer a funkcióra, neve, stack, params/NULL , priority, phandle/NULL
  // prioritás = [1,31] kisebb szám a kisebb prioritás, lehet/kell több egyenlő. 0 = IDLE foglalt!
  // xTaskCreate(TaskBlink1, "task1", 128, NULL, 8, NULL);    // csak debugra használatos
  xTaskCreate(TaskReceive,"task3", 512, NULL, 6, NULL);       // fogadja a json csomagokat
  xTaskCreate(TaskMotor,  "task2", 256, NULL, 4, NULL);       // motor init, az enkódereket jelentgeti
  xTaskCreate(TaskCliff,  "task4", 256, NULL, 2, NULL);       // a clif szenzorokat jelentgeti
  xTaskCreate(TaskBump,   "task5", 256, NULL, 2, NULL);       // a bumper szenzorokat jelentgeti
  xTaskCreate(TaskDist,   "task6", 256, NULL, 2, NULL);       // a distance szenzorokat jelentgeti
  xTaskCreate(TaskPwr,    "task7", 256, NULL, 2, NULL);       // az áram és feszültséget jelentgeti

  // Ez után nem volna szabad több utasítást beírni. Előtte szabad, akár blocking-ot is.
  vTaskStartScheduler(); // ettől kezdve az RTOS uralkodik! 
}

void loop() {
}

void TaskReceive(void *pvParameters) { // TASK: fogadja a json csomagokat
  (void) pvParameters;
  long color;
  int pwmL, pwmR;
  String command;
  StaticJsonDocument<100> doc; // Allocate the JSON document
  Serial.println("Receive task: running");

  while (1) {
    if (USART.available()) { // Check if the sender is transmitting
      DeserializationError err = deserializeJson(doc, USART); // Read the JSON document
      command = doc["cmd"].as<String>();
      if (err == DeserializationError::Ok) {
        Serial.print("cmd = ");
        Serial.println(command);
        if (command == "mot") {
          pwmL = doc["val"][0].as<int>();
          pwmR = doc["val"][1].as<int>();
          Serial.print("val = "); Serial.print(pwmL);
          Serial.print(", "); Serial.println(pwmR);
          if (pwmL < -90) pwmL = -90; else if (pwmL > 90) pwmL = 90;
          if (pwmR < -90) pwmR = -90; else if (pwmR > 90) pwmR = 90;
          setMotor(MOTOR_L, pwmL);
          setMotor(MOTOR_R, pwmR);
        }
        else if (command == "ledL") {
          color = doc["val"].as<long>();
          Serial.print("val = "); Serial.println(color);
          strip.setPixelColor(LED_LEFT, color); strip.show();
        }
        else if (command == "ledR") {
          color = doc["val"].as<long>();
          Serial.print("val = "); Serial.println(color);
          strip.setPixelColor(LED_RIGHT, color); strip.show();
        }
        else {
          Serial.println("Unknown command received!");
        }
      }
      else {
        Serial.print("deserializeJson() returned "); // Print error to the "debug" serial port
        Serial.println(err.c_str());
        while (USART.available() > 0) { USART.read(); } // Flush all bytes in the serial port buffer
      }
    }
    pause(PAUSE_RCV);
  }
}

void TaskMotor(void *pvParameters) { // TASK: további motor init, valamint az enkódereket jelentgeti
  (void) pvParameters;
  static char buf[32];
  int left, right;
  Serial.print("Time constant: "); Serial.println(portTICK_PERIOD_MS);
  Serial.println("Motor task: running");
  motorWake(); // Turn on motors
  motorEnable(MOTOR_L);
  motorEnable(MOTOR_R);
  setMotor(MOTOR_L, 0);
  setMotor(MOTOR_R, 0);

  while (1) {
    left = encL.getCnt();
    right = encR.getCnt();
    sprintf(buf, "{\"cmd\":\"enc\",\"val\":[%d,%d]}", left, right);
    USART.println(buf);
    pause(PAUSE_ENC);
  }
}

void TaskCliff(void *pvParameters) { // TASK: a clif szenzorokat jelentgeti
  (void) pvParameters;
  int CFL, CFM, CFR;
  int CRL, CRM, CRR;
  static char buf[100];
  Serial.println("Clif task: running");
  pinMode(CF_PW, OUTPUT); // Front cliff FET power init
  pinMode(CR_PW, OUTPUT); // Rear cliff FET power init
  digitalWrite(CF_PW, LOW); // Front cliff FET power ON
  digitalWrite(CR_PW, LOW); // Rear cliff FET power ON

  while(1) {
    CFL = analogRead(CF_L);
    CFM = analogRead(CF_M);
    CFR = analogRead(CF_R);
    CRL = analogRead(CR_L);
    CRM = analogRead(CR_M);
    CRR = analogRead(CR_R);
    sprintf(buf, "{\"cmd\":\"cliff\",\"val\":[%d,%d,%d,%d,%d,%d]}", CFL, CFM, CFR, CRL, CRM, CRR);
    USART.println(buf);  
    pause(PAUSE_CLIFF);
  }
}

void TaskBump(void *pvParameters) { // TASK: a bumper szenzorokat jelentgeti
  (void) pvParameters;
  int BPL, BPR;
  static char buf[100];
  Serial.println("Bumper task: running");
  pinMode(BP_PW, OUTPUT); // Bumper FET power init
  digitalWrite(BP_PW, LOW); // Bumper FET power ON

  while(1) {
    BPL = digitalRead(BP_L);
    BPR = digitalRead(BP_R);
    sprintf(buf, "{\"cmd\":\"bump\",\"val\":[%d,%d]}", BPL, BPR);
    USART.println(buf);
    pause(PAUSE_BUMP);
  }
}

void TaskDist(void *pvParameters) { // TASK: a distance szenzorokat jelentgeti
  (void) pvParameters;
  int DFL, DFM, DFR, DRM;
  static char buf[100];
  Serial.println("Distance task: running");
  pinMode(D_PW, OUTPUT); // Distance FET power init
  digitalWrite(D_PW, LOW); // Distance FET power ON

  while(1) {
    DFL = analogRead(DF_L);
    DFM = analogRead(DF_M);
    DFR = analogRead(DF_R);
    DRM = analogRead(DB_M);
    sprintf(buf, "{\"cmd\":\"dist\",\"val\":[%d,%d,%d,%d]}", DFL, DFM, DFR, DRM);
    USART.println(buf);
    pause(PAUSE_DIST);
  }
}

void TaskPwr(void *pvParameters) { // TASK: az áram és feszültséget jelentgeti
  (void) pvParameters;
  int aML_VP, aMR_VP, aU_BAT, aRS_OP, aI_ARD, aI_RPI;
  static char buf[100];
  Serial.println("Current/voltage task: running");
  pinMode(CF_PW, OUTPUT); // Front power FET power init
  pinMode(CR_PW, OUTPUT); // Rear power FET power init
  digitalWrite(CF_PW, LOW); // Front power FET power ON
  digitalWrite(CR_PW, LOW); // Rear power FET power ON

  while(1) {
    aML_VP = analogRead(ML_VP);
    aMR_VP = analogRead(MR_VP);
    aU_BAT = analogRead(U_BAT);
    aRS_OP = analogRead(RS_OP);
    aI_ARD = analogRead(I_ARD);
    aI_RPI = analogRead(I_RPI);
    sprintf(buf, "{\"cmd\":\"pwr\",\"val\":[%d,%d,%d,%d,%d,%d]}", aU_BAT, aRS_OP, aI_ARD, aI_RPI, aML_VP, aMR_VP);
    USART.println(buf);  
    pause(PAUSE_PWR);
  }
}

void TaskBlink1(void *pvParameters) { // TASK: debug célokra, a normál működéskor nincs bekapcsolva
  (void) pvParameters;
  long color0 = 0xA03060;
  long color1 = 0x1030B0;
  long step = 0x101814;
  Serial.println("Blink task: running");

  while(1) {
    led1.on();
    pause(PAUSE_BLINK);
    //strip.setPixelColor(0, color0);
    //strip.setPixelColor(1, color1);
    //strip.show();
    color0 += step;
    color1 -= step;
    Serial.println("Blink");
    led1.off();
    pause(PAUSE_BLINK);
  }
}
