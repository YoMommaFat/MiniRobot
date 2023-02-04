/*
file: Tele_04.ino
date: 2023.01.28. - már kevésbé, de még mindig fáj a fogam
auth: INIT - Szakáll Tibor
vers: 1.0
prev: Tele_03.ino
desc: . A két motor quadratura enkóderjének RTOS-task leolvasása könyvtárral
      . Az enkóder adatok a VisiBot v1.2-vel rendben vannak.
      . A két WS2812B GRB LED színváltogatása adafruit könyvtárral
      . A két motor PWM SELECT meghajtása
      . A tele commanding kezdetei serial txt parserrel
      . A Node Red nipple parancsok parserelése, OK
      . USART2 a NR kapcsolat, így az USART0 csak programoz és serial monitorozik. 1000x egyszerűbb így!
      . a cliff szenzorok olvasása és json küldésének implementálása
      . a motor enkóderek adatainak json küldéseének implementálása
      ! az "unknown command" feloldása és returnok bevezetése a parse-oláshoz
      + a bumper, distance szenzorok olvasása és json küldéseének implementálása
      + a áram és feszültség értékek olvasása és json küldéseének implementálása
stat: működik
*/

#include <Arduino_FreeRTOS.h>   // Arduino IDE-ből "FreeRTOS by Richard Barry"

#include <Adafruit_NeoPixel.h>  // Arduino IDE-ből "Adafruit NeoPixel by Adafruit"

#include <CmdParser.hpp>        // Arduino IDE-ből "CmdParser by Pascal Vizeli"
#include <CmdBuffer.hpp>
#include <CmdCallback.hpp>

// a hardver definíció
#include "VisiBot_lib.h"
#include "motor_3.h"

// melyik USART portot hasznalja a NodeRed felé CSAK AZ EGYIK!
#undef NR_USART_0
#define NR_USART_2
// az sima visszajelzesek serial monitorra csak az USART0-an mennek!!!

void TaskBlink1( void *pvParameters );
void TaskBlink2( void *pvParameters );
void Taskprint ( void *pvParameters );

Led led1(LED1);
Led led2(LED2);

RotEnc_L encL;  
RotEnc_R encR; 

Adafruit_NeoPixel strip(LED_COUNT, RGB_LED, NEO_GRB + NEO_KHZ800);

CmdCallback<3> cmdCallback;

CmdBuffer<64> myBuffer;
CmdParser     myParser;

char strHallo[] = "HALLO";     // parse tartalék slot
char strQuit[]  = "QUIT";      // parse tartalék slot
char strSet[]   = "SET";       // a NR parancsok "SET"-el kezdődnek


// jelenleg nincs használatban, bővítési lehetőség ha kell másik parancs csoport a NR->Arduino irányban
// tehát ha nem "SET" kezdetű parancsot akarunk
void functHallo(CmdParser *myParser) { Serial.println("Received Hallo"); }

// jelenleg csak ezt használom CMD parse-olásra NR->Arduino irányban
void functSet(CmdParser *myParser)
{
    //Serial.println("Receive Set");  //debug
    uint32_t color = 0, pwm = 0, pwm2;
    String dummy;    
    
    if (myParser->equalCmdParam(1, "ML")) {           // a LEFT motor kap parancsot
        String dummy = myParser->getCmdParam(2);
        int pwm = dummy.toInt();
        if (pwm<-90) pwm = -90;
        if (pwm>+90) pwm =  90;
       // Serial.print(pwm);      // debug
       // Serial.print(" L ");    // debug
        //void setMotor(int motor_no, int phase_pwm)  // MOTOR_L/MOTOR_R, -100..-1/0/+1..+100
        setMotor(MOTOR_L, pwm); // MOTOR_L/MOTOR_R, -100../0/..100 
        return;            
    }

    if (myParser->equalCmdParam(1, "MR")) {           // a RIGHT motor kap parancsot
        String dummy = myParser->getCmdParam(2);
        int pwm = dummy.toFloat();
        if (pwm<-90) pwm = -90;
        if (pwm>+90) pwm =  90;
       // Serial.print(pwm);     // debug
       // Serial.print(" R ");   // debug
        //void setMotor(int motor_no, int phase_pwm)  // MOTOR_L/MOTOR_R, -100..-1/0/+1..+100
        setMotor(MOTOR_R, pwm); // MOTOR_L/MOTOR_R, -100../0/..100               
        return;
    }

    if (myParser->equalCmdParam(1, "L")) {             // a LEFT RGB_LED kap parancsot
        dummy = myParser->getCmdParam(2);
        color = dummy.toInt();
        // Serial.print("L_rgb -> ");      // debug               
        // Serial.println(color);          // debug
        strip.setPixelColor(LED_LEFT, color);
        strip.show();            
        return;
    }
    
    if (myParser->equalCmdParam(1, "R")) {             // a RIGHT RGB_LED kap parancsot
        dummy = myParser->getCmdParam(2);
        color = dummy.toInt();
        // Serial.print("R_rgb -> ");     // debug    
        // Serial.println(color);         // debug
        strip.setPixelColor(LED_RIGHT, color);
        strip.show();            
        return;
    }

   // Itt további if-ekkel lehet a "SET" parancsok sorát bővíteni, további 2., 3. ... és n. paraméterekkel

    // Command Unknwon
    Serial.println("UNKNOWN command received!");
    Serial.println(myBuffer.getStringFromBuffer()); 
#ifdef NR_USART_2 
    Serial2.println("UNKNOWN command received!");  
#endif  

} // end of functSet -  aka. SET command parser

// jelenleg nincs használatban, bővítési lehetőség ha kell másik parancs csoport a NR->Arduino irányban
// tehát ha nem "SET" kezdetű parancsot akarunk
void functQuit(CmdParser *myParser) { Serial.println("Receive Quit"); }    

void setup() {

  powerOn(); // Turn on self power hold (if JP1 is not used)
  
  analogReference(INTERNAL2V56);
  
 // delay(1000);
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
#ifdef NR_USART_2  
  Serial2.begin(115200);  
#endif

  cmdCallback.addCmd(strHallo, &functHallo);
  cmdCallback.addCmd(strQuit, &functQuit);
  cmdCallback.addCmd(strSet, &functSet);

  // motor 
  motorInit();      // motor pins In/Out
  slowDecayLow();   // Set the decay mode of the motors
  

  //  More custom functions can be found in RGBWstrandtest example sketch
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.setBrightness(203);  // Set BRIGHTNESS (max = 255)  
  strip.clear();           // Turn OFF all pixels
  strip.show();            
  
 // pointer a funkcióra, neve, stack, params/NULL , priority, phandle/NULL
 // prioritás = [1,31] kisebb szám a kisebb prioritás, lehet/kell több egyenlő. 0 = IDLE foglalt!
 // xTaskCreate(TaskBlink1, "task1", 128, NULL, 8, NULL);      // csak debugra használatos
  xTaskCreate(TaskMotor,    "task2", 128, NULL, 4, NULL);        // további motor init, valamint az enkódereket jelentgeti
  xTaskCreate(TaskClif,     "task3", 256, NULL, 2, NULL);        // a clif szenzorokat jelentgeti
  xTaskCreate(TaskBumpDist, "task4", 256, NULL, 2, NULL);        // a bumper és distance szenzorokat jelentgeti
  xTaskCreate(Task_A_U,     "task5", 256, NULL, 2, NULL);        // az áram és feszültséget jelentgeti

// Ez után nem volna szabad több utasítást beírni. Előtte szabad, akár blocking-ot is.
  vTaskStartScheduler();     // ettől kezdve az RTOS uralkodik! 
}

void loop()   // TILOS blocking parancsot beleírni! RTOS miatt. Maradhat üres.
              // jelenleg csak a NR parancsok fogadása és parse-olása miatti utasítás van benne
              // RTOS nem örül, de elviseli, mert nem blocking a hívás
{
#ifdef NR_USART_0 
  cmdCallback.updateCmdProcessing(&myParser, &myBuffer, &Serial);   // Nem fogyaszt ez tul sokat idoben?
#endif  

#ifdef NR_USART_2 
  cmdCallback.updateCmdProcessing(&myParser, &myBuffer, &Serial2);   // Nem fogyaszt ez tul sokat idoben?
#endif  
}

// TASK: további motor init, valamint az enkódereket jelentgeti
void TaskMotor(void *pvParameters)  
{
  Serial.println("Motor task ON");
  Serial.print("ido konstans = ");
  Serial.println(portTICK_PERIOD_MS);  
  motorWake();      // Turn on the motors, azért itt mert van benne delay
  motorEnable(MOTOR_L);
  motorEnable(MOTOR_R);
  setMotor(MOTOR_L, MOTOR_STOP_PWM);    
  setMotor(MOTOR_R, MOTOR_STOP_PWM); 

  static char buf[32]; 
  int left, right;

  while (1)
  {
    left = encL.getCnt();
    right = encR.getCnt();
    sprintf(buf, "{\"cmd\":\"enc\",\"val\":[%d,%d]}", left, right);    // {"enc":[encL,encR]}
   //Serial.println(buf);   //debug

#ifdef NR_USART_0 
    Serial.println(buf);
#endif

#ifdef NR_USART_2 
    Serial2.println(buf);  
#endif          
    pause(500);  //1.2 sec
  }
}  // end of TaskMotor

// TASK: a clif szenzorokat jelentgeti
void TaskClif(void *pvParameters)  
{
  Serial.println("Clif task ON");

  // clif fet power init
  pinMode(CF_PW, OUTPUT);
  pinMode(CR_PW, OUTPUT);
  digitalWrite(CF_PW, LOW);  // power ON
  digitalWrite(CR_PW, LOW);
 
  int CFL, CFM, CFR;
  int CRL, CRM, CRR;
  static char buf[100];   
  const int cPause = 50;  // [ms]

  while(1)
  {
    CFL = analogRead(CF_L);
    pause(cPause);
    CFM = analogRead(CF_M);
    pause(cPause);
    CFR = analogRead(CF_R);
    pause(cPause);
    
    CRL = analogRead(CR_L);
    pause(cPause);
    CRM = analogRead(CR_M);
    pause(cPause);
    CRR = analogRead(CR_R);
    pause(cPause);

    sprintf(buf, "{\"cmd\":\"cliff\",\"val\":[%d,%d,%d,%d,%d,%d]}", CFL, CFM, CFR, CRL, CRM, CRR);    // {"clif":[CFL, CFM, CFR, CRL, CRM, CRR]}
 //   Serial.println(buf);   //debug

#ifdef NR_USART_0 
    Serial.println(buf);
#endif

#ifdef NR_USART_2 
    Serial2.println(buf);  
#endif          
    pause(200);  // 200 [ms]  + (6 * cPause) = 500 [ms]
  }
}  // end of TaskClif


// TASK: a bumper és distance szenzorokat jelentgeti
void TaskBumpDist(void *pvParameters)  
{
  Serial.println("Bumper & Distance task ON");

  // bumper & distance fet power init
  pinMode(BP_PW, OUTPUT);
  pinMode(D_PW, OUTPUT);
  digitalWrite(BP_PW, LOW);  // power ON
  digitalWrite(D_PW, LOW);
 
  int BPL, BPR;
  int DFL, DFM, DFR, DRM;
  static char buf[100];   
  const int bdPause = 100;  // [ms]

  while(1)
  {
    BPL = digitalRead(BP_L);
    pause(bdPause);
    BPR = digitalRead(BP_R);
    pause(bdPause);

    DFL = analogRead(DF_L);
    pause(bdPause);
    DFM = analogRead(DF_M);
    pause(bdPause);
    DFR = analogRead(DF_R);
    pause(bdPause);
    DRM = analogRead(DB_M);
    pause(bdPause);

//    sprintf(buf, "{"cmd":"bump","val":[%d,%d,%d,%d,%d,%d]}", BPL, BPR, DFL, DFM, DFR, DRM);    // {"bump":[BPL, BPR, DFL, DFM, DFR, DRM]}
    sprintf(buf, "{\"cmd\":\"bump\",\"val\":[%d,%d]}", BPL, BPR);    // {"bump":[BPL, BPR, DFL, DFM, DFR, DRM]}
    // Serial.println(buf);   //debug

#ifdef NR_USART_0 
    Serial.println(buf);
#endif

#ifdef NR_USART_2 
    Serial2.println(buf);  
#endif          
    pause(200);  // 400 [ms]  + (6 * bdPause) = 1000 [ms]
  }
}  // end of TaskBumpDist


// TASK: az áram és feszültséget jelentgeti
void Task_A_U(void *pvParameters)  
{
  Serial.println("Current/voltage task ON");

  // clif init
  pinMode(CF_PW, OUTPUT);
  pinMode(CR_PW, OUTPUT);
  digitalWrite(CF_PW, LOW);  // power ON
  digitalWrite(CR_PW, LOW);

#define MR_VP 81 // A12 - Right motor current
#define ML_VP 69 // A0  - Left motor current
#define U_BAT 76 // A7  - Battery voltage
#define I_ARD 75 // A6  - Arduino current
#define I_RPI 74 // A5  - Rasp. PI current
#define RS_OP 73 // A4  - External voltage
 
  int U_ML, U_MR, U__BAT, U_EXT;
  int I__ARD, I__RPI;
  static char buf[100];   
  const int auPause = 100;  // [ms]

  while(1)
  {
    U_ML = analogRead(ML_VP);
    pause(auPause);
    U_MR = analogRead(MR_VP);
    pause(auPause);
    U__BAT = analogRead(U_BAT);
    pause(auPause);
    U_EXT = analogRead(RS_OP);
    pause(auPause);
    
    I__ARD = analogRead(I_ARD);
    pause(auPause);
    I__RPI = analogRead(I_RPI);
    pause(auPause);

    sprintf(buf, "{\"cmd\":\"pwr\",\"val\":[%d,%d,%d,%d,%d,%d]}", U__BAT, U_EXT, I__ARD, I__RPI, U_ML, U_MR);    // {"pwr":[U_ML, U_MR, U__BAT, U_EXT, I__ARD, I__RPI]}
   // Serial.println(buf);   //debug

#ifdef NR_USART_0 
    Serial.println(buf);
#endif

#ifdef NR_USART_2 
    Serial2.println(buf);  
#endif          
    pause(200);  // 2000 [ms]  + (6 * auPause) = 5000 [ms]
  }
}  // end of Task_A_U

//*******************************************************
// only for DEBUG!
// TASK: csak debug célokra használatos, a normál működéskor nincs bekapcsolva, lásd setup()->xTaskCreate()
void TaskBlink1(void *pvParameters)  
{
  Serial.println("Task1 blink");
  uint32_t color0 = 0xA03060;
  uint32_t color1 = 0x1030B0;
  long step = 0x101814;
  while(1)
  {
    led1.on();   
    pause( 150 ); 
    //strip.setPixelColor(0, color0);
    //strip.setPixelColor(1, color1);
    //strip.show();
    color0 += step;
    color1 -= step;
    Serial.println("Task1 blink");
    led1.off();
    pause( 150 ); 
  }
}  // end of TaskBlink1
