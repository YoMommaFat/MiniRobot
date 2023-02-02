/*
file: VisiBot_lib.h
date: 2022.08.12.
auth: INIT - Szakáll Tibor
vers: 1.3  2023.01.28.
prev: Sinkovits Béla, "MiniRobot_1.5.ino"
desc: + A VisiBot robot hardwer definíciója
      + A motor enkóderekhez osztály 
stat: OK
*/

#ifndef __VISIBOT_LIB_H
#define __VISIBOT_LIB_H

#include <arduino.h>
#include <Arduino_FreeRTOS.h>   // Arduino IDE-ből "FreeRTOS by Richard Barry"

#define RTOS_DELAY              // a Pause függvénynek a delay vs vTaskDelay feloldasához kell

//**************************
//Adafruit GRB LED
#define LED_COUNT 2 //RGB pixel count
#define LED_RIGHT 0
#define LED_LEFT  1

//**************************
// VisiBot HW
#define L_DAT 3         // RGB LED
#define RGB_LED L_DAT   // szinoníma
#define LED1 57 // LED 1
#define LED2 58 // LED 2

//**************************
// Encoders
// Left encoder
#define EL_A 7
#define EL_B 6
// Right encoder
#define ER_A 4
#define ER_B 5
#define ENC_INDEX_MAX 8   // az ISR miatt kell

#define PAR_BTN 50 // Power button
#define PAR_HLD 51 // Power hold

//**************************
// CLIF sensor
// clif sensors power pins
#define CF_PW 61 // Front cliff sensors power
#define CR_PW 27 // Rear cliff sensors power
// cliff sensors ADC pins
#define CF_L 70 // A1  - Left-front cliff
#define CF_M 77 // A8  - Middle-front cliff
#define CF_R 80 // A11 - Right-front cliff
#define CR_L 82 // A13 - Left-rear cliff
#define CR_M 84 // A15 - Middle-rear cliff
#define CR_R 83 // A14 - Right-rear cliff

//**************************
// BUMPER sensor
// bumper sensors power
#define BP_PW 62 // Bumper sensors power
// bumper sensors pins
#define BP_L 2  // Left bumper
#define BP_R 85 // Right bumper

//**************************
// DISTANCE sensors
// distance sensors power
#define D_PW 63 // Distance sensors power
// distance sensors ADC pins
#define DF_L 78 // A9  - Left-front distance
#define DF_M 79 // A10 - Middle-front distance
#define DF_R 71 // A2  - Right-front distance
#define DB_M 72 // A3  - Middle-back distance (REAR)

//**************************
// Voltage and current sensors ADC pins
#define ML_VP 69 // A0  - Left motor current
#define MR_VP 81 // A12 - Right motor current
#define U_BAT 76 // A7  - Battery voltage
#define RS_OP 73 // A4  - External voltage
#define I_ARD 75 // A6  - Arduino current
#define I_RPI 74 // A5  - Rasp. PI current

//**************************
// MOTOR
/*PH - IRÁNY (elore-hatra)
  EN - SEBESSÉG (8 bit)
  M1 - fékezés mód, nem kell, be van állítva
  M2 - fékezés mód, nem kell, be van állítva
  SL - alvás energia spórolásra, egyelőre nem kell
  FT - fékezés mód, nem kell, be van állítva
*/
// Left motor control pins
#define ML_PH 19 // Left motor PH
#define ML_EN 14 // Left motor EN
#define ML_M1 22 // Left motor M1
#define ML_M2 20 // Left motor M2
#define ML_SL 21 // Left motor SL
#define ML_FT 23 // Left motor FT

// Right motor control pins
#define MR_PH 12 // Right motor PH
#define MR_EN 13 // Right motor EN
#define MR_M1 66 // Right motor M1
#define MR_M2 64 // Right motor M2
#define MR_SL 65 // Right motor SL
#define MR_FT 67 // Right motor FT



/*******************************
* CLASSES
********************************/

class Led
{
private:
    byte pin;

public:
    Led(byte pin)
    {
      this->pin = pin;
      init();
    }

    void init()
    {
      pinMode(pin, OUTPUT);
      off();
    }

    void on()
    {
      digitalWrite(pin, HIGH);
    }

    void off()
    {
      digitalWrite(pin, LOW);
    }
};

class RotEnc_L        // az ISR static void miatt két "pin bedrótozott" osztályt használok
{
 private:
    const static byte Apin = EL_A;
    const static byte Bpin = EL_B;
    volatile static long old_cntr;
    volatile static long cntr;  
    
    static void ISRread()
    {
      digitalRead(Bpin) ? cntr++ : cntr--;
    }       
    
public:
    RotEnc_L()
    {
      init();
    }

    void init()
    {
      cntr = -999;
      old_cntr = -999;      
      pinMode(Apin,INPUT_PULLUP);
      pinMode(Bpin,INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(Apin), ISRread, RISING);
    }

    long getCnt()
    {  
      return cntr;  
    }  
};

class RotEnc_R        // az ISR static void miatt két "pin bedrótozott" osztályt használok
{
private:
    const static byte Apin = ER_A;
    const static byte Bpin = ER_B;
    volatile static long old_cntr;
    volatile static long cntr;  

    static void ISRread()
    {
      digitalRead(Bpin) ? cntr++ : cntr--;
    }

public:
    RotEnc_R()
    {
      init();
    }

    void init()
    {
      cntr = -999;
      old_cntr = -999;      
      pinMode(this->Apin,INPUT_PULLUP);
      pinMode(this->Bpin,INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(Apin), ISRread, RISING);
    }

    long getCnt()
    {  
      return cntr;  
    }  
};

/*******************************
* FUNCTIONS
********************************/
void pause(int ms);  // a delay vs vTaskDelay feloldasa
void powerOn();       // Turn on self power hold for Arduino
void powerOff();      // Turn off Arduino

#endif // __VISIBOT_LIB_H