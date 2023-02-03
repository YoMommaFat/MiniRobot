/*
file: VisiBot_lib.cpp
date: 2022.08.12.
auth: INIT - Szakáll Tibor
vers: 1.3  2023.01.28.
prev: Sinkovits Béla, "MiniRobot_1.5.ino"
desc: + A VisiBot robot hardwer definíciója
      + A motor enkóderekhez osztály 
stat: OK
*/

// a hardver definíció
#include "VisiBot_lib.h"

void pause(int ms)         // a delay vs vTaskDelay feloldasa
{
#ifdef RTOS_DELAY          // FIGYELEM! ez 16MHZ-s processzorra igaz
    // TickType_t ticks = ms / portTICK_PERIOD_MS;   // AVR-en ez a konstans 16
    TickType_t ticks = ms >> 4;                      // shift, avagy 16-al osztok, gyorsabb
    vTaskDelay(ticks ? ticks : 1);                   // Minimum delay = 1 tick
#else
    delay(ms);
#endif
}

// Turn on self power hold for Arduino
void powerOn()
{
    digitalWrite(PAR_HLD, HIGH);
}

// Turn off Arduino
void powerOff()
{
    digitalWrite(PAR_HLD, LOW);
}


//  az osztály statikus változóinak létrehozása, sajnos ez kell
volatile long RotEnc_L::old_cntr = -999;
volatile long RotEnc_L::cntr = -999; 
volatile long RotEnc_R::old_cntr = -999;
volatile long RotEnc_R::cntr = -999; 

/*

   
    // Read the rotary encoder position in an atomic block to avoid a potential misread
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        for(int k = 0; k < NMOTORS; k++)
        {
            pos[k] = posIR[k];
        }
    }
        
*/
