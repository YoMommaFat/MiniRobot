#include "VisiBot_lib.h"

void pause(int ms) { // a delay vs vTaskDelay feloldasa

#ifdef RTOS_DELAY // FIGYELEM! ez 16MHZ-s processzorra igaz
    // TickType_t ticks = ms / portTICK_PERIOD_MS;   // AVR-en ez a konstans 16
    TickType_t ticks = ms >> 4;                      // shift, avagy 16-al osztok, gyorsabb
    vTaskDelay(ticks ? ticks : 1);                   // Minimum delay = 1 tick
#else
    delay(ms);
#endif
}

void powerOn() { // Turn on self power hold for Arduino
    digitalWrite(PAR_HLD, HIGH);
}

void powerOff() { // Turn off Arduino
    digitalWrite(PAR_HLD, LOW);
}

//  az osztály statikus változóinak létrehozása, sajnos ez kell
volatile long RotEnc_L::old_cntr = 0;
volatile long RotEnc_L::cntr = 0;
volatile long RotEnc_R::old_cntr = 0;
volatile long RotEnc_R::cntr = 0;

/*
// Read the rotary encoder position in an atomic block to avoid a potential misread
ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  for(int k = 0; k < NMOTORS; k++) {
    pos[k] = posIR[k];
  }
}
*/
