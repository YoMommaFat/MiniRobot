/*
file: motor_3.h
date: 2022.08.12.
auth: INIT - Szakáll Tibor
vers: 1.3
prev: Sinkovits Béla, "MiniRobot_1.5.ino"
desc: + A két motor alap fuggvényei
      + Ez esetben: Phase PWM, és az ENABLE digitális 0/1 out 
      lásd 9.2.3.2 Pulse-Width Modulating PHASE -> "DRV880x DMOS Full-Bridge Motor Drivers.pdf"
      REGI irányváltás 0%-49% PWM  vs. 51% - 100% PWM,, 50% = STOP motor
      UJ iranyvaltas: -100..-1, 0 és +1..+100, ahol 0= STOP
stat: OK
*/

#ifndef __MOTOR_H
#define __MOTOR_H

#include <arduino.h>
#include <pins_arduino.h>
// a hardver definíció
#include "VisiBot_lib.h"

// #define NMOTORS 2 // Number of motors
// az AVR 2560 PWM-je elvileg 490Hz vagy 980Hz, de lehet változtatni
// https://www.electronicwings.com/users/sanketmallawat91/projects/215/frequency-changing-of-pwm-pins-of-arduino-uno
// a H-bridge max. 100kHz-t bír el
#define PWM_MIDDLE 128           // a PWM 50%, avagy a phase/pálya fele, ahol nem mozog a motor
#define PWM_100 1.10             // hogy a táphoz lehessen igazítani a max V-t
#define PWM_OFF_MARGIN 38        // [-38,+38] között kikapcsolja a motrot, hogy ne fütyüljön az 50% PWM miatt
#define MOTOR_STOP_PWM 0         // PHASE PWM = 50%

#define MOTOR_L 0 
#define MOTOR_R 1

const int enable[] = {ML_EN, MR_EN}; 
const int pwm[] = {ML_PH, MR_PH}; 

void motorInit();    // a motorokhoz tartozó pinek beállítása
void setMotor(int motor_no, int phase_pwm); // MOTOR_L/MOTOR_R, 0..49/50/51..100

void motorEnable(int motor_no);   // engedélyezzük a motort
void motorDisable(int motor_no);  // letiltjuk a motort

void motorSleep();  // Turn off the motor power
void motorWake();   // Turn on the motor power 

// Set the decay mode of the motors
void fastDecay();
void slowDecayHigh();
void slowDecayLow();

#endif // __MOTOR_H