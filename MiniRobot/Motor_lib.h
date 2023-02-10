#ifndef __MOTOR_LIB_H
#define __MOTOR_LIB_H

#include <arduino.h>
#include <pins_arduino.h>
#include "VisiBot_lib.h"

// #define NMOTORS 2 // Number of motors
// az AVR 2560 PWM-je elvileg 490Hz vagy 980Hz, de lehet változtatni
// https://www.electronicwings.com/users/sanketmallawat91/projects/215/frequency-changing-of-pwm-pins-of-arduino-uno
// a H-bridge max. 100kHz-t bír el
#define PWM_100 1.10 // hogy a táphoz lehessen igazítani a max V-t
#define MOTOR_L 0 
#define MOTOR_R 1

const int enable[] = {ML_EN, MR_EN};
const int phase[] = {ML_PH, MR_PH};

void motorInit(); // a motorokhoz tartozó pinek beállítása
void setMotor(int motor_no, int phase_pwm); // MOTOR_L/MOTOR_R, 0..49/50/51..100

//void motorEnable(int motor_no); // engedélyezzük a motort
//void motorDisable(int motor_no); // letiltjuk a motort

void motorSleep(); // Turn off the motor power
void motorWake(); // Turn on the motor power

// Set the decay mode of the motors
void fastDecay();
void slowDecayHigh();
void slowDecayLow();

#endif // __MOTOR_LIB