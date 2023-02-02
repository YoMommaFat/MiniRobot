PRÓBA
/*
file: motor_3.cpp
date: 2022.07.31.
auth: INIT - Szakáll Tibor
vers: 1.3
prev: Sinkovits Béla, "MiniRobot_1.5.ino"
desc: + A két motor alap fuggvényei
      + Ez esetben: Phase digitális 0/1 out és az ENABLE PWM
      lásd 9.2.3.1 Pulse-Width Modulating ENABLE -> "DRV880x DMOS Full-Bridge Motor Drivers.pdf"
      UJ iranyvaltas: -100..-1, 0 és +1..+100, ahol 0= STOP
      + +/- 100 működik 
      + a futyules kiiktatás működik
stat: OK
*/

#include "motor_3.h"

void motorInit()    // a motorokhoz tartozó pinek beállítása
{
  pinMode(ML_PH, OUTPUT);
  pinMode(ML_EN, OUTPUT);
  pinMode(ML_M1, OUTPUT);
  pinMode(ML_M2, OUTPUT);
  pinMode(ML_SL, OUTPUT);
  pinMode(ML_FT, INPUT);
     
  pinMode(MR_PH, OUTPUT);
  pinMode(MR_EN, OUTPUT);
  pinMode(MR_M1, OUTPUT);
  pinMode(MR_M2, OUTPUT);
  pinMode(MR_SL, OUTPUT);
  pinMode(MR_FT, INPUT);
  
  motorSleep(); // a motorok letiltva, stop pwm értéken
}

void setMotor(int motor_no, int phase_pwm)  // MOTOR_L/MOTOR_R, -100..-1/0/+1..+100
{
  phase_pwm = (motor_no == MOTOR_R) ? -phase_pwm : phase_pwm;  // a jobb motor tengelye ellentétes, ezért a másik irány az előre
  phase_pwm *= PWM_100;                                        // hogy a táphoz lehessen igazítani a max V-t
  if(abs(phase_pwm) < PWM_OFF_MARGIN)                          // ez kidobható, csak a STOP körüli "fütyülést" szünteti meg
  {
     motorDisable(motor_no);
  }
  else
  {
     motorEnable(motor_no);
  }
  analogWrite(pwm[motor_no], phase_pwm + PWM_MIDDLE);
}  

void motorEnable(int motor_no)  // engedélyezzük a motort
{
  digitalWrite(enable[motor_no], HIGH);
}

void motorDisable(int motor_no)  // letiltjuk a motort
{
  digitalWrite(enable[motor_no], LOW);
}

void motorSleep()  // Turn off the motors power AND disable AND stop pwm!
{
    digitalWrite(ML_SL, LOW);
    digitalWrite(MR_SL, LOW);
    motorDisable(MOTOR_L);              // letiltjuk
    motorDisable(MOTOR_R);              // letiltjuk 
    setMotor(MOTOR_L, MOTOR_STOP_PWM);  // le is allítjuk, ha legközelebb indulna
    setMotor(MOTOR_R, MOTOR_STOP_PWM);  // le is allítjuk, ha legközelebb indulna
}

void motorWake()  // Turn off the motor power, NO enabling, NO pwmchange!
{
    digitalWrite(ML_SL, HIGH);
    digitalWrite(MR_SL, HIGH);
    pause(15); // Allow 5ms delay before applying PWM signals (DRV8801), need a 1ms for charge pump
}

// Set the decay mode of the motors
void fastDecay()
{
    digitalWrite(ML_M1, LOW);
    digitalWrite(ML_M2, LOW);
    digitalWrite(MR_M1, LOW);
    digitalWrite(MR_M2, LOW);
}

// Set the decay mode of the motors
void slowDecayHigh()
{
    digitalWrite(ML_M1, HIGH);
    digitalWrite(ML_M2, HIGH);
    digitalWrite(MR_M1, HIGH);
    digitalWrite(MR_M2, HIGH);
}

// Set the decay mode of the motors
void slowDecayLow()
{
    digitalWrite(ML_M1, HIGH);
    digitalWrite(ML_M2, LOW);
    digitalWrite(MR_M1, HIGH);
    digitalWrite(MR_M2, LOW);
}
