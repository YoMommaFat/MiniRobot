#include "Motor_lib.h"

bool assistance = true;
int storedPwmL = 0;
int storedPwmR = 0;

void motorInit() { // a motorokhoz tartozó pinek beállítása
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

void setMotorDest(int motor_no, int phase_pwm) { // MOTOR_L/MOTOR_R, -100..-1/0/+1..+100
  if (motor_no == MOTOR_L) { storedPwmL = phase_pwm; }
  else if (motor_no == MOTOR_R) { storedPwmR = phase_pwm; }
}

void setMotorPwm(int motor_no, int phase_pwm) { // MOTOR_L/MOTOR_R, -100..-1/0/+1..+100
  if (motor_no == MOTOR_L) { storedPwmL = phase_pwm; }
  else if (motor_no == MOTOR_R) { storedPwmR = phase_pwm; }
  phase_pwm = (motor_no == MOTOR_R) ? phase_pwm : -phase_pwm;  // a jobb motor tengelye ellentétes, ezért a másik irány az előre
//  phase_pwm *= PWM_100;                                        // hogy a táphoz lehessen igazítani a max V-t
  if (phase_pwm < 0) { digitalWrite(phase[motor_no], HIGH); }
  else { digitalWrite(phase[motor_no], LOW); }
  analogWrite(enable[motor_no], abs(phase_pwm)*2);
}

int getMotorPwm(int motor_no) { // Return motor PWM
  if (motor_no == MOTOR_L) { return storedPwmL; }
  else if (motor_no == MOTOR_R) { return storedPwmR; }
  else { return 0; }
}

void setMotorAssist(bool ifAssist) { // Set motor assistance
  assistance = ifAssist;
}

bool getMotorAssist() { // Get motor assistance
  return assistance;
}

//void motorEnable(int motor_no) { // engedélyezzük a motort
//  digitalWrite(enable[motor_no], HIGH);
//}

//void motorDisable(int motor_no) { // letiltjuk a motort
//  digitalWrite(enable[motor_no], LOW);
//}

void motorSleep() { // Stop the motors power and sleep
  setMotorPwm(MOTOR_L, 0);
  setMotorPwm(MOTOR_R, 0);
  digitalWrite(ML_SL, LOW);
  digitalWrite(MR_SL, LOW);
}

void motorWake() { // Stop the motors power and wake
  setMotorPwm(MOTOR_L, 0);
  setMotorPwm(MOTOR_R, 0);
  digitalWrite(ML_SL, HIGH);
  digitalWrite(MR_SL, HIGH);
  pause(15); // Allow 5ms delay before applying PWM signals (DRV8801), need a 1ms for charge pump
}

void fastDecay() { // Set the decay mode of the motors
  digitalWrite(ML_M1, LOW);
  digitalWrite(ML_M2, LOW);
  digitalWrite(MR_M1, LOW);
  digitalWrite(MR_M2, LOW);
}

void slowDecayHigh() { // Set the decay mode of the motors
  digitalWrite(ML_M1, HIGH);
  digitalWrite(ML_M2, HIGH);
  digitalWrite(MR_M1, HIGH);
  digitalWrite(MR_M2, HIGH);
}

void slowDecayLow() { // Set the decay mode of the motors
  digitalWrite(ML_M1, HIGH);
  digitalWrite(ML_M2, LOW);
  digitalWrite(MR_M1, HIGH);
  digitalWrite(MR_M2, LOW);
}
