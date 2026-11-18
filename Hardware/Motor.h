#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void Motor_SetPWM_L(int16_t Duty);
void Motor_SetPWM_R(int16_t Duty);
void Motor_Stop(void);
void Motor_Brake(int16_t Pwm);

#endif
