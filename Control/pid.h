#ifndef __PID_H
#define __PID_H

#include "config.h"

#define INTEGRAL_MAX  1000
#define Min_Max(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define Max_PWM       6800

void Control(void);
void Different_Speed(void);
void Speed_Ramp(void);
void PID_Reset(void);
void PID_ResetPlace(void);

int   PID_Control_L(int NowPoint, int SetPoint);
int   PID_Control_R(int NowPoint, int SetPoint);
float Place_Control(float NowPoint, float SetPoint, float *TURN_PID, float *LastError);

extern int Speed_Out_L, Speed_Out_R, Place_Out, Angle_out;
extern int Left_Speed, Right_Speed;
extern float sensor_err;
extern float final_err;
extern int Basic_Speed;
extern uint8_t Place_Enable, PWM_Enable, Angle_Enable;

#endif
