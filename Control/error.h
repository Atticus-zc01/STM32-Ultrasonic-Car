#ifndef __ERROR_H
#define	__ERROR_H

#include "sensor.h"

float Error_Calcaulate(void);
float get_fused_error(float sensor_err, float gyro_z);
float Right_err(void);
float low_pass_filter(float input, float alpha);

/* 1 = 至少一路探头压线；0 = 五路全灭（丢线） */
extern uint8_t Line_Active;

#endif
