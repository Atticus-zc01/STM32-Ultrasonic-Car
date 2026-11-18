#ifndef __MPU6050_H
#define __MPU6050_H

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Buffer, uint8_t Length);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *GyroZ);

/* 主循环调用：I2C 读一次角速度，刷新 gyro_z（不在中断里做 I2C） */
void MPU6050_PollGyro(void);
/* 中断调用：只用缓存值积分，不访问 I2C */
void update_yaw(void);

void calibrate_gyro(void);
void Clear_yaw(void);

extern float pitch, roll, yaw;
extern int16_t GZ;
extern volatile float gyro_z;

/* 1 = 陀螺不可用（WHO_AM_I 不对 或 I2C 连续失败） */
extern volatile uint8_t g_MPUFault;

#endif
