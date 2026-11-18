#include "myfile.h"
#include <math.h>

/* =====================================================================
 *  陀螺链路自检
 *  - 每次 I2C 读都检查从机应答（原实现全部丢弃 ACK）
 *  - 初始化时校验 WHO_AM_I，不对就置故障位
 *  - 连续失败达 MPU_FAIL_CNT 次置故障位；故障期间 gyro_z 强制为 0，
 *    避免把悬空读到的乱码当成角速度积分进 yaw
 * ===================================================================== */

float pitch, roll, yaw;
int16_t AX, AY, AZ, GX, GY, GZ;

float gyro_zero_z = 0.0f;
float yaw_angle = 0.0f;
float last_gyro_z = 0.0f;
volatile float gyro_z = 0.0f;

volatile uint8_t g_MPUFault = 0;

static int16_t  gz_raw = 0;
static uint8_t  mpu_fail = 0;

#define MPU6050_ADDRESS     0xD0
#define MPU6050_ID_VALUE    0x68
#define GYRO_LSB_PER_DPS    16.4f       /* ±2000 dps 量程 */

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddress);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return Data;
}

/* 带应答检查的连续读：返回 1 成功，0 从机无应答 */
static uint8_t MPU6050_ReadRegsAck(uint8_t RegAddress, uint8_t *Buffer, uint8_t Length)
{
    uint8_t i;

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS);
    if (MyI2C_ReceiveAck() != 0) { MyI2C_Stop(); return 0; }   /* 0 = ACK */

    MyI2C_SendByte(RegAddress);
    if (MyI2C_ReceiveAck() != 0) { MyI2C_Stop(); return 0; }

    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
    if (MyI2C_ReceiveAck() != 0) { MyI2C_Stop(); return 0; }

    for (i = 0; i < Length; i++)
    {
        Buffer[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck((i + 1 < Length) ? 0 : 1);
    }

    MyI2C_Stop();
    return 1;
}

void MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Buffer, uint8_t Length)
{
    MPU6050_ReadRegsAck(RegAddress, Buffer, Length);
}

void MPU6050_Init(void)
{
    MyI2C_Init();
    Delay_ms(50);

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);     /* 时钟源取陀螺 X 轴 */
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);     /* 1 kHz / 5 = 200 Hz */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);         /* DLPF 44 Hz */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);    /* ±2000 dps */
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);   /* ±16 g */

    Delay_ms(200);

    /* 校验器件 ID：读不到或不对，说明陀螺不在线 */
    if (MPU6050_ReadReg(MPU6050_WHO_AM_I) != MPU6050_ID_VALUE)
    {
        g_MPUFault = 1;
        gyro_z = 0.0f;
    }
    else
    {
        g_MPUFault = 0;
    }
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *GyroZ)
{
    uint8_t DataH, DataL;

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *GyroZ = (int16_t)(((uint16_t)DataH << 8) | DataL);
}

/* 主循环调用：只做 I2C 与换算，约 0.5 ms */
void MPU6050_PollGyro(void)
{
    uint8_t buf[2];

    if (!MPU6050_ReadRegsAck(MPU6050_GYRO_ZOUT_H, buf, 2))
    {
        if (mpu_fail < 250) mpu_fail++;
        if (mpu_fail > MPU_FAIL_CNT)
        {
            g_MPUFault = 1;
            gyro_z = 0.0f;                  /* 故障期间不积分乱码 */
        }
        return;
    }

    mpu_fail = 0;
    g_MPUFault = 0;

    gz_raw = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    gyro_z = ((float)gz_raw - gyro_zero_z) / GYRO_LSB_PER_DPS;

    if (fabsf(gyro_z) < 0.5f) gyro_z = 0.0f;        /* 死区 */
}

/* 中断调用：只用缓存值做梯形积分，不访问 I2C */
void update_yaw(void)
{
    yaw += (last_gyro_z + gyro_z) * 0.5f * CTRL_DT_S;
    last_gyro_z = gyro_z;
}

void Clear_yaw(void)
{
    yaw = 0.0f;
    last_gyro_z = 0.0f;
}

/* 零漂标定：静止状态下取 200 次平均 */
void calibrate_gyro(void)
{
    float sum = 0.0f;
    int16_t gz;
    uint16_t i;

    if (g_MPUFault) { gyro_zero_z = 0.0f; return; }

    for (i = 0; i < 200; i++)
    {
        MPU6050_GetData(&gz);
        sum += (float)gz;
        Delay_ms(1);
    }

    gyro_zero_z = sum / 200.0f;
}
