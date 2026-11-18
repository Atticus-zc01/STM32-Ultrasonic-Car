#include "myfile.h"
#include "sensor.h"

/* =====================================================================
 *  整改说明（2026-11）
 *  1) 先判"有没有探头压线"再做除法（原实现 0/0 会算出 NaN）
 *  2) 取消边缘 x27 增益（它与 times=10、限幅 ±30 叠加后必然饱和，
 *     边缘增益约为中心的 18 倍，表现为贴线左右猛甩）
 *  3) 丢线时不再冻结误差（原实现会保持最后转角原地画圈），
 *     改为按指数衰减，让车先直线冲过缝隙，交给 element.c 的搜索流程
 *  4) 丢线状态用 Line_Active 显式传递，不再靠 "err==0" 猜
 * ===================================================================== */

#define SENSOR_NUM 5

typedef struct
{
    float Angle;
    float channel;
} SensorAngle;

typedef struct
{
    float Angle;
    float error;
    float ALPHA;
    float times;
} FusedAngle;

static FusedAngle fusedSensor = {0.0f, 0.0f, FUSED_ALPHA, FUSED_TIMES};

static const SensorAngle sensorAngle[SENSOR_NUM] =
{
    {-2.5f, 1},   /* L2 */
    {-1.0f, 2},   /* L1 */
    { 0.0f, 3},   /* M  */
    { 1.0f, 4},   /* R1 */
    { 2.5f, 5}    /* R2 */
};

uint8_t Line_Active = 1;

float Error_Calcaulate(void)
{
    float sum = 0.0f;
    int   cnt = 0;
    int   i;

    for (i = 0; i < SENSOR_NUM; i++)
    {
        if (digital((unsigned char)sensorAngle[i].channel))
        {
            sum += sensorAngle[i].Angle;
            cnt++;
        }
    }

    if (cnt == 0)
    {
        Line_Active = 0;
        return 0.0f;                 /* 先判空再做除法 */
    }

    Line_Active = 1;
    return (sum / (float)cnt) * Right_err();
}

/* 保留接口；边缘放大已取消，增益统一由 FUSED_TIMES 决定 */
float Right_err(void)
{
    return 1.0f;
}

float get_fused_error(float sensor_err, float gyro_z)
{
    static float last_gyro_z = 0.0f;

    fusedSensor.Angle += (last_gyro_z + gyro_z) * 0.5f * CTRL_DT_S;
    fusedSensor.Angle = Min_Max(fusedSensor.Angle, -10.0f, 10.0f);

    if (Line_Active)
    {
        fusedSensor.Angle = fusedSensor.Angle * (1.0f - fusedSensor.ALPHA)
                          + sensor_err * fusedSensor.ALPHA;

        fusedSensor.error = fusedSensor.Angle * (1.0f - fusedSensor.ALPHA)
                          + sensor_err * fusedSensor.ALPHA * fusedSensor.times;
    }
    else
    {
        /* 丢线：衰减到零 = 指令回正，先直线穿过缝隙 */
        fusedSensor.error *= LINE_LOST_DECAY;
        fusedSensor.Angle  = fusedSensor.error;
    }

    last_gyro_z = gyro_z;

    fusedSensor.error = Min_Max(fusedSensor.error, -FUSED_LIMIT, FUSED_LIMIT);
    fusedSensor.error = low_pass_filter(fusedSensor.error, FUSED_LP_ALPHA);

    return fusedSensor.error;
}

float low_pass_filter(float input, float alpha)
{
    static float output = 0.0f;
    output = alpha * input + (1.0f - alpha) * output;
    return output;
}
