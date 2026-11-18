#include "myfile.h"

/* =====================================================================
 *  控制环（2026-11 整改）
 *  1) 左右轮各用一份 PID 状态
 *  2) 速度指令改为斜坡输出，元素切换速度不再阶跃
 *  3) 转向环的"上次误差"按通道分开传入，不与角度环共用
 *  4) 控制函数内不访问 I2C、不做阻塞测距，只做计算
 * ===================================================================== */

uint8_t Place_Enable = 1, PWM_Enable = 1;
uint8_t Angle_Enable = 0;
int Speed_Out_L = 0, Speed_Out_R = 0, Place_Out = 0, Angle_out = 0;
float sensor_err = 0, final_err = 0;

int Basic_Speed = TRACK_SPEED;      /* 目标速度：元素/状态机写这个 */
static int Basic_Speed_Cmd = 0;     /* 斜坡后的实际执行速度 */

int Left_Speed = 0, Right_Speed = 0;

int Speed_PID[3] = {150, 0, 80};
float Place_PD[4] = {5.0f, 0.0f, 4.3f, 0.0f};
float Angle_PD[4] = {4.0f, 0.0f, 2.0f, 0.0f};

/* 转向环与角度环各自的上次误差 */
static float place_last_err = 0.0f;
static float angle_last_err = 0.0f;

typedef struct
{
    int Integral;
    int LastError;
} PID_State;

static PID_State pid_left  = {0, 0};
static PID_State pid_right = {0, 0};

static int PID_Step(PID_State *st, int NowPoint, int SetPoint, const int *Gain)
{
    int NowError, Out;

    NowError = SetPoint - NowPoint;

    st->Integral += NowError;
    st->Integral = Min_Max(st->Integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    Out = Gain[0] * NowError + Gain[1] * st->Integral + Gain[2] * (NowError - st->LastError);
    st->LastError = NowError;

    return Out;
}

int PID_Control_L(int NowPoint, int SetPoint)
{
    return PID_Step(&pid_left, NowPoint, SetPoint, Speed_PID);
}

int PID_Control_R(int NowPoint, int SetPoint)
{
    return PID_Step(&pid_right, NowPoint, SetPoint, Speed_PID);
}

void PID_Reset(void)
{
    pid_left.Integral = 0;
    pid_left.LastError = 0;
    pid_right.Integral = 0;
    pid_right.LastError = 0;
}

/* 转向环/角度环的上次误差清零：进入绕行、找回线时调用，避免 D 项拿旧值冲击 */
void PID_ResetPlace(void)
{
    place_last_err = 0.0f;
    angle_last_err = 0.0f;
}

/* 速度斜坡：把 Basic_Speed 的阶跃变成斜坡，减少打滑与电流冲击 */
void Speed_Ramp(void)
{
    if (Basic_Speed_Cmd < Basic_Speed)
    {
        Basic_Speed_Cmd += SPEED_RAMP_STEP;
        if (Basic_Speed_Cmd > Basic_Speed) Basic_Speed_Cmd = Basic_Speed;
    }
    else if (Basic_Speed_Cmd > Basic_Speed)
    {
        Basic_Speed_Cmd -= SPEED_RAMP_STEP;
        if (Basic_Speed_Cmd < Basic_Speed) Basic_Speed_Cmd = Basic_Speed;
    }
}

void Control(void)
{
    update_yaw();                       /* 只用缓存角速度积分，不访问 I2C */

    sensor_err = Error_Calcaulate();
    final_err  = get_fused_error(sensor_err, gyro_z);

    if (Place_Enable)
    {
        Place_Out = (int)Place_Control(final_err, 0, Place_PD, &place_last_err);
    }

    if (Angle_Enable)
    {
        Angle_out = (int)Place_Control(yaw, 0, Angle_PD, &angle_last_err);
        Angle_out = Min_Max(Angle_out, -50, 50);
    }

    Element_Process();

    Encoder_Read();
    Speed_Ramp();
    Different_Speed();

    Speed_Out_L = PID_Control_L(Speed_L, Left_Speed);
    Speed_Out_R = PID_Control_R(Speed_R, Right_Speed);

    Speed_Out_L = Min_Max(Speed_Out_L, -Max_PWM, Max_PWM);
    Speed_Out_R = Min_Max(Speed_Out_R, -Max_PWM, Max_PWM);

    if (PWM_Enable)
    {
        Motor_SetPWM_L((int16_t)Speed_Out_L);
        Motor_SetPWM_R((int16_t)Speed_Out_R);
    }
    else
    {
        PID_Reset();                    /* 直驱期间清积分，避免恢复时冲击 */
    }
}

float Place_Control(float NowPoint, float SetPoint, float *TURN_PID, float *LastError)
{
    float NowError, Out;

    NowError = SetPoint - NowPoint;

    Out = TURN_PID[0] * NowError
        + TURN_PID[1] * (NowError * fabsf(NowError))
        + TURN_PID[2] * (NowError - *LastError)
        + TURN_PID[3] * gyro_z;

    if (fabsf(Out) <= 5.0f) Out = 0;

    *LastError = NowError;
    return Out;
}

void Different_Speed(void)
{
    float k;
    const float Turn_factor = 0.5f;

    k = (float)(Place_Out + Angle_out) * 0.01f;

    if (k > 0.95f)  k = 0.95f;
    if (k < -0.95f) k = -0.95f;

    if (k >= 0.0f)
    {
        Left_Speed  = (int)(Basic_Speed_Cmd * (1.0f - k));
        Right_Speed = (int)(Basic_Speed_Cmd * (1.0f + k * Turn_factor));
    }
    else
    {
        k = -k;
        Left_Speed  = (int)(Basic_Speed_Cmd * (1.0f + k * Turn_factor));
        Right_Speed = (int)(Basic_Speed_Cmd * (1.0f - k));
    }
}
