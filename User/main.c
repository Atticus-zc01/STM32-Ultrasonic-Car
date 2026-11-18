#include "myfile.h"

/* =====================================================================
 *  主循环调度
 *  1) 陀螺 100 Hz（每个控制节拍读一次），中断只读缓存
 *  2) 超声波由 Ultrasonic_Task() 内部状态机推进，非阻塞、两路交替
 *  3) OLED 5 Hz 刷新，按键时立即重绘
 *  4) 定时器最后启动，避免初始化期间就进中断
 * ===================================================================== */

#define OLED_REFRESH_MS     200u        /* 刷屏周期，5 Hz */

#if USE_IWDG
static void IWDG_Setup(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(IWDG_RELOAD_VAL);
    IWDG_ReloadCounter();
    IWDG_Enable();
}
#endif

/* 上电倒计时：规则只禁止运行中的人工干预，起跑前等待不违规 */
static void Start_Countdown(void)
{
    int8_t s;

    for (s = START_DELAY_S; s > 0; s--)
    {
        OLED_Clear();
        OLED_Printf(0, 8,  OLED_8X16, "PLACE CAR");
        OLED_Printf(0, 32, OLED_8X16, "START IN %d", (int)s);
        OLED_Update();
        Delay_ms(1000);
    }

    OLED_Clear();
    OLED_Printf(0, 8,  OLED_8X16, "CALIBRATING");
    OLED_Printf(0, 32, OLED_8X16, "KEEP STILL");
    OLED_Update();
}

/************************主函数***************************/
int main(void)
{
    uint32_t now, t_oled, t_fed, t_gyro;
    uint8_t  key;

    MPU6050_Init();             /* 内含软件 I2C 初始化与 JTAG 释放，并校验 WHO_AM_I */
    /* Serial_Init();  —— 蓝牙座 PB10/PB11 已改作 2 号超声波，串口不可用；
       若以后要用串口调试，必须先把 2 号探头挪到别的空闲引脚 */
    //Serial_Init();
    Key_Init();
    OLED_Init();
    Encoder_Init();
    SENSOR_GPIO_Config();       /* 循迹引脚初始化 */
    Motor_Init();               /* 电机初始化（内含 PWM_Init） */
    Ultrasonic_Init();

    Start_Countdown();          /* 3 s 倒计时：这段时间用来把车摆到起始线 */
    calibrate_gyro();           /* 静止状态下 200 次平均，约 0.4 s */

    OLED_Clear();
    Menu_Refresh();             /* 第 0 页：开机页（Boot_animation） */
    OLED_Update();

#if USE_IWDG
    IWDG_Setup();
#endif

    t_oled = 0;
    t_fed  = 0;
    t_gyro = 0xFFFFFFFFu;       /* 强制第一拍就读一次 */

    Timer_Init();               /* 最后启动 10 ms 节拍 */

    while (1)
    {
        now = Get_Tick();

        /* ---- 按键：只在有键时重绘 ---- */
        key = Key_GetNum();
        if (key)
        {
            Key_Num = key;
            menu_operation();
            if (Ultra_Idle())               /* 整屏刷新会打断回波计时，宁可晚 30 ms */
            {
                Menu_Refresh();
                OLED_Update();
            }
        }

        /* ---- 陀螺：每个控制节拍读一次（100 Hz） ---- */
        if (now != t_gyro && !Ultra_Timing())
        {
            t_gyro = now;
            MPU6050_PollGyro();             /* 约 0.5 ms：避回波高电平期间，否则脉宽多算 1 ms */
        }

        /* ---- 超声：非阻塞状态机，每次循环推进一点 ---- */
        Ultrasonic_Task();

        /* ---- 刷屏：5 Hz，且必须等测距状态机空闲 ----
           整屏软 I2C 要 10~15 ms，若在测量途中刷屏会漏掉回波或把脉宽算长 */
        if (Ultra_Idle() && (uint32_t)(now - t_oled) >= OLED_REFRESH_MS)
        {
            t_oled = now;
            Menu_Refresh();
            OLED_Update();
        }

#if USE_IWDG
        if (now != t_fed)
        {
            t_fed = now;
            IWDG_ReloadCounter();
        }
#endif
    }
}
