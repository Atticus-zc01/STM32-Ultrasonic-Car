#include "myfile.h"

/* 页面函数只往显存里画，不调用 OLED_Update；
   由 main.c 以 5 Hz 统一调用 Menu_Refresh() + OLED_Update() */

uint8_t Key_Num = 0;
uint8_t func_index = 0;

typedef struct
{
    uint8_t Current;
    uint8_t Up;
    uint8_t Down;
    void (*current_operation)(void);
} Key_table;

static Key_table table[5] =
{
    {0, 4, 1, Boot_animation},
    {1, 0, 2, Homepage_1},
    {2, 1, 3, Homepage_2},
    {3, 2, 4, Homepage_3},
    {4, 3, 0, Homepage_4},
};

void menu_operation(void)
{
    if (Key_Num == 1)
    {
        func_index = table[func_index].Up;
        OLED_Clear();
    }
    if (Key_Num == 2)
    {
        func_index = table[func_index].Down;
        OLED_Clear();
    }
    Key_Num = 0;
}

/* 只重画当前页，不刷屏 */
void Menu_Refresh(void)
{
    table[func_index].current_operation();
}

/************************显示部分***************************/
void Boot_animation(void)
{
    OLED_Printf(0, 8,  OLED_8X16, "SILK ROAD CAR");
    OLED_Printf(0, 32, OLED_8X16, "READY");
}

/* 第 1 页：速度环 */
void Homepage_1(void)
{
    OLED_Printf(0, 1,  OLED_8X16, " Speed_L:%+05d", Speed_L);
    OLED_Printf(0, 17, OLED_8X16, " Speed_R:%+05d", Speed_R);
    OLED_Printf(0, 34, OLED_8X16, " PWM_L:%+05d", Speed_Out_L);
    OLED_Printf(0, 51, OLED_8X16, " PWM_R:%+05d", Speed_Out_R);
}

/* 第 2 页：转向环与元素状态 */
void Homepage_2(void)
{
    OLED_Printf(0, 0,  OLED_8X16, " A_out:%+04d", Angle_out);
    OLED_Printf(0, 17, OLED_8X16, " f_err:%+04.1f", final_err);
    OLED_Printf(0, 34, OLED_8X16, " BFlag:%+02d", Busy_Flag);
    OLED_Printf(0, 51, OLED_8X16, " BFlag2:%+02d", Busy_Flag2);
}

/* 第 3 页：里程、姿态、两路距离 */
void Homepage_3(void)
{
    OLED_Printf(0, 0,  OLED_8X16, " s:%+06.2f", Location);
    OLED_Printf(0, 17, OLED_8X16, " yaw:%+04.1f", yaw);
    OLED_Printf(0, 34, OLED_8X16, " sl:%+05.1f", (float)g_Ultrasonic1Distance);
    OLED_Printf(0, 51, OLED_8X16, " sr:%+05.1f", (float)g_Ultrasonic2Distance);
}

/* 第 4 页：距离差、连续无回波次数、两个故障位、停车状态
   Q 每路连续"无回波"次数：一路一直涨、另一路正常 = 那只探头可能掉线
   F 两个故障位：超声波 / 陀螺                                            */
void Homepage_4(void)
{
    OLED_Printf(0, 0,  OLED_8X16, " d:%+05.0f", (float)g_Ultrasonic1Distance - (float)g_Ultrasonic2Distance);
    OLED_Printf(0, 17, OLED_8X16, " Q:%02d,%02d", (int)g_UltraQuiet1, (int)g_UltraQuiet2);
    OLED_Printf(0, 34, OLED_8X16, " F:%d,%d", (int)g_UltraFault, (int)g_MPUFault);
    OLED_Printf(0, 51, OLED_8X16, " FS:%+02d", Stop_Flag);
}
