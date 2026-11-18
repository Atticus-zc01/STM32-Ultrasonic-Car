#include "stm32f10x.h"
#include "PWM.h"
#include "config.h"

/* =====================================================================
 *  整改说明（2026-11）
 *  1) 增加 Motor_Stop()（自由停车）与 Motor_Brake()（短接制动），
 *     供终点停车与超时保护使用
 *  2) 电机极性集中到 config.h（MOTOR_x_INVERT），换电机只改一处
 * ===================================================================== */

void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    PWM_Init();
}

void Motor_SetPWM_L(int16_t Duty)
{
#if MOTOR_L_INVERT
    Duty = (int16_t)(-Duty);
#endif

    if (Duty >= 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompare4((uint16_t)Duty);
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        PWM_SetCompare4((uint16_t)(-Duty));
    }
}

void Motor_SetPWM_R(int16_t Duty)
{
#if MOTOR_R_INVERT
    Duty = (int16_t)(-Duty);
#endif

    if (Duty >= 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        PWM_SetCompare1((uint16_t)Duty);
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompare1((uint16_t)(-Duty));
    }
}

/* 自由停车：方向脚全低 + 占空比 0，电机两端悬空 */
void Motor_Stop(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    PWM_SetCompare4(0);
    PWM_SetCompare1(0);
}

/* 短接制动：IN1=IN2=1 且 PWM 到指定占空比，电机绕组短接，快速停车 */
void Motor_Brake(int16_t Pwm)
{
    if (Pwm < 0) Pwm = (int16_t)(-Pwm);

    GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
    GPIO_SetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
    PWM_SetCompare4((uint16_t)Pwm);
    PWM_SetCompare1((uint16_t)Pwm);
}
