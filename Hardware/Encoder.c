#include "stm32f10x.h"
#include "config.h"

/* =====================================================================
 *  整改说明（2026-11）
 *  1) 修正速度滤波：原实现先 Speed_L = 原始值，再用 Speed_L*0.7 +
 *     Speed_L_Temp*0.3 计算，而 Speed_L_Temp 已被写成同一个原始值，
 *     结果 0.7x+0.3x 恒等于 x，滤波完全没生效。此处改为
 *     "用上一拍滤波值 + 本拍原始值"，系数取 0.2（接近原始的快速响应）
 *  2) 编码器极性集中到 config.h（ENCODER_x_INVERT），换电机只改一处
 *  3) 里程增加 LOC_CAL 标定系数，默认 1.0 不改变原有行为
 * ===================================================================== */

int   Speed_L, Speed_R;
int   Speed_L_Temp, Speed_R_Temp;
float Location = 0.0f;

float Speed_Smooth = 0.2f;                                  /* 0~1，越大越平滑 */
float Real_Distance = CM_PER_COUNT_LR;   /* 名义值，实际以 LOC_CAL 标定为准 */

void Encoder_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;        /* ARR */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;         /* PSC */
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);

    TIM_Cmd(TIM3, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;        /* ARR */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;         /* PSC */
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);

    TIM_Cmd(TIM4, ENABLE);
}

int16_t Encoder_Get_L(void)
{
    int16_t Temp1;
    Temp1 = (int16_t)TIM_GetCounter(TIM4);
#if ENCODER_L_INVERT
    Temp1 = (int16_t)(-Temp1);
#endif
    TIM_SetCounter(TIM4, 0);
    return Temp1;
}

int16_t Encoder_Get_R(void)
{
    int16_t Temp2;
    Temp2 = (int16_t)TIM_GetCounter(TIM3);
#if ENCODER_R_INVERT
    Temp2 = (int16_t)(-Temp2);
#endif
    TIM_SetCounter(TIM3, 0);
    return Temp2;
}

void Encoder_Read(void)
{
    int rawL, rawR;

    rawL = Encoder_Get_L();
    rawR = Encoder_Get_R();

    Speed_L = (int)(Speed_L * Speed_Smooth + rawL * (1.0f - Speed_Smooth));
    Speed_R = (int)(Speed_R * Speed_Smooth + rawR * (1.0f - Speed_Smooth));

    Speed_L_Temp = rawL;
    Speed_R_Temp = rawR;

    Location += (float)(rawL + rawR) * Real_Distance * LOC_CAL;
}

void Clear_Location(void)
{
    Location = 0.0f;
}
