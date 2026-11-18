#include "myfile.h"                  // Device header

/* =====================================================================
 *  整改说明（2026-11）
 *  1) 中断里只做控制计算：超声波阻塞测距移出中断（改由 main.c 调度），
 *     陀螺 I2C 也移到主循环，10 ms 周期不再被拉长到 11~15 ms
 *  2) 增加 system_ticks（单位 ms），供主循环做非阻塞调度
 * ===================================================================== */

volatile uint32_t system_ticks = 0;

void Timer_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

uint32_t Get_Tick(void)
{
    return system_ticks;
}

/************************中断***************************/
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        system_ticks += CTRL_DT_MS;

        Control();      /* 只做控制计算：读缓存、算 PID，不做任何阻塞 I/O */
        Key_Tick();

        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
