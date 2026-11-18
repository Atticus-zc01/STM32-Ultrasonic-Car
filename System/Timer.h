#ifndef __TIMER_H
#define __TIMER_H

void Timer_Init(void);

uint32_t Get_Tick(void);
extern volatile uint32_t system_ticks;      /* 单位 ms，每 10 ms 递增 */

#endif
