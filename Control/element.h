#ifndef _ELEMENT_H__
#define _ELEMENT_H__

void Element_Process(void);
void Element_Normal(void);
void Element_Busy(void);
void Element_Noline(void);
void Element_Stop(void);

/* 以下为原作者的赛道元素逻辑，保留但默认不调用
   （本赛赛道没有圆环/十字/掉头/跷跷板） */
void Element_Ring(void);
void Element_Ten(void);
void Element_MoreRing(void);
void Element_Turnback(void);

extern uint32_t Ctrl_Tick;                  /* 10 ms 递增的软件节拍（ms） */

extern uint8_t Element_Flag;
extern uint8_t Noline_Flag;
extern uint8_t Stop_Flag;
extern uint8_t Busy_Flag;
extern uint8_t Busy_Flag2;
extern uint8_t Ring_Flag;
extern uint8_t Ten_Flag;
extern uint8_t MoreRing_Flag;
extern uint8_t Turnback_Flag;

extern int Ring_Turn, Ten_Temp, Turnback_time;

#endif
