#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f10x.h"

/* =====================================================================
 *  探头引脚
 *  1 号：Trig PA3 / Echo PA2
 *  2 号：Trig PB10 / Echo PB11 —— 借用原 HC-06 蓝牙座（USART3），
 *       因此 main.c 里的 Serial_Init() 必须保持注释，且 HC-06 要拔掉，
 *       否则模块的 TXD 会一直驱动 PB11，与本探头冲突。
 *  电平：PB10、PB11 是 5 V 容忍脚；PA2 不是（TTa），
 *       建议给 1 号模块供 3.3 V 或串电阻分压。
 * ===================================================================== */
#define ULTRASONIC1_TRIG_PORT  GPIOA
#define ULTRASONIC1_TRIG_PIN   GPIO_Pin_3
#define ULTRASONIC1_ECHO_PORT  GPIOA
#define ULTRASONIC1_ECHO_PIN   GPIO_Pin_2

#define ULTRASONIC2_TRIG_PORT  GPIOB
#define ULTRASONIC2_TRIG_PIN   GPIO_Pin_10
#define ULTRASONIC2_ECHO_PORT  GPIOB
#define ULTRASONIC2_ECHO_PIN   GPIO_Pin_11

void Ultrasonic_Init(void);

/* 主循环反复调用：非阻塞状态机，内部自己按周期调度两路交替 */
void Ultrasonic_Task(void);

/* 两路状态机都空闲（可以安全刷屏） */
uint8_t Ultra_Idle(void);

/* 正在量回波高电平（此时不要做耗时的 I2C） */
uint8_t Ultra_Timing(void);

/* 阻塞式单次测量：只给分模块测试用，正常运行不要调用 */
float Ultrasonic1_MeasureDistance(void);
float Ultrasonic2_MeasureDistance(void);

/* 1 = 计时基准异常或回波异常（此时禁止避障） */
extern volatile uint8_t  g_UltraFault;
extern volatile float    g_Ultrasonic1Distance;
extern volatile float    g_Ultrasonic2Distance;
extern volatile uint16_t g_UltraQuiet1;     /* 1 号探头连续"无回波"次数 */
extern volatile uint16_t g_UltraQuiet2;     /* 2 号探头连续"无回波"次数 */

float LowPassFilter(float new_value, float *last_value, float alpha);

#endif
