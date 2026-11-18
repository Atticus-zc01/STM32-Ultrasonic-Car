#include "ultrasonic.h"
#include "Delay.h"
#include "myfile.h"

/* =====================================================================
 *  非阻塞测距
 *  - 时间基准直接用 DWT 周期数（72 MHz），所有比较都用 uint32 差值，
 *    让取模交给 2^32 自然溢出：不做除法、不引入第二种模数
 *  - 状态机：触发 → 等上升沿（≤20 ms）→ 量高电平（≤30 ms），全程不阻塞
 *  - 两路交替触发，间隔 ULTRA_PERIOD_MS，避免余振串扰
 *  - 无回波 = 前方无目标（400 cm）；只有回波异常才算失败
 *  - 每路有"连续无回波"计数，用来发现探头掉线
 * ===================================================================== */

/* 老版 CMSIS 头文件里没有 DWT 结构体，直接用寄存器地址 */
#define DWT_CTRL_ADDR       (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT_ADDR     (*(volatile uint32_t *)0xE0001004UL)
#define DEMCR_ADDR          (*(volatile uint32_t *)0xE000EDFCUL)
#define DEMCR_TRCENA        (1UL << 24)
#define DWT_CTRL_CYCCNTENA  (1UL << 0)

volatile float    g_Ultrasonic1Distance = 400.0f;
volatile float    g_Ultrasonic2Distance = 400.0f;
volatile uint8_t  g_UltraFault = 0;
volatile uint16_t g_UltraQuiet1 = 0;
volatile uint16_t g_UltraQuiet2 = 0;

static uint8_t  DWT_OK = 0;
static uint16_t fail1 = 0;
static uint16_t fail2 = 0;

typedef struct
{
    uint8_t  state;         /* 0 空闲 / 1 等上升沿 / 2 量高电平 */
    uint8_t  done;
    uint32_t t0;            /* 触发时刻（周期数） */
    uint32_t t1;            /* 上升沿时刻（周期数） */
    float    result;
    float    last;
    uint8_t  pending;       /* 有一个"跳变过大"的读数在等确认 */
    float    pending_d;
} UltraChan;

static UltraChan ch[2];
static uint32_t  next_slot = 0;     /* 下一次触发的时刻（周期数） */
static uint8_t   slot_ch = 0;

static uint32_t us_cycles(void)
{
    return SystemCoreClock / 1000000u;              /* 72 MHz -> 72 */
}

#define US2CYC(us)  ((uint32_t)(us) * us_cycles())

static uint32_t cyc(void)
{
    return DWT_CYCCNT_ADDR;
}

static void DWT_Setup(void)
{
    uint32_t a, b;

    DEMCR_ADDR |= DEMCR_TRCENA;
    DWT_CYCCNT_ADDR = 0;
    DWT_CTRL_ADDR |= DWT_CTRL_CYCCNTENA;

    a = DWT_CYCCNT_ADDR;
    Delay_us(50);
    b = DWT_CYCCNT_ADDR;

    DWT_OK = (b != a) ? 1 : 0;
}

static void ch_trig(uint8_t i)
{
    if (i == 0)
    {
        GPIO_ResetBits(ULTRASONIC1_TRIG_PORT, ULTRASONIC1_TRIG_PIN);
        Delay_us(2);
        GPIO_SetBits(ULTRASONIC1_TRIG_PORT, ULTRASONIC1_TRIG_PIN);
        Delay_us(10);
        GPIO_ResetBits(ULTRASONIC1_TRIG_PORT, ULTRASONIC1_TRIG_PIN);
    }
    else
    {
        GPIO_ResetBits(ULTRASONIC2_TRIG_PORT, ULTRASONIC2_TRIG_PIN);
        Delay_us(2);
        GPIO_SetBits(ULTRASONIC2_TRIG_PORT, ULTRASONIC2_TRIG_PIN);
        Delay_us(10);
        GPIO_ResetBits(ULTRASONIC2_TRIG_PORT, ULTRASONIC2_TRIG_PIN);
    }
}

static uint8_t ch_echo(uint8_t i)
{
    if (i == 0)
        return GPIO_ReadInputDataBit(ULTRASONIC1_ECHO_PORT, ULTRASONIC1_ECHO_PIN);
    return GPIO_ReadInputDataBit(ULTRASONIC2_ECHO_PORT, ULTRASONIC2_ECHO_PIN);
}

/* 两路都空闲才能安全刷屏（整屏软 I2C 要 10~15 ms） */
uint8_t Ultra_Idle(void)
{
    if (!DWT_OK) return 1;          /* 计时基准坏了就别再等状态机，否则刷屏永久冻结 */
    return (ch[0].state == 0 && ch[1].state == 0) ? 1 : 0;
}

/* 正在量回波高电平（此时任何 >0.1 ms 的阻塞都会把脉宽算长） */
uint8_t Ultra_Timing(void)
{
    if (!DWT_OK) return 0;
    return (ch[0].state == 2 || ch[1].state == 2) ? 1 : 0;
}

void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = ULTRASONIC1_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ULTRASONIC1_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(ULTRASONIC1_TRIG_PORT, ULTRASONIC1_TRIG_PIN);

    /* Echo 用下拉输入：模块没插时读到低电平，表现为"没有回波"，
       而不是悬空乱跳导致误判障碍 */
    GPIO_InitStructure.GPIO_Pin  = ULTRASONIC1_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(ULTRASONIC1_ECHO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = ULTRASONIC2_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(ULTRASONIC2_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(ULTRASONIC2_TRIG_PORT, ULTRASONIC2_TRIG_PIN);

    GPIO_InitStructure.GPIO_Pin  = ULTRASONIC2_ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(ULTRASONIC2_ECHO_PORT, &GPIO_InitStructure);

    ch[0].last = ULTRA_MAX_CM;
    ch[1].last = ULTRA_MAX_CM;

    DWT_Setup();
    if (!DWT_OK) g_UltraFault = 1;
}

static void apply_result(uint8_t i, float d)
{
    UltraChan *c   = &ch[i];
    float     *last = &c->last;
    uint8_t    accept = 1;

    if (d < 0.0f)
    {
        if (i == 0) { if (fail1 < 1000) fail1++; }
        else        { if (fail2 < 1000) fail2++; }
    }
    else
    {
        if (i == 0) fail1 = 0; else fail2 = 0;

        /* 非对称滤波：逼近时立即跟随（安全优先），远离时缓慢释放 */
        /* 单次刷新内不可能掉这么多：先记账，等下一次刷新确认；
           连续两次都这么近才接受，只出现一次就当毛刺丢掉（否则一次噪声会触发 9 s 绕行） */
        if (d < *last - ULTRA_BIG_DROP_CM)
        {
            if (c->pending && fabsf(d - c->pending_d) < 30.0f)
            {
                *last = d;
                c->pending = 0;
            }
            else
            {
                c->pending   = 1;
                c->pending_d = d;
                accept = 0;
            }
        }
        else
        {
            c->pending = 0;
            if (d < *last) *last = d;                            /* 正常逼近：立即跟随 */
            else           *last = *last + ULTRA_FILTER_ALPHA * (d - *last);   /* 远离：缓慢释放 */
        }

        if (accept)
        {
            if (i == 0)
            {
                g_Ultrasonic1Distance = *last;
                if (d >= ULTRA_VALID_CM) g_UltraQuiet1 = (g_UltraQuiet1 < 999) ? (g_UltraQuiet1 + 1) : 999;
                else                     g_UltraQuiet1 = 0;
            }
            else
            {
                g_Ultrasonic2Distance = *last;
                if (d >= ULTRA_VALID_CM) g_UltraQuiet2 = (g_UltraQuiet2 < 999) ? (g_UltraQuiet2 + 1) : 999;
                else                     g_UltraQuiet2 = 0;
            }
        }
    }

    if (!DWT_OK || fail1 > ULTRA_FAULT_CNT || fail2 > ULTRA_FAULT_CNT) g_UltraFault = 1;
    else                                                              g_UltraFault = 0;
}

void Ultrasonic_Task(void)
{
    uint32_t now;
    uint8_t  i;

    if (!DWT_OK) return;            /* 计时基准失效：不启动新测量，g_UltraFault 已关掉避障 */

    now = cyc();

    /* 1. 推进两路状态机（纯轮询，不等待） */
    for (i = 0; i < 2; i++)
    {
        UltraChan *c = &ch[i];

        if (c->state == 1)
        {
            if (ch_echo(i))
            {
                c->t1 = now;
                c->state = 2;
            }
            else if ((uint32_t)(now - c->t0) > US2CYC(ULTRA_RISE_TIMEOUT_US))
            {
                c->result = ULTRA_MAX_CM;      /* 20 ms 内没有回波 = 前方 4 m 内无目标 */
                c->state = 0;
                c->done = 1;
            }
        }
        else if (c->state == 2)
        {
            if (!ch_echo(i))
            {
                uint32_t dt_us = (uint32_t)(now - c->t1) / us_cycles();
                c->result = (dt_us < 100u) ? -1.0f : (float)dt_us * 0.01715f;
                if (c->result > ULTRA_MAX_CM) c->result = ULTRA_MAX_CM;
                c->state = 0;
                c->done = 1;
            }
            else if ((uint32_t)(now - c->t1) > US2CYC(ULTRA_HIGH_TIMEOUT_US))
            {
                c->result = -1.0f;             /* 高电平过长 = 回波异常 */
                c->state = 0;
                c->done = 1;
            }
        }
    }

    /* 2. 取结果 */
    for (i = 0; i < 2; i++)
    {
        if (ch[i].done)
        {
            ch[i].done = 0;
            apply_result(i, ch[i].result);
        }
    }

    /* 3. 两路都空闲时，按周期交替触发下一路（周期数比较，自然回绕） */
    if (ch[0].state == 0 && ch[1].state == 0 && (int32_t)(cyc() - next_slot) >= 0)
    {
        ch_trig(slot_ch);
        ch[slot_ch].t0 = cyc();
        ch[slot_ch].state = 1;
        slot_ch ^= 1u;
        next_slot = cyc() + US2CYC(ULTRA_PERIOD_MS * 1000u);
    }
}

/* 阻塞式单次测量：只给分模块测试用 */
static float measure_blocking(uint8_t i)
{
    uint32_t start;

    ch_trig(i);
    ch[i].t0 = cyc();
    start = ch[i].t0;
    ch[i].state = 1;

    for (;;)
    {
        uint32_t now = cyc();

        if (ch[i].state == 1)
        {
            if (ch_echo(i)) { ch[i].t1 = now; ch[i].state = 2; }
            else if ((uint32_t)(now - ch[i].t0) > US2CYC(ULTRA_RISE_TIMEOUT_US)) { ch[i].state = 0; return ULTRA_MAX_CM; }
        }
        else if (ch[i].state == 2)
        {
            if (!ch_echo(i))
            {
                uint32_t dt_us = (uint32_t)(now - ch[i].t1) / us_cycles();
                ch[i].state = 0;
                return (dt_us < 100u) ? -1.0f : (float)dt_us * 0.01715f;
            }
            if ((uint32_t)(now - ch[i].t1) > US2CYC(ULTRA_HIGH_TIMEOUT_US)) { ch[i].state = 0; return -1.0f; }
        }

        if ((uint32_t)(now - start) > US2CYC(40000u)) { ch[i].state = 0; return -1.0f; }
    }
}

float Ultrasonic1_MeasureDistance(void)
{
    return measure_blocking(0);
}

float Ultrasonic2_MeasureDistance(void)
{
    return measure_blocking(1);
}

float LowPassFilter(float new_value, float *last_value, float alpha)
{
    *last_value = alpha * new_value + (1.0f - alpha) * (*last_value);
    return *last_value;
}
