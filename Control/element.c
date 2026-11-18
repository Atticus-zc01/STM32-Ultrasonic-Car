#include "myfile.h"

/************************元素***************************/

//变量定义

uint8_t Element_Flag=0;
uint8_t Ten_Flag=0;
uint8_t Ring_Flag=0;
uint8_t Noline_Flag=0;
uint8_t Stop_Flag = 0;
uint8_t MoreRing_Flag=0;
uint8_t Right_Flag=0;
uint8_t Turnback_Flag=0;
uint8_t Busy_Flag=0;
uint8_t Busy_left=0;
uint8_t Busy_mid=0;
uint8_t Busy_right=0;
uint8_t Busy_time=0;
uint8_t Busy_Flag2=0;
int Turnback_time=0;

uint8_t K=0;
int Speed_Choice[5]={100,0,30,0,0};    //速度决策
int Ring_Turn=0;
int Ten_Temp=0;
/************************元素控制台***************************/
//在这里启动或者关掉某个元素

/* =====================================================================
 *  元素状态机
 *  Element_Flag : 0 正常循迹 / 1 避障(含接近减速) / 2 停车结束 / 3 丢线搜索
 *  Busy_Flag    : 0 空闲 / 1 外转 / 2 直行 / 3 回转 / 4 找线 / 5 失败停车
 *  Busy_Flag2   : 1 左绕 / 2 右绕
 *
 *  避障流程：80 cm 减速 → 45 cm 连续确认 → 判侧 → 60° 外转 → 380 mm 直行
 *            → 回转 55° → 带偏置找线，每步与整体都有超时
 * ===================================================================== */

uint32_t Ctrl_Tick = 0;

static uint8_t  Busy_Cnt  = 0;
static uint8_t  Busy_Retry = 0;      /* 找线重试次数 */
static uint32_t Busy_T0   = 0;
static uint32_t Busy_TAll = 0;
static float    Busy_Loc0 = 0.0f;

static void Busy_End(void)
{
    Busy_Flag    = 0;
    Busy_Cnt     = 0;
    Element_Flag = 0;
    Place_Enable = 1;
    Place_Out    = 0;
    Basic_Speed  = TRACK_SPEED;
    PID_Reset();
}

static void Busy_Start(uint8_t side)
{
    Busy_Flag2   = side;
    Busy_Flag    = 1;
    Busy_Cnt     = 0;
    Busy_Retry   = 0;
    Busy_T0      = Ctrl_Tick;
    Busy_TAll    = Ctrl_Tick;
    Element_Flag = 1;
    Place_Enable = 0;
    Clear_yaw();
    PID_ResetPlace();           /* 清转向环 D 项，避免用几秒前的误差 */
    Basic_Speed  = AVOID_SPEED;
}

/* 转角判定：有陀螺时看角度，陀螺失效时退化为定时 */
static uint8_t TurnDone(uint32_t T0, float Angle)
{
    if (g_MPUFault)
        return ((Ctrl_Tick - T0) >= AVOID_TURN_MS_FALLBACK) ? 1 : 0;

    return (fabsf(yaw) >= Angle) ? 1 : 0;
}

void Element_Process(void)
{
    Ctrl_Tick += CTRL_DT_MS;

    Element_Normal();
    Element_Stop();
    Element_Noline();
    Element_Busy();
}

void Element_Busy(void)
{
    float  d1, d2, dL, dR, dmin;
    int8_t dir;

    if (Element_Flag == 2) return;              /* 已停车 */
    if (Stop_Flag >= 1)    return;              /* 正在终点流程 */

    if (g_UltraFault)                           /* 测距不可信：不冒险避障 */
    {
        if (Busy_Flag != 0) Busy_End();
        if (Element_Flag == 0) Basic_Speed = AVOID_SLOW_SPEED;
        return;
    }

    d1 = (float)g_Ultrasonic1Distance;
    d2 = (float)g_Ultrasonic2Distance;

#if ULTRA1_IS_LEFT
    dL = d1; dR = d2;
#else
    dL = d2; dR = d1;
#endif

    dmin = (dL < dR) ? dL : dR;

    /* ---------------- 空闲：减速 + 判侧 ---------------- */
    if (Busy_Flag == 0)
    {
        if (dmin < AVOID_TRIGGER_CM)
        {
            Busy_Cnt++;
            if (Busy_Cnt >= AVOID_CONFIRM_CNT)
            {
                /* 判侧：两侧都有有效读数时比较远近；
                   只有一侧看到目标时，绕向另一侧；都看不到才默认右绕 */
                if (dL < ULTRA_VALID_CM && dR < ULTRA_VALID_CM)
                {
                    if (fabsf(dL - dR) > AVOID_SIDE_DIFF_CM)
                        dir = (dL < dR) ? 2 : 1;    /* 左侧更近 -> 右绕；右侧更近 -> 左绕 */
                    else
                        dir = 2;                    /* 正对前方，默认右绕 */
                }
                else if (dL < ULTRA_VALID_CM)
                {
                    dir = 2;                        /* 只有左侧看到目标 -> 向右绕 */
                }
                else if (dR < ULTRA_VALID_CM)
                {
                    dir = 1;                        /* 只有右侧看到目标 -> 向左绕 */
                }
                else
                {
                    dir = 2;
                }

                Busy_Start((uint8_t)dir);
                return;
            }
        }
        else
        {
            Busy_Cnt = 0;
        }

        if (dmin < AVOID_SLOW_CM && Element_Flag == 0)
        {
            Element_Flag = 1;
            Basic_Speed  = AVOID_SLOW_SPEED;    /* 提前减速，留出判侧时间 */
        }
        else if (dmin >= AVOID_SLOW_CM && Element_Flag == 1)
        {
            Element_Flag = 0;
            Basic_Speed  = TRACK_SPEED;
        }
        return;
    }

    dir = (Busy_Flag2 == 1) ? 1 : -1;           /* +1 左绕，-1 右绕 */

    /* ---------------- 绕行状态机 ---------------- */
    switch (Busy_Flag)
    {
    case 1:                                     /* 1. 外转 */
        Place_Enable = 0;
        Place_Out    = dir * AVOID_TURN_DIFF;
        Basic_Speed  = AVOID_SPEED;

        if (TurnDone(Busy_T0, AVOID_TURN_ANGLE) ||
            (Ctrl_Tick - Busy_T0) > AVOID_STEP_TIMEOUT_MS)
        {
            Place_Out  = 0;
            Busy_Loc0  = Location;
            Busy_T0    = Ctrl_Tick;
            Busy_Flag  = 2;
        }
        break;

    case 2:                                     /* 2. 直行绕过 */
        Place_Enable = 0;
        Place_Out    = 0;
        Basic_Speed  = AVOID_SPEED;

        if ((Location - Busy_Loc0) >= MM2LOC(AVOID_DRIVE_MM) ||
            (Ctrl_Tick - Busy_T0) > AVOID_STEP_TIMEOUT_MS)
        {
            Clear_yaw();
            Busy_T0   = Ctrl_Tick;
            Busy_Flag = 3;
        }
        break;

    case 3:                                     /* 3. 回转，车头朝回赛道 */
        Place_Enable = 0;
        Place_Out    = -dir * AVOID_TURN_DIFF;
        Basic_Speed  = AVOID_SPEED;

        /* 第一次转 AVOID_BACK_ANGLE，重试时只多转 AVOID_RETRY_ANGLE */
        if (TurnDone(Busy_T0, (Busy_Retry > 0) ? AVOID_RETRY_ANGLE : AVOID_BACK_ANGLE) ||
            (Ctrl_Tick - Busy_T0) > AVOID_STEP_TIMEOUT_MS)
        {
            Busy_T0   = Ctrl_Tick;
            Busy_Flag = 4;
        }
        break;

    case 4:                                     /* 4. 带偏置找线 */
        Place_Enable = 0;
        Place_Out    = -dir * AVOID_RETURN_DIFF;
        Basic_Speed  = AVOID_SPEED;

        if (M == 1 || L1 == 1 || R1 == 1)
        {
            Busy_End();                         /* 回到线上，交还循迹环 */
        }
        else if ((Ctrl_Tick - Busy_T0) > AVOID_STEP_TIMEOUT_MS)
        {
            if (Busy_Retry < AVOID_RETRY_MAX)
            {
                Busy_Retry++;
                Clear_yaw();                    /* 不清零，case 3 的 TurnDone 立刻为真，重试等于没转 */
                Busy_T0   = Ctrl_Tick;
                Busy_Flag = 3;
            }
            else
            {
                Busy_Flag = 5;                  /* 两次都没找到：停车等人工 */
            }
        }
        break;

    case 5:                                     /* 5. 失败：停车等人工 */
        PWM_Enable   = 0;
        Place_Enable = 0;
        Basic_Speed  = 0;
        Element_Flag = 2;
        Motor_Stop();
        break;

    default:
        Busy_Flag = 0;
        break;
    }

    /* 整个绕行总超时：卡住就停车，绝不无限转圈 */
    if (Busy_Flag >= 1 && Busy_Flag <= 4 &&
        (Ctrl_Tick - Busy_TAll) > AVOID_TOTAL_TIMEOUT_MS)
    {
        Busy_Flag = 5;
    }
}
void Element_Normal()
{
 if(Element_Flag==0)
 {
	 Place_Enable=1;
	 PWM_Enable=1;
	 Basic_Speed=TRACK_SPEED;
 }
}


//十字识别，辅助判断圆环，我的圆环在十字之后，根据赛道情况自己调整
void Element_Ten()
{
	if ((Ten_Flag==0)&&(L2 == 1 && L1 == 1 && M == 1 && R1 == 1 && R2 == 1))
	{
		Clear_Location();
		Element_Flag=1;
		Ten_Flag=1;  	
	}	
	if(Ten_Flag==1||Ten_Flag==2)
	{		
		if(Location<100)Basic_Speed=100;	
		else if(Location>150&&Location<650){Basic_Speed=Speed_Choice[0]+5;}
		else if(Location>700&&Location<800){Basic_Speed=Speed_Choice[3];}
		else{Basic_Speed=Speed_Choice[0];}
	}		
	if(Ten_Flag==1&&Location>30)
	{
		Ten_Flag=2;	
	}

	if ((Ten_Flag==2)&&(L2 == 1 && L1 == 1 && M == 1 && R1 == 1 && R2 == 1))
	{
		Clear_Location();
		Ten_Flag=3;  
		Place_Enable=0;//关掉转向环  
		Place_Out=80; //给一个固定的差速
	}	
	if(Ten_Flag==3&&Location>10&& M == 1)
	{
	 Place_Enable=1;
	 Ten_Flag=4;			
	}	
	if(Ten_Flag==4)
	{		
		if(Location<50){Basic_Speed=Speed_Choice[0]-5;}
		else if(Location>100&&Location<150){Basic_Speed=50;}
		else if(Location>150&&Location<200){Basic_Speed=Speed_Choice[2];}
		else if(Location>200&&Location<270){Basic_Speed=Speed_Choice[2]-15;}
		else
		{Basic_Speed=Speed_Choice[0];}
	}	 
	if(Ten_Flag==4&&L2 == 1 && L1 == 1 && M == 1 && R1 == 1 && R2 == 1)
	{
		Clear_yaw();	
		Basic_Speed=40;
		Ten_Flag=5;
		Clear_Location();
		Place_Enable=0;//关掉转向环  
		Place_Out=-75; //给一个固定的差速   		
	}	
	if(Ten_Flag==5&&(Location>10&&fabs(yaw)>20))
	{
		Basic_Speed=45;
		Place_Out=0;
		Ten_Flag=6;
		Clear_Location();
	}	
	if(Ten_Flag==6&&(Location>10&&M == 1))
	{
		Place_Enable=1;
		Ten_Flag=7;
	}	
	if(Ten_Flag==7&&((L2 == 1&& R2 == 1)))//(L2 == 1&& R2 == 0)||
	{
		Ten_Flag=8;
		Clear_Location(); 
		Place_Enable=0;//关掉转向环  
		Place_Out=-60; //给一个固定的差速   
	}
	if(Ten_Flag==8&&(Location>5&&M == 1))
	{
		Place_Enable=1;//关掉转向环  	
		Ten_Flag=9;
		Basic_Speed=Speed_Choice[0]-5;
	}		
	if(Ten_Flag==9)
	{
		if(Location>120&&Location<200)
		{Basic_Speed=Speed_Choice[2]+5;}
		else{Basic_Speed=Speed_Choice[0]-5;}
	}
	
	if(Ten_Flag==9 &&(L2 == 1 && L1 == 1 && M == 1 && R1 == 1 && R2 == 1)&&Location>150)
	{
	 Clear_yaw();	
	 Clear_Location();
	 Place_Enable=0;
	 Ten_Flag=10;	
	 Place_Enable=0;//关掉转向环  
	 Place_Out=-90; //给一个固定的差速
	}	
	if(Ten_Flag==10&&Location>10&&M == 1 )
	{
	 Place_Enable=1;
	 Ten_Flag=11;
	}
	
	if(Ten_Flag==11 &&(L2 == 0&& R2 == 1)&&yaw>60)
	{
	 Clear_Location();
	 Place_Enable=0;
	 Ten_Flag=12;	
	 Place_Enable=0;//关掉转向环  
	 Place_Out=-90; //给一个固定的差速
	}	
	if(Ten_Flag==12&&Location>8&&M == 1 )
	{
	 Place_Enable=1;
	 Ten_Flag=13;
	 Basic_Speed=Speed_Choice[2]-10;
	}	
	
}	
//圆环
void Element_Ring()
{
	if(Element_Flag==0&&Ring_Flag==0&&Noline_Flag==4&&Ten_Flag==1)	//调试状态下可以将&&Noline_Flag==4&&Ten_Flag==1移出，这里我是比赛为了防止误触，做了一个元素顺序限制
	{
			//第一次检测入环
			if(R2==1&&M==1&&(L1==0||R1==0)&&L2==0)//初次检测
		{		
				Ring_Flag=1;	
				Clear_Location();			
		}
	}
		//第一次检测到后过10cm再进行检测然后入环
	if(Ring_Flag==1&&Location>=10.5)
	{		
		if((L1==0&&M==1&&R1==0)||((L1==1||R1==1)&&M==1))
		{
			Ring_Flag=2;	
			Clear_Location();	
			Element_Flag=1;	
			Place_Enable=0;//关掉转向环
			Place_Out=-15; //给一个固定的差速
		}
		else{Ring_Flag=0;}
	}
		//结束入环动作，正常巡线
	if(Ring_Flag==2&&Location>30)
	{
		Place_Enable=1;
		Ring_Flag=3;
		Basic_Speed=Speed_Choice[2];
		
	}
	//出环动作和入环同理
	if(Ring_Flag==3&&Location>380)
	{
		if(L2==1&&M==1&&R2==0)
		{
			Clear_Location();	
			Place_Enable=0;
			Place_Out=-15;	
			Ring_Flag=4;			
		}
	}
	//出环结束
	if(Ring_Flag==4&&Location>50)
	{
		Element_Flag=0;
		K=1;
		Ring_Flag=5;
		Clear_Location();
		
	}
}


//丢线   
/* =====================================================================
 *  丢线自救
 *  0 -> 1：检测到五路全灭，先直行 LINE_LOST_GRACE_MS 冲过缝隙
 *  1 -> 2：仍未找到，进入慢速左右扫线（左右交替）
 *  2 -> 3：累计超过 LINE_LOST_GIVEUP_MS 仍找不到，停车等人工
 *  任何时刻只要有一路压线，立即回到正常循迹
 *  绕行期间（Busy_Flag != 0）只暂停、不清状态，避免计时被反复重置
 * ===================================================================== */

static int8_t   Noline_Dir = 1;
static uint8_t  Line_Seen  = 0;      /* 本次上电是否见过线：没见过就不自救，只直行 */
static uint32_t Noline_T0  = 0;
static uint32_t Noline_TPh = 0;

void Element_Noline(void)
{
    if (Element_Flag == 2)                          /* 已停车 */
    {
        Noline_Flag = 0;
        return;
    }

    if (Busy_Flag != 0) return;                     /* 正在绕行：暂停，保留计时与状态 */

    if (Line_Active)
    {
        Line_Seen = 1;

        if (Noline_Flag == 1 || Noline_Flag == 2)   /* 掉线后找回 */
        {
            Noline_Flag  = 0;
            Element_Flag = 0;
            Place_Enable = 1;
            Place_Out    = 0;
            Basic_Speed  = TRACK_SPEED;
            PID_Reset();
        }
        return;
    }

    switch (Noline_Flag)
    {
    case 0:                                         /* 刚开始丢线 */
        Noline_Flag = 1;
        Noline_T0   = Ctrl_Tick;
        break;

    case 1:                                         /* 直行穿越缝隙 */
        /* 起步时车头压在线上、灰度排还在白底上，这里是常态而不是故障：
           没见过线之前最多直行 START_BLIND_MS，绝不进入左右扫线 */
        if ((Ctrl_Tick - Noline_T0) > (Line_Seen ? LINE_LOST_GRACE_MS : START_BLIND_MS))
        {
            Noline_Flag  = 2;
            Element_Flag = 3;
            Noline_TPh   = Ctrl_Tick;
            Noline_Dir   = 1;
        }
        break;

    case 2:                                         /* 慢速左右扫线 */
        Element_Flag = 3;                           /* 每拍补标志，避免被 Busy_End 清掉后失去语义 */
        Basic_Speed  = LINE_LOST_SPEED;
        Place_Enable = 0;
        Place_Out    = Noline_Dir * LINE_LOST_SEARCH_DIFF;

        if ((Ctrl_Tick - Noline_TPh) > LINE_LOST_SEARCH_MS)
        {
            Noline_TPh = Ctrl_Tick;
            Noline_Dir = -Noline_Dir;               /* 换个方向再找 */
        }

        if ((Ctrl_Tick - Noline_T0) > LINE_LOST_GIVEUP_MS)
        {
            Noline_Flag  = 3;
            Element_Flag = 2;
            PWM_Enable   = 0;
            Place_Enable = 0;
            Basic_Speed  = 0;
            Motor_Stop();
        }
        break;

    default:
        break;
    }
}
/* =====================================================================
 *  终点停车
 *   0 找终点线：4 m 之前的全黑一律忽略（起步时的起始线也在其中）；
 *              里程超过 FINISH_MIN_TRAVEL_MM 之后的全黑才允许判为终点
 *   1 压线后按 STOP_ADVANCE_MM 继续前进（保证确实过线）
 *   2 电机短接制动 STOP_BRAKE_MS（满占空比，真正抱死而不是滑行）
 *   3 停稳、关闭输出
 *  避障与找线期间不参与判定，避免路过的黑区把计数带偏
 * ===================================================================== */

static uint8_t  Black_Cnt   = 0;
static uint8_t  Black_Hold  = 0;     /* 连续全黑拍数，去抖用 */
static float    Last_BlackL = 0.0f;
static float    Stop_Loc0   = 0.0f;
static uint32_t Stop_T0     = 0;

static void Stop_Begin(void)
{
    Stop_Flag    = 1;
    Element_Flag = 2;
    Stop_Loc0    = Location;
    Stop_T0      = Ctrl_Tick;
    Place_Enable = 1;
    Basic_Speed  = STOP_APPROACH_SPEED;
}

void Element_Stop(void)
{
    if (Busy_Flag != 0 || Element_Flag == 3) return;    /* 正在绕行或找线：不判终点（80 cm 减速区不算） */
    if (Element_Flag == 2 && Stop_Flag == 0)    return;

    switch (Stop_Flag)
    {
    case 0:                                         /* 等终点线 */
        /* 最后 3.5 m 降到 45（STOP_APPROACH_SPEED）：20 mm 的线只走 12.9 ms（1.3 拍），降速后是 28.6 ms（2.9 拍），
           连续确认才采得到，顺带把停车精度也提上去 */
        if (Location >= MM2LOC(FINISH_SLOW_MM) && Element_Flag == 0)
            Basic_Speed = STOP_APPROACH_SPEED;

        if (L2 == 1 && L1 == 1 && M == 1 && R1 == 1 && R2 == 1)
        {
            if (Black_Hold < 250) Black_Hold++;
        }
        else
        {
            Black_Hold = 0;
        }

        if (Black_Hold == FINISH_CONFIRM_CNT)          /* 只在刚满足的那一拍处理一次 */
        {
            /* 只认里程 4 m 之后、且与前一次黑区间隔 800 mm 的全黑；
               连续第 FINISH_LINE_INDEX 次才真正停车（默认 1 次即停） */
            if (Location >= MM2LOC(FINISH_MIN_TRAVEL_MM) &&
                (Last_BlackL == 0.0f || (Location - Last_BlackL) > MM2LOC(FINISH_MIN_GAP_MM)))
            {
                Black_Cnt++;
                Last_BlackL = Location;
                if (Black_Cnt >= FINISH_LINE_INDEX) Stop_Begin();
            }
        }
        break;

    case 1:                                         /* 压线后继续前进一小段 */
        Basic_Speed = STOP_APPROACH_SPEED;
        if ((Location - Stop_Loc0) >= MM2LOC(STOP_ADVANCE_MM) ||
            (Ctrl_Tick - Stop_T0) > 800u)
        {
            Stop_Flag = 2;
            Stop_T0   = Ctrl_Tick;
        }
        break;

    case 2:                                         /* 短接制动 */
        PWM_Enable   = 0;
        Place_Enable = 0;
        Basic_Speed  = 0;
        Motor_Brake(STOP_BRAKE_PWM);

        if ((Ctrl_Tick - Stop_T0) >= STOP_BRAKE_MS)
        {
            Motor_Stop();
            Stop_Flag = 3;
        }
        break;

    case 3:                                         /* 停稳 */
        PWM_Enable   = 0;
        Place_Enable = 0;
        Basic_Speed  = 0;
        break;

    default:
        break;
    }
}
void Element_MoreRing()
{
 if(Element_Flag==1&&MoreRing_Flag==0&&Ten_Flag==8)
	{ 
		if (L2 == 1&& L1 == 1&& M == 1&& R1 == 1)
		{
			MoreRing_Flag=1;
			Element_Flag=2;
			Place_Enable=0;
			Place_Out=-35;
			Clear_Location();	
		}
	}
 if(MoreRing_Flag==1&&Location>3)
	{
		Place_Enable=1;
		MoreRing_Flag=2;
	}	
 if ( MoreRing_Flag==2 && L1 == 1&& M == 1&& R1 == 1&&R2 == 1 )
		{
			MoreRing_Flag=3;
			Place_Enable=0;
			Place_Out=35;
			Clear_Location();	
		}
	
 if(MoreRing_Flag==3&&Location>3)
	{
		Place_Enable=1;
		MoreRing_Flag=4;
	}	
if ( MoreRing_Flag==4&&L2 == 1&& L1 == 1&& M == 1&& R1 == 1 )
		{
			MoreRing_Flag=5;
			Place_Enable=0;
			Place_Out=-35;
			Clear_Location();	
		}
	
 if(MoreRing_Flag==5&&Location>5)
	{
		Place_Enable=1;
		MoreRing_Flag=6;
	}	
if ( MoreRing_Flag==6&& R1 ==1&& R2 ==1 )
		{
			MoreRing_Flag=7;
			Place_Enable=0;
			Place_Out=-80;
			Clear_Location();	
		}
	
 if(MoreRing_Flag==7&&Location>5&&M == 1)
	{
		Place_Enable=1;
		MoreRing_Flag=8;
	}	
}

void Element_Turnback()  //掉头
{
	if (Turnback_Flag==0&&(L2 == 1&& L1 == 1&& M == 1&& R1 == 1)) //第一个十字
	{
		Element_Flag=1;
		Turnback_Flag=1;
		Clear_Location();			
	}
	if(Turnback_Flag==1&&Location>=50)	//经过十字防误触
	{Turnback_Flag=2;}
	
	
	
	if (Turnback_Flag==2&&(L2 == 1&& L1 == 1&& M == 1&& R1 == 1)) //第二个十字
	{
		Clear_Location();	
		Turnback_Flag=3;
	}
	if(Turnback_Flag==3&&Location>=50) //经过十字防误触
	{Turnback_Flag=4;}	
	
	
if (Turnback_Flag==4&&(L2 == 1&& L1 == 1&& M == 1&& R1 == 1)) //第三个十字
	{
		Clear_Location();	
		Turnback_Flag=5;
	}
	if(Turnback_Flag==5&&Location>=50) //经过十字防误触
	{Turnback_Flag=6;}		
	
	
	if (Turnback_Flag==6&&(L2 == 1&& L1 == 1&& M == 1&& R1 == 1)) //终点线识别
	{		
		Place_Enable=0;
		Basic_Speed=Speed_Choice[1];	//停车
		Turnback_Flag=7;
	}
	if(Turnback_Flag==7)
	{
		Turnback_time++;      //停车计时器
		Clear_Location();
	}
	if (Turnback_Flag==7&&Turnback_time>=200)  //停车2s
	{ 				
		Turnback_Flag=8;
	}
	if (Turnback_Flag==8)
	{			
		Place_Enable=0;	
		PWM_Enable=0;	
		Turnback_time=0;
		Turnback_Flag=9;		
	}
	if (Turnback_Flag==9)    //开始掉头
	{
		Place_Enable=0;	
		Motor_SetPWM_L(2000);
		Motor_SetPWM_R(-2000);	
		Turnback_time++;
	}
	if (Turnback_Flag==9&& M == 1&&Turnback_time>50)  //掉头防误触机制，重新巡线
	{
		Turnback_time=0;
		Place_Enable=1;
		PWM_Enable=1;	
		Turnback_Flag=10;
		Basic_Speed=Speed_Choice[0];  //恢复正常速度
	}	
	
	if (Turnback_Flag==10&&(L2 == 1&& L1 == 1&& M == 1&& R1 == 1))  //跷跷板前识别
	{
		Turnback_Flag=11;
		Clear_Location();			
	}
	if(Turnback_Flag==11&&Location>=150)  //跷跷板停车  自己调试location来达到平衡点
	{
		Basic_Speed=Speed_Choice[1];			
		Turnback_time++;	
	}

	if(Turnback_Flag==11&&Turnback_time>=500)   //停车5s
	{
		Turnback_Flag=12;
		Basic_Speed=Speed_Choice[0];	
	}
			
	if (Turnback_Flag==12&&(L2 == 1&& L1 == 1&& M == 1&& R1 == 1))   
	{
		Turnback_Flag=13;
		Clear_Location();			
	}
	if(Turnback_Flag==13&Location>=30)
	{Turnback_Flag=14;}	
	
	
	if (Turnback_Flag==14&&(L2 == 1&& L1 == 1&& M == 1&& R1 == 1))  //终点线停车
	{
		Turnback_Flag=15;
		Clear_Location();			
	}
	if(Turnback_Flag==15&Location>=10)  //车身压住终点线 距离自定
	{
		Turnback_Flag=16; 
		Basic_Speed=Speed_Choice[1];	
	}	
		
}









