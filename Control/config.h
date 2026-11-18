#ifndef __CONFIG_H
#define __CONFIG_H

/* =====================================================================
 *  参赛参数集中配置（2026-11 整改版）
 *  现场 15 分钟只改这一个文件，不要到处翻 .c
 *  速度单位：编码器脉冲 / 10 ms 控制周期
 *           100 约等于 1.5 m/s（65 mm 轮、1:30 减速、11 线编码器）
 * ===================================================================== */

/* ---------------- 0. 节拍 ---------------- */
#define CTRL_DT_S               0.010f      /* 控制周期 10 ms，与 Timer.c 一致 */
#define CTRL_DT_MS              10u

/* ---------------- 1. 循迹 ---------------- */
#define TRACK_SPEED             100         /* 正常巡航速度 */
#define SPEED_RAMP_STEP         4           /* 每周期速度变化上限，抑制阶跃冲击 */
#define FUSED_ALPHA             0.7f        /* 灰度/陀螺互补系数 */
#define FUSED_TIMES             10.0f       /* 偏差放大到 ±30 量级 */
#define FUSED_LIMIT             30.0f       /* 融合偏差限幅 */
#define FUSED_LP_ALPHA          0.5f        /* 输出低通 */
#define LINE_LOST_DECAY         0.85f       /* 丢线时误差每周期衰减（先走直线过缝隙） */

/* ---------------- 2. 丢线自救 ---------------- */
#define LINE_LOST_GRACE_MS      150         /* 先直行穿越缝隙的时间 */
#define LINE_LOST_SPEED         45          /* 搜索速度 */
#define LINE_LOST_SEARCH_DIFF   45          /* 搜索时固定差速 */
#define LINE_LOST_SEARCH_MS     1400        /* 单方向搜索时长，左右交替 */
#define START_BLIND_MS          600         /* 起步后还没见过线：只直行这么久，不扫线 */
#define LINE_LOST_GIVEUP_MS     3000        /* 放弃并停车 */

/* ---------------- 3. 避障 ---------------- */
#define ULTRA1_IS_LEFT          1           /* 1 号探头装左侧=1，装右侧=0 */
#define AVOID_SLOW_CM           80.0f       /* 进入减速区 */
#define AVOID_SLOW_SPEED        45          /* 接近段速度：45 约 0.7 m/s，确认窗口内少走 3 cm */
#define AVOID_TRIGGER_CM        45.0f       /* 确认触发距离 */
#define AVOID_CONFIRM_CNT       6           /* 连续节拍数：6 × 10 ms ≈ 60 ms ≈ 一次测距刷新 */
#define AVOID_SIDE_DIFF_CM      8.0f        /* 左右距离差超过它才判侧 */
#define AVOID_TURN_ANGLE        60.0f       /* 外转角度 */
#define AVOID_BACK_ANGLE        55.0f       /* 回转角度 */
#define AVOID_DRIVE_MM          380.0f      /* 外转后直行距离 */
#define AVOID_SPEED             45
#define AVOID_TURN_DIFF         90
#define AVOID_RETURN_DIFF       35
#define AVOID_STEP_TIMEOUT_MS   2500u       /* 单步超时 */
#define AVOID_TOTAL_TIMEOUT_MS  9000u       /* 整个绕行超时，超时停车 */
#define AVOID_RETRY_ANGLE       12.0f       /* 找线失败时再多转的角度 */
#define AVOID_RETRY_MAX         2           /* 最多重试几次，之后停车等人工 */
#define AVOID_TURN_MS_FALLBACK  240u        /* 陀螺失效时的定时间转角替代（ms），需台架实测后填 */

/* ---------------- 4. 终点停车 ---------------- */
#define CM_PER_COUNT_LR         0.0077347f  /* 名义值：π×65mm/1320/2。必须手推 1000 mm 标定 LOC_CAL */
#define START_DELAY_S           3           /* 上电后的放车倒计时（秒），期间不动车 */
#define FINISH_MIN_TRAVEL_MM    4000.0f     /* 只有里程超过它，全黑才允许判为终点 */
#define FINISH_LINE_INDEX       1           /* 4 m 之后第几次全黑算终点；场地黑区多就加到 2/3 */
#define FINISH_SLOW_MM          3500.0f     /* 最后这段降到 45（STOP_APPROACH_SPEED），把压线采样窗口从 1.3 拍拉到 2.9 拍 */
#define FINISH_CONFIRM_CNT      2           /* 连续 N 拍全黑才认压线 */
#define FINISH_MIN_GAP_MM       800.0f      /* 两次全黑之间的最小里程，防误判 */
#define STOP_ADVANCE_MM         40.0f       /* 压线后继续前进的距离：车尾不得越线，留足余量 */
#define STOP_APPROACH_SPEED     45        /* 末端与压线后的接近速度：45≈0.7 m/s，20 mm 线停留 28.6 ms（2.9 拍），2 拍确认才够余量 */
#define STOP_BRAKE_PWM          7100        /* 短接制动占空比：接近满占空（ARR=7199） */
#define STOP_BRAKE_MS           150u

/* ---------------- 5. 超声波 ---------------- */
#define ULTRA_PERIOD_MS         30u         /* 两路交替，单路约 60 ms 刷新一次 */
#define ULTRA_FILTER_ALPHA      0.5f
#define ULTRA_MAX_CM            400.0f
#define ULTRA_BIG_DROP_CM       150.0f      /* 单次刷新内掉得比它多，先当毛刺，等下一次确认 */
#define ULTRA_VALID_CM          399.0f      /* 小于它才算"看到目标" */
#define ULTRA_FAULT_CNT         40u         /* 连续异常次数达到即判故障 */
#define ULTRA_RISE_TIMEOUT_US   20000u      /* 等上升沿超时 = 前方无目标 */
#define ULTRA_HIGH_TIMEOUT_US   30000u      /* 高电平超时 = 回波异常 */

/* ---------------- 6. 里程与极性 ---------------- */
#define MM_PER_LOC              10.0f       /* Location 单位为 cm */
#define MM2LOC(mm)              ((mm) / MM_PER_LOC)
#define LOC_CAL                 1.0f        /* 里程标定：手推 1000 mm 读 s（单位 cm），填 100/读数 */
#define MOTOR_L_INVERT          0
#define MOTOR_R_INVERT          0
#define ENCODER_L_INVERT        1           /* 与作者 Encoder_Get_L 取负一致 */
#define ENCODER_R_INVERT        0

/* ---------------- 7. 保护 ---------------- */
#define USE_IWDG                0           /* 比赛时置 1；调试打断点时置 0 */
#define IWDG_RELOAD_VAL         625u        /* LSI 40 kHz /64，625 计约 1 s（LSI 实际 30-60 kHz） */
#define MYI2C_DELAY_US          2           /* 软 I2C 半周期；10 us 时一次陀螺读要 1.2 ms */
#define MPU_FAIL_CNT            20u         /* 连续 I2C 失败次数达到即判陀螺故障 */

#endif
