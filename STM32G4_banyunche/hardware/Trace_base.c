/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制 — PID 与 Stanley 两种控制器分开实现
 *        Trace_LineFollow() 按 TRACE_USE_STANLEY 选择调用哪个
 */

#include "Common_used.h"
#include "stdlib.h"
#include "math.h"

/* ======================== 共用状态 (供打印/调试) ======================== */
float g_trace_v      = 0.0f;
float g_trace_w      = 0.0f;
float g_trace_angle  = 0.0f;
float g_trace_posx   = 0.0f;
float g_trace_target = 0.0f;

/* ================================================================
 * PID 控制器 — 串级 PID
 *   外环: 角度偏差 → PID → 目标位置
 *   内环: 位置误差 → PID → 角速度 w → Mecanum_Calc → 电机
 * ================================================================ */
static pid_type_def g_pid_angle;
static pid_type_def g_pid_pos;
static uint8_t g_pid_inited = 0;

void Trace_LineFollow_PID(void)
{
    float v, w;
    MecanumResult motor;

    /* ---- 1. 获取 K230 角度 + 位置 ---- */
    float k230_angle, k230_posx = 0.0f;
    if (!K230_GetLineAngle(&k230_angle)) {
        return;
    }
    (void)K230_GetPosition(&k230_posx, NULL);

    /* ---- 2. PID 初始化 ---- */
    if (!g_pid_inited) {
        const fp32 ak[3] = { ANGLE_KP, ANGLE_KI, ANGLE_KD };
        const fp32 pk[3] = { POS_KP,   POS_KI,   POS_KD };
        PID_init(&g_pid_angle, PID_POSITION, ak, ANGLE_OUT_MAX, 0.0f);
        PID_init(&g_pid_pos,   PID_POSITION, pk, TRACE_W_MAX, POS_INTEGRAL_MAX);
        g_pid_inited = 1;
    }

    /* ---- 3. 外环: 角度 → 目标位置 ---- */
    float target_posx = PID_calc(&g_pid_angle, k230_angle, 0.0f);
    if (target_posx >  ANGLE_OUT_MAX) target_posx =  ANGLE_OUT_MAX;
    if (target_posx < -ANGLE_OUT_MAX) target_posx = -ANGLE_OUT_MAX;

    /* ---- 4. 内环: 位置 → 角速度 w ---- */
    w = PID_calc(&g_pid_pos, k230_posx, target_posx);
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 5. 速度自适应 ---- */
    v = TRACE_BASE_SPEED;
    {
        float abs_err = fabsf(k230_angle);
        if (abs_err > 30.0f)       v = TRACE_BASE_SPEED * 0.5f;
        else if (abs_err > 15.0f)  v = TRACE_BASE_SPEED * 0.75f;
    }

    /* ---- 6. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = k230_angle;
    g_trace_posx   = k230_posx;
    g_trace_target = target_posx;

    /* ---- 7. 麦轮解算 + 发送 ---- */
    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}

/* ================================================================
 * Stanley 控制器 — 横向误差 e + 航向误差 θ → 转向角 δ → 角速度 w
 *   δ = θ + atan(k·e/v),  w = v·tan(δ)/L
 * ================================================================ */
void Trace_LineFollow_Stanley(void)
{
    float v, w;
    MecanumResult motor;

    /* ---- 1. 获取 K230 角度 + 位置 ---- */
    float k230_angle, k230_posx = 0.0f;
    if (!K230_GetLineAngle(&k230_angle)) {
        return;
    }
    (void)K230_GetPosition(&k230_posx, NULL);

    /* ---- 2. 速度自适应 ---- */
    v = TRACE_BASE_SPEED;
    {
        float abs_err = fabsf(k230_angle);
        if (abs_err > 30.0f)       v = TRACE_BASE_SPEED * 0.5f;
        else if (abs_err > 15.0f)  v = TRACE_BASE_SPEED * 0.75f;
    }
    if (v < MIN_SPEED) v = MIN_SPEED;   /* 防止除零 */

    /* ---- 3. Stanley 控制器 ---- */
    float e     = k230_posx;                    /* 横向误差 (右侧为正) */
    float theta = k230_angle;                   /* 航向误差 (度) */
    float cross_rad = atanf(STANLEY_K * e / v); /* 横向误差项 (弧度) */
    float cross_deg = cross_rad * 180.0f / (float)M_PI;
    float delta_deg = theta + cross_deg;        /* 合成目标转向角, 符号可调 */
    if (delta_deg >  MAX_STEER_DEG) delta_deg =  MAX_STEER_DEG;
    if (delta_deg < -MAX_STEER_DEG) delta_deg = -MAX_STEER_DEG;
    float delta_rad = delta_deg * (float)M_PI / 180.0f;
    w = v * tanf(delta_rad) / WHEEL_BASE;
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 4. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = k230_angle;
    g_trace_posx   = k230_posx;
    g_trace_target = delta_deg;

    /* ---- 5. 麦轮解算 + 发送 ---- */
    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}

/* ================================================================
 * 入口 — 按 TRACE_USE_STANLEY 选择控制器
 * ================================================================ */
void Trace_LineFollow(void)
{
#if TRACE_USE_STANLEY
    Trace_LineFollow_Stanley();
#else
    Trace_LineFollow_PID();
#endif
}