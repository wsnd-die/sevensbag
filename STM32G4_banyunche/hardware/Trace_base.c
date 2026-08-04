/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制 — 串级 PID
 *        外环: 角度 → PID → 目标横轴位置
 *        内环: 位置 → PID → 角速度 w → Mecanum_Calc → 电机
 */

#include "Common_used.h"
#include "Trace_base.h"
#include "k230.h"
#include "pid.h"

/* ======================== 内部状态 ======================== */

static pid_type_def g_pid_angle;
static pid_type_def g_pid_pos;
static uint8_t g_pid_inited = 0;

float g_trace_v      = 0.0f;
float g_trace_w      = 0.0f;
float g_trace_angle  = 0.0f;
float g_trace_posx   = 0.0f;
float g_trace_target = 0.0f;

/* ======================== 循线跟随主函数 ======================== */

void Trace_LineFollow(void)
{
    float v, w;
    MecanumResult motor;

    /* ---- 1. 获取 K230 角度 + 位置 ---- */
    float k230_angle, k230_posx = 0.0f;
    if (!K230_GetLineAngle(&k230_angle)) {
        return;
    }
    int has_pos = K230_GetPosition(&k230_posx, NULL);

    /* ---- 2. PID 初始化 ---- */
    if (!g_pid_inited) {
        const fp32 ak[3] = { ANGLE_KP, ANGLE_KI, ANGLE_KD };
        const fp32 pk[3] = { POS_KP,   POS_KI,   POS_KD };
        PID_init(&g_pid_angle, PID_POSITION, ak, ANGLE_OUT_MAX, 10.0f);
        PID_init(&g_pid_pos,   PID_POSITION, pk, TRACE_W_MAX, POS_INTEGRAL_MAX);
        g_pid_inited = 1;
    }

    /* ---- 3. 外环: 角度 → 目标位置 ---- */
    float target_pos = PID_calc(&g_pid_angle, k230_angle, 0.0f);

    /* ---- 4. 内环: 位置 → 角速度 w ---- */
    if (has_pos) {
        w = PID_calc(&g_pid_pos, k230_posx, target_pos);
    } else {
        w = k230_angle * 0.03f;
    }
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
    g_trace_target = target_pos;

    /* ---- 7. 麦轮解算 + 发送 ---- */
    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}