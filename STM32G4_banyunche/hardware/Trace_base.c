/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制实现
 *        K230 循迹角度 → 内置 PID 角速度 → Mecanum_Calc 麦轮解算
 */

#include "Common_used.h"
#include "Trace_base.h"
#include "k230.h"
#include "pid.h"

/* ======================== 内部状态 ======================== */

static pid_type_def g_trace_pid;
static uint8_t g_trace_pid_inited = 0;

float g_trace_v = 0.0f;
float g_trace_w = 0.0f;
float g_trace_angle = 0.0f;
float g_trace_posx  = 0.0f;

/* ======================== 循线跟随主函数 ======================== */

void Trace_LineFollow(void)
{
    float v, w;
    MecanumResult motor;

    /* ---- 1. 从 K230 获取循迹角度 ---- */
    float k230_angle;
    if (!K230_GetLineAngle(&k230_angle)) {
        return;
    }

    /* ---- 2. PID 初始化（首次调用）---- */
    if (!g_trace_pid_inited) {
        const fp32 k[3] = { TRACE_KP, TRACE_KI, TRACE_KD };
        PID_init(&g_trace_pid, PID_POSITION, k,
                 TRACE_W_MAX, TRACE_INTEGRAL_MAX);
        g_trace_pid_inited = 1;
    }

    /* ---- 3. PID 计算角速度 w ---- */
    /* 目标: 角度=0 (线居中), 反馈: k230_angle */
    w = PID_calc(&g_trace_pid, k230_angle, 0.0f);
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 4. 线速度 — 自适应减速 ---- */
    v = TRACE_BASE_SPEED;
    {
        float abs_err = fabsf(k230_angle);
        if (abs_err > 30.0f) {
            v = TRACE_BASE_SPEED * 0.5f;
        } else if (abs_err > 15.0f) {
            v = TRACE_BASE_SPEED * 0.75f;
        }
    }

    /* ---- 5. 保存数据供主任务打印 ---- */
    g_trace_v     = v;
    g_trace_w     = w;
    g_trace_angle = k230_angle;
    K230_GetPosition(&g_trace_posx, NULL);

    /* ---- 6. 麦轮逆运动学解算 ---- */
    motor = Mecanum_Calc(v, w);

    /* ---- 7. 发送电机指令 ---- */
    Send_commandmotor(&motor);
}