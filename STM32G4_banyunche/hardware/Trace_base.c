/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制 — 串级 PID
 *        外环: 位置 → PID → 目标角度
 *        内环: 角度 → PID → 角速度 w → Mecanum_Calc → 电机
 */

#include "Common_used.h"
#include "HWT101_iic.h"
#include "stdlib.h"
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
        PID_init(&g_pid_angle, PID_POSITION, ak, TRACE_W_MAX, 10.0f);       /* 内环输出 w, 限幅 TRACE_W_MAX */
        PID_init(&g_pid_pos,   PID_POSITION, pk, ANGLE_OUT_MAX, POS_INTEGRAL_MAX); /* 外环输出角度, 限幅 ANGLE_OUT_MAX */
        g_pid_inited = 1;
    }

    /* ---- 3. 外环: 位置 → 目标角度 (期望=0, 反馈=位置) ---- */
    /* PID_calc(pid, ref, set): ref=反馈(实际位置), set=目标(0) */
    float target_angle = PID_calc(&g_pid_pos, k230_posx, 0.0f);
    float curve_comp = -k230_angle * TRACE_CURVE_OUTER_GAIN;
    if (curve_comp >  TRACE_CURVE_OUTER_MAX_DEG) curve_comp =  TRACE_CURVE_OUTER_MAX_DEG;
    if (curve_comp < -TRACE_CURVE_OUTER_MAX_DEG) curve_comp = -TRACE_CURVE_OUTER_MAX_DEG;
    target_angle += curve_comp;
    if (target_angle >  ANGLE_OUT_MAX) target_angle =  ANGLE_OUT_MAX;
    if (target_angle < -ANGLE_OUT_MAX) target_angle = -ANGLE_OUT_MAX;

    /* ---- 4. 内环: 角度 → 角速度 w (期望=target_angle, 反馈=角度) ---- */
    /* PID_calc(pid, ref, set): ref=反馈(实际角度), set=目标(target_angle) */
    w = PID_calc(&g_pid_angle, k230_angle, target_angle);
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 5. 速度自适应 ---- */
    v = TRACE_BASE_SPEED;
    {
        float abs_angle = fabsf(k230_angle);
        float abs_posx = fabsf(k230_posx);
        float curve_level = abs_angle / 35.0f;
        float pos_level = abs_posx / 6.0f;
        float slow_level = curve_level > pos_level ? curve_level : pos_level;
        static float v_smooth = TRACE_BASE_SPEED;

        if (slow_level > 1.0f) slow_level = 1.0f;
        v = TRACE_BASE_SPEED * (1.0f - 0.38f * slow_level);
        v_smooth += (v < v_smooth ? 0.55f : 0.18f) * (v - v_smooth);
        v = v_smooth;
    }

    /* ---- 6. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = k230_angle;
    g_trace_posx   = k230_posx;
    g_trace_target = target_angle;

    /* ---- 7. 麦轮解算 + 发送 ---- */
    motor = Mecanum_Calc(v, w);
    /* ---- 7.1 实时打印巡线状态 ---- */
    printf("[TRACE] v=%.3f w=%.3f angle=%.2f posx=%.3f target=%.2f comp=%.2f | FL=%u FR=%u RL=%u RR=%u | hwt_yaw=%.2f hwt_gz=%.2f\r\n",
           g_trace_v, g_trace_w, g_trace_angle, g_trace_posx, g_trace_target, curve_comp,
           motor.fl_speed, motor.fr_speed, motor.rl_speed, motor.rr_speed,
           HWT101_GetZeroYaw(), g_hwt101_gyro_z);
    Send_commandmotor(&motor);
}

void Trace_LineTask(void) {

    while (1) {
        Trace_LineFollow();
        osDelay(20);
    }

}
