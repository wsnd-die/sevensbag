/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制实现
 *        基于 K230 循迹传感器圆弧切角 → PID 角速度 → Mecanum_Calc 麦轮解算
 */


#include "Common_used.h"

/* ======================== 内部状态变量 ======================== */

static float g_trace_last_err  = 0.0f;   /* 上次角度误差 (rad)，用于微分 */
static float g_trace_integral  = 0.0f;   /* 误差积分项 */

/* ======================== 循线跟随主函数 ======================== */

/**
 * @brief  循线跟随 — 基于圆弧切角误差的 PID 控制
 *
 * @note   传感器数据解包:
 *         数据包格式: 0xA3 0xB3 [int16_lo] [int16_hi] 0xFF
 *         其中 int16 值为切角度，90 = 线在正中心（前进方向与圆弧切线重合）
 *
 * @note   控制原理:
 *         - 角度误差 = (raw_data - 90) [°] → 转弧度
 *         - PID 计算角速度 w = Kp·err + Ki·∫err + Kd·derr
 *         - v 为基础线速度，大弯自动减速
 *         - Mecanum_Calc(v, w) 逆运动学 → 四轮转速
 *
 *         轮子布局（俯视图）:
 *           前左(ω1) ─── 前右(ω2)
 *              │    ↑x(前)   │
 *              │             │
 *           后左(ω3) ─── 后右(ω4)
 *
 *         V+ω 模型逆解:
 *           ω1 = V − (a+b)·ω     ω2 = V + (a+b)·ω
 *           ω3 = V − (a+b)·ω     ω4 = V + (a+b)·ω
 *         当 w>0 (逆时针转) → 左侧轮减速、右侧轮加速 → 车体左转趋近切线
 */
void Trace_LineFollow(void)
{
    uint8_t  raw_buf[2];
    int16_t  raw_angle;
    float    angle_err_deg;
    float    angle_err_rad;
    float    derivative;
    float    w, v;
    MecanumResult motor;

    /* ---- 1. 读取传感器数据 ---- */
    Read_Tracedata(raw_buf);

    if (!Read_TraceFlag()) {
        return;  /* 无新数据，保持当前电机状态不变 */
    }

    /* 清除标志，准备接收下一包 */
    K230_Clear();

    /* ---- 2. 解析 int16 角度值（小端序）---- */
    raw_angle = (int16_t)((uint16_t)raw_buf[0] | ((uint16_t)raw_buf[1] << 8));

    /* ---- 3. 计算角度误差 ---- */
    /* 偏离中心 90 的量 = 角度偏差（°），正值=线偏右/切线偏右转 */
    angle_err_deg = (float)(raw_angle - (int16_t)TRACE_CENTER_VALUE);

    /* 转换为弧度 */
    angle_err_rad = angle_err_deg * (3.14159265f / 180.0f);

    /* ---- 4. PID 控制器计算角速度 w ---- */

    /* 积分累加 + 限幅 */
    g_trace_integral += angle_err_rad;
    if (g_trace_integral >  TRACE_INTEGRAL_MAX) {
        g_trace_integral =  TRACE_INTEGRAL_MAX;
    }
    if (g_trace_integral < -TRACE_INTEGRAL_MAX) {
        g_trace_integral = -TRACE_INTEGRAL_MAX;
    }

    /* 微分项 */
    derivative = angle_err_rad - g_trace_last_err;
    g_trace_last_err = angle_err_rad;

    /* PID 合成角速度 */
    w = TRACE_KP * angle_err_rad
      + TRACE_KI * g_trace_integral
      + TRACE_KD * derivative;

    /* 角速度输出限幅 */
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 5. 线速度 — 弯道自适应减速 ---- */
    v = TRACE_BASE_SPEED;
    {
        float abs_err = fabsf(angle_err_deg);
        if (abs_err > 30.0f) {
            v = TRACE_BASE_SPEED * 0.5f;   /* 急弯: 半速 */
        } else if (abs_err > 15.0f) {
            v = TRACE_BASE_SPEED * 0.75f;  /* 中弯: 3/4 速 */
        }
        /* 小偏差: 全速前进 */
    }

    /* ---- 6. 麦轮逆运动学解算 (V + ω 单轴模型) ---- */
    motor = Mecanum_Calc(v, w);

    /* ---- 7. 发送电机指令 ---- */
    Send_commandmotor(&motor);
}
