/**
 * @file  Trace_base.h
 * @brief 循迹底盘控制模块
 *        通过 K230 循迹传感器获取圆弧切角偏移，
 *        PID 解算角速度 w，配合 Mecanum_Calc 驱动麦轮底盘循线行驶
 * @version 0.1
 * @date   2026-08-03
 */

#ifndef __TRACE_BASE_H
#define __TRACE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 循线控制参数 ======================== */
#define TRACE_BASE_SPEED       0.25f    /* 基础线速度 (m/s)，前进为正 */
#define TRACE_KP               2.0f     /* 角度误差比例系数 */
#define TRACE_KI               0.0f    /* 角度误差积分系数 */
#define TRACE_KD               0.15f    /* 角度误差微分系数 */
#define TRACE_CENTER_VALUE     0       /* 传感器中心值 (线在正中间) */
#define TRACE_INTEGRAL_MAX     5.0f     /* 积分限幅 */
#define TRACE_W_MAX            2.5f     /* 最大角速度 (rad/s) */

/* ======================== 函数声明 ======================== */

/**
 * @brief  循线跟随主函数（基于圆弧切角误差的 PID 控制）
 * @note   调用周期: 建议 10~20ms (与传感器数据更新率匹配)
 *
 *         控制流程:
 *         1. 通过 Read_Tracedata / Read_TraceFlag 获取传感器偏移
 *         2. 角度误差 = 原始数据 - 90 → 转换为弧度
 *         3. PID 控制器计算角速度 w
 *         4. 弯道根据误差大小自动减速
 *         5. Mecanum_Calc(v, w) 解算四轮转速
 *         6. Send_commandmotor 发送电机指令
 */
void Trace_LineFollow(void);

extern float g_trace_v;
extern float g_trace_w;
extern float g_trace_angle;
extern float g_trace_posx;
extern float g_trace_posx;

#ifdef __cplusplus
}
#endif

#endif /* __TRACE_BASE_H */
