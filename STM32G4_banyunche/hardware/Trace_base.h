/**
 * @file  Trace_base.h
 * @brief 循迹底盘控制 — 串级 PID
 *        外环: 位置误差 → 目标角度
 *        内环: 角度偏差 → 角速度 w
 */

#ifndef TRACE_BASE_H
#define TRACE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 外环: 位置 → 目标角度 ======================== */
#define ANGLE_KP      0.03f     /* 外环 P (作用于位置环) */
#define ANGLE_KI      0.0f     /* 外环 I (作用于位置环) */
#define ANGLE_KD      0.0f     /* 外环 D (作用于位置环) */
#define ANGLE_OUT_MAX  100.0f   /* 外环输出限幅 (目标角度, deg) */
/* ======================== 内环: 角度 → 角速度 ======================== */
#define POS_KP        0.13f    /* 内环 P (作用于角度环) */
#define POS_KI        0.0f   /* 内环 I (作用于角度环) */
#define POS_KD        0.0f    /* 内环 D (作用于角度环) */
#define POS_INTEGRAL_MAX  10.0f /* 内环积分限幅 */

/* ======================== 共用 ======================== */
#define TRACE_BASE_SPEED   0.65f   /* 基础线速度 (m/s) */
#define TRACE_W_MAX        0.95f    /* 最大角速度 (rad/s) */

/* ======================== 函数声明 ======================== */

void Trace_LineFollow(void);
void Trace_LineTask(void);
extern float g_trace_v;
extern float g_trace_w;
extern float g_trace_angle;
extern float g_trace_posx;
extern float g_trace_target;

#ifdef __cplusplus
}
#endif

#endif /* TRACE_BASE_H */