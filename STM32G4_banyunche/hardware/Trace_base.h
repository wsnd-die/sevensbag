/**
 * @file  Trace_base.h
 * @brief 循迹底盘控制 — 串级 PID
 *        外环: 角度偏差 → 目标横轴位置
 *        内环: 位置误差 → 角速度 w
 */

#ifndef TRACE_BASE_H
#define TRACE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 外环: 角度 → 目标位置 ======================== */
#define ANGLE_KP      0.03f     /* 外环 P */
#define ANGLE_KI      0.0f     /* 外环 I */
#define ANGLE_KD      0.0f     /* 外环 D */
#define ANGLE_OUT_MAX  80.0f   /* 外环输出限幅 (像素) */
/* ======================== 内环: 位置 → 角速度 ======================== */
#define POS_KP        0.024f    /* 内环 P */
#define POS_KI        0.00f   /* 内环 I */
#define POS_KD        0.009f    /* 内环 D */
#define POS_INTEGRAL_MAX  3.0f /* 内环积分限幅 */

/* ======================== 共用 ======================== */
#define TRACE_BASE_SPEED   0.95f   /* 基础线速度 (m/s) */
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