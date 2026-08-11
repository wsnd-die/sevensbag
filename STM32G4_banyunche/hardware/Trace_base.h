/**
 * @file  Trace_base.h
 * @brief 循迹底盘控制 — PID / Stanley 两种控制器分开实现
 *        TRACE_USE_STANLEY 0 = PID, 1 = Stanley
 *        PID: 外环 角度→目标位置, 内环 位置→角速度 w
 */

#ifndef TRACE_BASE_H
#define TRACE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 控制器选择 ======================== */
#define TRACE_USE_STANLEY   0   /* 0=PID, 1=Stanley */

/* ======================== PID 外环: 位置 → 目标角度 ======================== */
#define ANGLE_KP      0.04f     /* 外环 P (作用于位置环) */
#define ANGLE_KI      0.0f     /* 外环 I */
#define ANGLE_KD      0.0f     /* 外环 D */
#define ANGLE_OUT_MAX  100.0f   /* 外环输出限幅 (目标角度, deg) */
/* ======================== PID 内环: 角度 → 角速度 ======================== */
#define POS_KP        0.15f    /* 内环 P (作用于角度环) */
#define POS_KI        0.0f   /* 内环 I */
#define POS_KD        0.0f    /* 内环 D */
#define POS_INTEGRAL_MAX  10.0f /* 内环积分限幅 */

/* ======================== Stanley 参数 ======================== */
#define STANLEY_K           0.5f   /* 横向误差增益 */
#define WHEEL_BASE          0.25f  /* 轴距 (m) */
#define MAX_STEER_DEG       30.0f  /* 最大转向角 (度) */
#define MIN_SPEED           0.1f   /* 防止除零最低速度 (m/s) */

/* ======================== 共用 ======================== */
#define TRACE_BASE_SPEED   0.69f   /* 基础线速度 (m/s) */
#define TRACE_W_MAX        0.95f  /* 最大角速度 (rad/s) */

/* ======================== 函数声明 ======================== */
void Trace_LineFollow(void);          /* 按 TRACE_USE_STANLEY 选择调用哪个 */
void Trace_LineFollow_PID(void);      /* PID 控制器 */
void Trace_LineFollow_Stanley(void);  /* Stanley 控制器 */

extern float g_trace_v;
extern float g_trace_w;
extern float g_trace_angle;
extern float g_trace_posx;
extern float g_trace_target;

#ifdef __cplusplus
}
#endif

#endif /* TRACE_BASE_H */