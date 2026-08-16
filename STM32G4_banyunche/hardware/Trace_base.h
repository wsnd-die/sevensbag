/**
 * @file  Trace_base.h
 * @brief 循迹底盘控制 — 8路灰度传感器 + 麦轮
 *        灰度误差 err → 角速度 w → Mecanum_Calc → 电机
 */

#ifndef TRACE_BASE_H
#define TRACE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 8路灰度循迹参数 ======================== */
#define GRAY_ERR_TO_W_GAIN   0.9f    /* 误差 → 角速度 w (rad/s) */
#define GRAY_ERR_MAX         9.0f     /* 误差量程 (Track_Err 最大输出) */

/* ======================== 共用 ======================== */
#define TRACE_BASE_SPEED   0.5f   /* 基础线速度 (m/s) */
#define TRACE_W_MAX        0.95f   /* 最大角速度 (rad/s) */

/* 原 K230 串级 PID 参数(已废弃保留)：
 * 外环: 位置 → 目标角度 = ANGLE_KP/ANGLE_KI/ANGLE_KD, 限幅 ANGLE_OUT_MAX
 * 内环: 角度 → 角速度   = POS_KP/POS_KI/POS_KD, 限幅 TRACE_W_MAX, 积分限幅 POS_INTEGRAL_MAX
 * 弯道补偿: TRACE_CURVE_OUTER_GAIN / TRACE_CURVE_OUTER_MAX_DEG
 */

/* ======================== 8路灰度传感器 ========================
 * 引脚(本车接线): PA4 = GRAY_CLK (CLK 输出), PB7 = GRAY_DAT (DAT 输入, 上拉)
 * ======================== */
void Trace_Gray_Init(void);            /* 初始化灰度 GPIO (幂等) */
void Trace_Gray_ReadData(uint8_t *data); /* 读取 8 路灰度字节 */
float Trace_Gray_Error(void);          /* 灰度字节 → 误差 */

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
