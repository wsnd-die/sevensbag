/**
 * @file  Trace_base.h
 * @brief 循迹底盘控制 — 8路灰度传感器 + 麦轮
 *        灰度误差 err → 角速度 w → Mecanum_Calc → 电机
 *
 * 2026-08 重构(修复循迹左右大幅晃动):
 *   - 误差改为 8 路加权质心(连续、单调自洽), 替代离散查表
 *   - 控制改为 低通 + PD, 消除原 bang-bang(开关式) 振荡
 *   - 丢线(全白/全黑)时保持上次误差一段时间, 避免假对中导致满打
 */

#ifndef TRACE_BASE_H
#define TRACE_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== 8路灰度循迹参数 ======================== */
/* 误差: 8路质心, 量程约 ±7 (详见 Trace_base.c 的 Trace_Gray_Centroid) */
#define GRAY_P_KP      0.22f   /* 误差 → 角速度 w 比例增益
                                *  err=±1 → w≈0.12(直线上微修)
                                *  err=±7 → w≈0.84(出线时强修正) */
#define GRAY_P_KD      0.80f   /* 误差每周期变化 → w 微分阻尼(按 ~10ms 周期标定)
                                *  误差增大时提前补转, 误差回落时抑制过冲 */
#define GRAY_ERR_LP    0.25f   /* 误差低通系数 (0~1, 越小越平滑但响应越慢) */
#define GRAY_ERR_MAX   7.0f    /* 误差量程 (质心最大输出), 用于速度自适应 */
#define GRAY_LOST_HOLD_CYCLES 25U  /* 丢线(全白/全黑)时保持上次误差的周期数(~250ms) */

/* ======================== 共用 ======================== */
#define TRACE_BASE_SPEED   0.65f   /* 基础线速度 (m/s) */
#define TRACE_W_MAX        0.95f   /* 最大角速度 (rad/s) */

/* 若实车转向与期望相反, 将 GRAY_P_KP 取负即可 (原 GRAY_ERR_TO_W_GAIN 同逻辑) */

/* ======================== 8路灰度传感器 ========================
 * 引脚(本车接线): PA4 = GRAY_CLK (CLK 输出), PB7 = GRAY_DAT (DAT 输入, 上拉)
 * 数据语义: 返回原始字节, bit=0 表示该传感器压在线上
 * ======================== */
void Trace_Gray_Init(void);              /* 初始化灰度 GPIO (幂等) */
void Trace_Gray_ReadData(uint8_t *data); /* 读取 8 路灰度原始字节 */

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
