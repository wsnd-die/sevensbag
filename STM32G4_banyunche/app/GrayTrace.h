/**
 * @file GrayTrace.h
 * @brief 八通道灰度循迹控制器 — 重心偏差 + 项目 PID 库 + 麦轮输出
 *
 * 流程: 灰度 8 通道数字量 → 加权重心偏差 error → PID → 角速度 w
 *       → Mecanum_Calc(v, w) → 四麦轮电机
 *
 * 偏差约定 (与参考 follow_line 一致):
 *   error > 0 → 黑线在车偏右 (需左转修正)   [按实际走线方向调符号]
 */

#ifndef GRAYTRACE_H
#define GRAYTRACE_H

#include "grayscale.h"
#include "pid.h"
#include <stdint.h>

/* ======================== 循迹参数 (可调) ======================== */
#define GRAY_BASE_SPEED   0.5f     /* 基础线速度 m/s */
#define GRAY_W_MAX        0.8f     /* 最大角速度 rad/s */
#define GRAY_KP           0.8f     /* PID P */
#define GRAY_KI           0.0f     /* PID I */
#define GRAY_KD           0.3f     /* PID D */
#define GRAY_ERR_MAX      52.5f    /* 偏差限幅 (与通道权重范围一致) */

/* 8 通道权重: 通道0(最左) = -52.5, 通道7(最右) = +52.5 (同参考) */
#define GRAY_CH_WEIGHTS \
    { -52.5f, -37.5f, -22.5f, -7.5f, 7.5f, 22.5f, 37.5f, 52.5f }

/* ======================== 数据结构 ======================== */
typedef struct {
    Grayscale_Sensor_t sensor;   /* 灰度传感器状态 */
    pid_type_def       pid;      /* 项目 PID 库控制器 */
    uint8_t            inited;
} GrayTrace_t;

/* ======================== API ======================== */
void GrayTrace_Init(GrayTrace_t *gt);            /* 初始化 GPIO + PID */
void GrayTrace_Update(GrayTrace_t *gt);          /* 读灰度 → 偏差 → PID → 麦轮输出 */
float GrayTrace_Calc_Error(GrayTrace_t *gt);     /* 只算重心偏差 (供调试) */

#endif /* GRAYTRACE_H */
