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
#include "trace_tune.h"
#include "pid.h"
#include <stdint.h>

/* ======================== 循迹参数 (可调) ======================== */
#define GRAY_BASE_SPEED   0.2f     /* 基础线速度 m/s */
#define GRAY_BASE_SPEED   0.2f     /* 基础线速度 m/s (默认; 在线调参 #vmax 会覆盖) */
#define GRAY_W_MAX        1.4f     /* 最大角速度 rad/s */
#define GRAY_KP           0.2f     /* PID P */
#define GRAY_KI           0.0f     /* PID I */
#define GRAY_KD           0.0f     /* PID D */
#define GRAY_ERR_MAX      47.55f    /* 偏差限幅 (与通道权重范围一致) */
#define GRAY_W_ALPHA      0.3f     /* 角速度一阶低通滤波系数 (0~1, 越小越平滑/响应越慢) */

/* 8 通道权重: 通道0(最左) = -52.5, 通道7(最右) = +52.5 (同参考) */
#define GRAY_CH_WEIGHTS { -19.0f, -15.0f, -13.0f, -1.0f, 1.0f, 13.0f, 15.0f, 19.0f }

/* ======================== 数据结构 ======================== */
typedef struct {
    Grayscale_Sensor_t sensor;   /* 灰度传感器状态 */
    float              last_error; /* 上一帧真实偏差 (丢线时保持用; 勿用 pid.error[1], 那是负值) */
    float              w_smooth;   /* 角速度一阶低通滤波后的输出值 */
    uint8_t            inited;
} GrayTrace_t;


/* ======================== API ======================== */
void GrayTrace_Init(GrayTrace_t *gt);            /* 初始化 GPIO + PID */
void GrayTrace_Update(GrayTrace_t *gt);          /* 读灰度 → 偏差 → PID → 麦轮输出 */
float GrayTrace_Calc_Error(GrayTrace_t *gt);     /* 只算重心偏差 (供调试) */

#endif /* GRAYTRACE_H */
