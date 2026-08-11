/**
 * @file    angle_ctrl.h
 * @brief   纯角度控制: 角度环 (角速度环已去掉)
 *          仅维持目标角度，不做位置角度控制
 */
#ifndef ANGLE_CTRL_H
#define ANGLE_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

typedef enum {
    ANGLE_IDLE    = 0,
    ANGLE_MOVING  = 1,
    ANGLE_ARRIVED = 2
} AngleState;

typedef struct {
    AngleState state;

    /* 目标 */
    float target_yaw;     /* deg, 目标朝向 */

    /* ---- 角度环 (唯一环, 角速度环已去掉) ---- */
    pid_type_def pid_angle;  /* yaw_err(deg) → cmd_w(rad/s) */

    /* 限幅 */
    float max_w;          /* rad/s */

    /* 阈值 */
    float yaw_tol;        /* deg */

    /* 输出 */
    float cmd_w;

    /* 诊断 */
    float yaw_err;
} AngleCtrl;

/* 角度环 (角速度环已去掉, cur_w 不再参与控制) */
void AngleLoop_Update(AngleCtrl *ac,
                    float cur_yaw, float cur_w);

/* 整体 */
void  Angle_Init        (AngleCtrl *ac);
void  Angle_SetTarget   (AngleCtrl *ac, float yaw);
void  Angle_UpdateTarget(AngleCtrl *ac, float yaw);  /* 仅更新目标，不重置PID */
void  Angle_Update      (AngleCtrl *ac,
                       float cur_yaw, float cur_w);
bool  Angle_Arrived     (const AngleCtrl *ac);

#endif
