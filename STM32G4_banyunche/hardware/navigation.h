/**
 * @file    navigation.h
 * @brief   纯角度控制: 角度环 → 角速度环 (串级 PID)
 *          仅维持目标角度，不做位置导航
 */
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

typedef enum {
    NAV_IDLE    = 0,
    NAV_MOVING  = 1,
    NAV_ARRIVED = 2
} NavState;

typedef struct {
    NavState state;

    /* 目标 */
    float target_yaw;     /* deg, 目标朝向 */

    /* ---- 角度环 (中层) ---- */
    pid_type_def pid_angle;  /* yaw_err(deg) → target_w(deg/s) */

    /* ---- 角速度环 (内层) ---- */
    pid_type_def pid_w;   /* w_err(deg/s) → cmd_w(rad/s) */

    /* 限幅 */
    float max_w;          /* rad/s */

    /* 阈值 */
    float yaw_tol;        /* deg */

    /* ---- 陀螺仪滤波 ---- */
    float gyro_alpha;      /* 低通系数: 0~1, 越小滤波越强 */
    float gyro_deadband;   /* 死区 deg/s, 低于此值置零 */
    float gyro_scale;      /* 缩放系数: 0~1 */
    float gyro_filt;       /* 滤波后的角速度 deg/s */

    /* 输出 */
    float cmd_w;

    /* 诊断 */
    float yaw_err, target_w;
} NavController;

/* 角度环 + 角速度环 */
void YawLoop_Update(NavController *nav,
                    float cur_yaw, float cur_w);

/* 整体 */
void  Nav_Init        (NavController *nav);
void  Nav_SetTarget   (NavController *nav, float yaw);
void  Nav_UpdateTarget(NavController *nav, float yaw);  /* 仅更新目标，不重置PID */
void  Nav_Update      (NavController *nav,
                       float cur_yaw, float cur_w);
bool  Nav_Arrived     (const NavController *nav);

#endif
