/**
 * @file    navigation.h
 * @brief   导航: 位置环(独立) + 角度串级(角度环→角速度环)
 *          固定 yaw 全向行驶
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
    float target_x;       /* mm */
    float target_y;       /* mm */
    float target_yaw;     /* deg, 锁定朝向 */

    /* ---- 位置环 (独立) ---- */
    pid_type_def pid_px;  /* 位置误差 → 速度 m/s */
    pid_type_def pid_py;
    float pos_kp;         /* 梯形前馈系数, 1.0=满前馈 */

    /* ---- 角度环 (中层) ---- */
    pid_type_def pid_angle;  /* yaw_err(deg) → target_w(deg/s) */

    /* ---- 角速度环 (内层) ---- */
    pid_type_def pid_w;   /* w_err(deg/s) → cmd_w(rad/s) */

    /* 限幅 */
    float max_v;          /* m/s */
    float max_w;          /* rad/s */
    float decel;          /* m/s² */
    float accel;          /* m/s² */
    float cur_speed;

    /* 阈值 */
    float pos_tol;        /* mm */
    float yaw_tol;        /* deg */

    /* ---- 陀螺仪滤波 ---- */
    float gyro_alpha;      /* 低通系数: 0~1, 越小滤波越强 */
    float gyro_deadband;   /* 死区 deg/s, 低于此值置零 */
    float gyro_scale;      /* 缩放系数: 0~1 */
    float gyro_filt;       /* 滤波后的角速度 deg/s */

    /* 输出 */
    float cmd_vx, cmd_vy, cmd_w;

    /* 诊断 */
    float dist, yaw_err, target_w;
} NavController;

/* 位置环 */
void PosLoop_Update(NavController *nav,
                    float cur_x, float cur_y,
                    float locked_yaw_deg);

/* 角度环 + 角速度环 */
void YawLoop_Update(NavController *nav,
                    float cur_yaw, float cur_w);

/* 整体 */
void  Nav_Init     (NavController *nav);
void  Nav_SetTarget(NavController *nav, float x, float y, float yaw);
void  Nav_Update   (NavController *nav,
                    float cur_x, float cur_y,
                    float cur_yaw, float cur_w);
bool  Nav_Arrived  (const NavController *nav);

#endif