/**
 * @file    navigation.c
 * @brief   导航: 位置环(独立) + 角度串级(角度环→角速度环)
 */
#include "navigation.h"
#include <math.h>
#include <string.h>

#define CLAMP(v,lo,hi)  ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#define DEG2RAD(d)      ((d)*0.01745329252f)
#define RAD2DEG(r)      ((r)*57.2957795131f)
#define DT              0.01f

/* 默认参数 */
#define CFG_MAX_V       1.0f
#define CFG_MAX_W       3.0f
#define CFG_MAX_W_DEG   180.0f
#define CFG_ACCEL       0.5f
#define CFG_DECEL       0.4f
#define CFG_POS_TOL     10.0f
#define CFG_YAW_TOL     2.0f

static float norm_deg(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* ================================================================
 *  位置环 (独立)
 *   输入: 当前位置 + 锁定 yaw
 *   输出: nav->cmd_vx, nav->cmd_vy
 * ================================================================ */
void PosLoop_Update(NavController *nav,
                    float cur_x, float cur_y,
                    float locked_yaw_deg)
{
    float dx = nav->target_x - cur_x;
    float dy = nav->target_y - cur_y;
    nav->dist  = sqrtf(dx * dx + dy * dy);

    /* ---- 梯形速度前馈 ---- */
    float v_des = sqrtf(2.0f * nav->decel * nav->dist * 0.001f);
    v_des = CLAMP(v_des, 0.0f, nav->max_v);
    float dv = v_des - nav->cur_speed;
    nav->cur_speed += CLAMP(dv, -nav->accel * DT, nav->accel * DT);

    /* ---- 全局 → 机器人 (锁定 yaw) ---- */
    float yr = DEG2RAD(locked_yaw_deg);
    float cos_y = cosf(yr), sin_y = sinf(yr);
    float ex =  dx * cos_y + dy * sin_y;
    float ey = -dx * sin_y + dy * cos_y;

    /* ---- 前馈方向 + PID 修正 ---- */
    float ux = (nav->dist > 0.0f) ? (ex / nav->dist) : 0.0f;
    float uy = (nav->dist > 0.0f) ? (ey / nav->dist) : 0.0f;

    nav->cmd_vx = CLAMP(ux * nav->cur_speed * nav->pos_kp
                        + PID_calc(&nav->pid_px, 0.0f, ex * 0.001f),
                        -nav->max_v, nav->max_v);
    nav->cmd_vy = CLAMP(uy * nav->cur_speed * nav->pos_kp
                        + PID_calc(&nav->pid_py, 0.0f, ey * 0.001f),
                        -nav->max_v, nav->max_v);
}

/* ================================================================
 *  角度串级: 角度环 → 角速度环
 *   输入: 当前 yaw + 角速度
 *   输出: nav->cmd_w, nav->target_w, nav->yaw_err
 * ================================================================ */
void YawLoop_Update(NavController *nav,
                    float cur_yaw, float cur_w)
{
    /* ---- 陀螺仪滤波 ---- */
    /* 低通滤波 */
    nav->gyro_filt = nav->gyro_alpha * cur_w
                   + (1.0f - nav->gyro_alpha) * nav->gyro_filt;

    /* 死区: 小于阈值当 0 */
    float w_filt = nav->gyro_filt;
    if (fabsf(w_filt) < nav->gyro_deadband) w_filt = 0.0f;

    /* 缩放 */
    w_filt *= nav->gyro_scale;

    /* ---- 角度环 ---- */
    nav->yaw_err = norm_deg(nav->target_yaw - cur_yaw);

    /* 中层: yaw → target_w (deg/s) */
    nav->target_w = PID_calc(&nav->pid_angle, 0.0f, nav->yaw_err);
    nav->target_w = CLAMP(nav->target_w, -CFG_MAX_W_DEG, CFG_MAX_W_DEG);

    /* 内层: w → cmd_w (rad/s), 用滤波后的角速度做反馈 */
    nav->cmd_w = PID_calc(&nav->pid_w, w_filt, nav->target_w);
    nav->cmd_w = CLAMP(nav->cmd_w, -nav->max_w, nav->max_w);
}

/* ================================================================
 *  整体
 * ================================================================ */

void Nav_Init(NavController *nav)
{
    memset(nav, 0, sizeof(*nav));
    nav->state  = NAV_IDLE;
    nav->max_v  = CFG_MAX_V;
    nav->max_w  = CFG_MAX_W;
    nav->accel  = CFG_ACCEL;
    nav->decel  = CFG_DECEL;
    nav->pos_tol = CFG_POS_TOL;
    nav->yaw_tol = CFG_YAW_TOL;
    nav->pos_kp = 0.0f;

    /* 位置环 PID */
    const fp32 pk[3] = { 1.0f, 0.0f, 0.0f };
    PID_init(&nav->pid_px, PID_POSITION, pk, CFG_MAX_V, 0.3f);
    PID_init(&nav->pid_py, PID_POSITION, pk, CFG_MAX_V, 0.3f);

    /* 角度环 */
    const fp32 ak[3] = { 4.0f, 0.0f, 0.001f };
    PID_init(&nav->pid_angle, PID_POSITION, ak, CFG_MAX_W_DEG, 30.0f);

    /* 角速度环 */
    const fp32 wk[3] = { 0.028f, 0.01f, 0.001f };
    PID_init(&nav->pid_w, PID_POSITION, wk, CFG_MAX_W, 0.5f);

    /* 陀螺仪滤波 */
    nav->gyro_alpha    = 0.14f;   /* 低通: 越小越平滑, 0.1=强滤波 */
    nav->gyro_deadband = 0.15f;    /* 死区: |w|<2 deg/s → 当 0 处理 */
    nav->gyro_scale    = 0.05f;    /* 缩放: 乘系数缩窄gyro抖动 */
    nav->gyro_filt     = 0.0f;
}

void Nav_SetTarget(NavController *nav, float x, float y, float yaw)
{
    nav->target_x   = x;
    nav->target_y   = y;
    nav->target_yaw = yaw;
    nav->state      = NAV_MOVING;
    nav->cur_speed  = 0.0f;
    PID_clear(&nav->pid_px);
    PID_clear(&nav->pid_py);
    PID_clear(&nav->pid_angle);
    PID_clear(&nav->pid_w);
}

void Nav_Update(NavController *nav,
                float cur_x, float cur_y,
                float cur_yaw, float cur_w)
{
    if (nav->state != NAV_MOVING) {
        nav->cmd_vx = 0.0f; nav->cmd_vy = 0.0f; nav->cmd_w = 0.0f;
        return;
    }

    /* ---- 位置环 (独立) ---- */
    PosLoop_Update(nav, cur_x, cur_y, nav->target_yaw);

    /* ---- 角度串级 ---- */
    YawLoop_Update(nav, cur_yaw, cur_w);

    /* ---- 到达判断 ---- */
    if (nav->dist < nav->pos_tol &&
        fabsf(nav->yaw_err) < nav->yaw_tol) {
        nav->state  = NAV_ARRIVED;
        nav->cmd_vx = 0.0f; nav->cmd_vy = 0.0f; nav->cmd_w = 0.0f;
    }
}

bool Nav_Arrived(const NavController *nav)
{
    return (nav->state == NAV_ARRIVED);
}