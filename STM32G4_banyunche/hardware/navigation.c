/**
 * @file    navigation.c
 * @brief   纯角度控制: 角度环 → 角速度环 (串级 PID)
 *          仅维持目标角度，不做位置导航
 */
#include "Common_used.h"

#define CLAMP(v,lo,hi)  ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#define DT              0.01f

/* 默认参数 */
#define CFG_MAX_W       3.0f
#define CFG_MAX_W_DEG   180.0f
#define CFG_YAW_TOL     2.0f

static float norm_deg(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
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
    nav->max_w  = CFG_MAX_W;
    nav->yaw_tol = CFG_YAW_TOL;

    /* 角度环 */
    const fp32 ak[3] = { 4.0f, 0.0f, 0.001f };
    PID_init(&nav->pid_angle, PID_POSITION, ak, CFG_MAX_W_DEG, 30.0f);

    /* 角速度环 */
    const fp32 wk[3] = { 0.028f, 0.01f, 0.001f };
    PID_init(&nav->pid_w, PID_POSITION, wk, CFG_MAX_W, 0.5f);

    /* 陀螺仪滤波 */
    nav->gyro_alpha    = 0.14f;
    nav->gyro_deadband = 0.15f;
    nav->gyro_scale    = 0.05f;
    nav->gyro_filt     = 0.0f;
}

void Nav_SetTarget(NavController *nav, float yaw)
{
    nav->target_yaw = yaw;
    nav->state      = NAV_MOVING;
    PID_clear(&nav->pid_angle);
    PID_clear(&nav->pid_w);
}

void Nav_UpdateTarget(NavController *nav, float yaw)
{
    nav->target_yaw = yaw;
    nav->state      = NAV_MOVING;
    /* 不重置 PID，适合连续追踪 */
}

void Nav_Update(NavController *nav,
                float cur_yaw, float cur_w)
{
    if (nav->state != NAV_MOVING) {
        nav->cmd_w = 0.0f;
        return;
    }

    /* ---- 角度串级 ---- */
    YawLoop_Update(nav, cur_yaw, cur_w);

    /* ---- 到达判断 ---- */
    if (fabsf(nav->yaw_err) < nav->yaw_tol) {
        nav->state = NAV_ARRIVED;
        nav->cmd_w = 0.0f;
    }
}

bool Nav_Arrived(const NavController *nav)
{
    return (nav->state == NAV_ARRIVED);
}
