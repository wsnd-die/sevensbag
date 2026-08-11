/**
 * @file    angle_ctrl.c
 * @brief   纯角度控制: 角度环 (角速度环已去掉)
 *          仅维持目标角度，不做位置导航
 */
#include "Common_used.h"

#define CLAMP(v,lo,hi)  ((v)<(lo)?(lo):((v)>(hi)?(hi):(v)))
#define DEG2RAD         0.0174532925f   /* π/180 */

/* 默认参数 */
#define CFG_MAX_W       3.0f
#define CFG_MAX_W_DEG   180.0f
#define CFG_YAW_TOL     5.0f

static float norm_deg(float d)
{
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* ================================================================
 *  角度环 (角速度环已去掉)
 *   输入: 当前 yaw
 *   输出: ac->cmd_w(rad/s), ac->yaw_err
 * ================================================================ */
void AngleLoop_Update(AngleCtrl *ac,
                    float cur_yaw, float cur_w)
{
    (void)cur_w;   /* 角速度环已去掉, 陀螺仪角速度不再参与控制 */

    /* ---- 角度环: yaw_err(deg) → 角速度指令 ---- */
    ac->yaw_err = norm_deg(ac->target_yaw - cur_yaw);

    /* 角度环输出为角速度目标 (deg/s) */
    float target_w = PID_calc(&ac->pid_angle, 0.0f, ac->yaw_err);
    target_w = CLAMP(target_w, -CFG_MAX_W_DEG, CFG_MAX_W_DEG);

    /* deg/s → rad/s, 直接作为电机指令 */
    ac->cmd_w = target_w * DEG2RAD;
    ac->cmd_w = CLAMP(ac->cmd_w, -ac->max_w, ac->max_w);
}

/* ================================================================
 *  整体
 * ================================================================ */

void Angle_Init(AngleCtrl *ac)
{
    memset(ac, 0, sizeof(*ac));
    ac->state  = ANGLE_IDLE;
    ac->max_w  = CFG_MAX_W;
    ac->yaw_tol = CFG_YAW_TOL;

    /* 角度环 */
    const fp32 ak[3] = { 0.9f, 0.0f, 0.001f };
    PID_init(&ac->pid_angle, PID_POSITION, ak, CFG_MAX_W_DEG, 30.0f);
}

void Angle_SetTarget(AngleCtrl *ac, float yaw)
{
    ac->target_yaw = yaw;
    ac->state      = ANGLE_MOVING;
    PID_clear(&ac->pid_angle);
}

void Angle_UpdateTarget(AngleCtrl *ac, float yaw)
{
    ac->target_yaw = yaw;
    ac->state      = ANGLE_MOVING;
    /* 不重置 PID，适合连续追踪 */
}

void Angle_Update(AngleCtrl *ac,
                float cur_yaw, float cur_w)
{
    if (ac->state != ANGLE_MOVING) {
        ac->cmd_w = 0.0f;
        return;
    }

    /* ---- 角度环 ---- */
    AngleLoop_Update(ac, cur_yaw, cur_w);

    /* ---- 到达判断 ---- */
    if (fabsf(ac->yaw_err) < ac->yaw_tol) {
        ac->state = ANGLE_ARRIVED;
        ac->cmd_w = 0.0f;
    }
}

bool Angle_Arrived(const AngleCtrl *ac)
{
    return (ac->state == ANGLE_ARRIVED);
}
