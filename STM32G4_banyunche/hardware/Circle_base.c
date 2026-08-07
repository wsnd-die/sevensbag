/**
 * @file  Circle_base.c
 * @brief 找圆跟随控制实现
 *        K230 圆检测方向 → Mecanum_Calc_Full 全向移动 → 电机指令
 */

#include "Common_used.h"

/* ======================== 全局状态 ======================== */

float g_circle_vx = 0.0f;
float g_circle_vy = 0.0f;

/* ======================== 找圆跟随主函数 ======================== */

void Circle_Follow(void)
{
    char dir;
    MecanumResult motor;

    /* ---- 1. 获取圆方向 ---- */
    if (!K230_GetCircleDir(&dir)) {
        return;  /* 无新数据，保持当前状态 */
    }

    /* ---- 2. 方向 → 速度映射 ---- */
    switch (dir) {
    case 'O':
        Place('0');
        break;

    case 'N':
        /* 圆心偏上: 前进 */
        g_circle_vx =  CIRCLE_SPEED_V;
        g_circle_vy =  0.0f;
        break;

    case 'S':
        /* 圆心偏下: 后退 */
        g_circle_vx = -CIRCLE_SPEED_V;
        g_circle_vy =  0.0f;
        break;

    case 'W':
        /* 圆心偏右: 右移 */
        g_circle_vx =  0.0f;
        g_circle_vy = -CIRCLE_SPEED_VY;
        break;

    case 'E':
        /* 圆心偏左: 左移 */
        g_circle_vx =  0.0f;
        g_circle_vy =  CIRCLE_SPEED_VY;
        break;

    default:
        /* 未知方向: 停止 */
        g_circle_vx = 0.0f;
        g_circle_vy = 0.0f;
        break;
    }

    /* ---- 3. 全向移动解算 (vx, vy, w=0) ---- */
    motor = Mecanum_Calc_Full(g_circle_vx, g_circle_vy, 0.0f);

    /* ---- 4. 发送电机指令 ---- */
    Send_commandmotor(&motor);
}