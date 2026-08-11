/**
 * @file  Circle_base.c
 * @brief 找圆跟随控制实现
 *        K230 圆检测方向 → Mecanum_Calc_Full 全向移动 → 电机指令
 */

#include "Common_used.h"

/* ======================== 全局状态 ======================== */

float g_circle_vx = 0.0f;
float g_circle_vy = 0.0f;
char  g_circle_dir = '?';
float g_circle_speed = 1.0f;  /* 速度系数: 1.0=快(左侧), 0.4=慢(右侧) */

/* ======================== 找圆跟随主函数 ======================== */

void Circle_Follow(void)
{
    char dir;
    MecanumResult motor;

    /* ---- 1. 获取圆方向 ---- */
    K230_GetCircleDir(&dir);

    g_circle_dir = dir;  /* 保存方向供外部打印 */

    /* ---- 2. 方向 → 速度映射 ---- */
    switch (dir) {
    case 'O':
        //g_circle_vx = 0.0f;
        //g_circle_vy = 0.0f;
        break;

    case 'N':
        /* 圆心偏上: 前进 */
            Nav_MoveForward(0.02f);
        break;

    case 'S':
        /* 圆心偏下: 后退 */
            Nav_MoveForward(-0.02f);
        break;

    case 'W':
        /* 圆心偏右: 右移 */
            Nav_MoveLeft(-0.02f);
        break;

    case 'E':
        /* 圆心偏左: 左移 */
            Nav_MoveLeft(0.02f);
        break;

    default:
        /* 未知方向: 停止 */

        break;
    }

    /* ---- 3. 全向移动解算 (vx, vy, w=0) ---- */
    //motor = Mecanum_Calc_Full(g_circle_vx, g_circle_vy, 0.0f);

    /* ---- 4. 发送电机指令 ---- */
   // Send_commandmotor(&motor);
}