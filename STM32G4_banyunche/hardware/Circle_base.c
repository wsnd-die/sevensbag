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
        float pos_x,posy;
    g_circle_dir = dir;  /* 保存方向供外部打印 */

    /* ---- 2. 方向 → 速度映射 ---- */
    switch (dir) {
    case 'O':
        g_circle_vx = 0.0f;
        g_circle_vy = 0.0f;
        break;

    case 'N':
        g_circle_vx = -CIRCLE_SPEED_V * g_circle_speed;
        g_circle_vy =  0.0f;

        break;

    case 'S':
        g_circle_vx = CIRCLE_SPEED_V * g_circle_speed;
        g_circle_vy =  0.0f;

        break;

    case 'W':
        g_circle_vx =  0.0f;
        g_circle_vy = -CIRCLE_SPEED_VY * g_circle_speed;


        break;

    case 'E':
        g_circle_vx =  0.0f;
        g_circle_vy =  CIRCLE_SPEED_VY * g_circle_speed;

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