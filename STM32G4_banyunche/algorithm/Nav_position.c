//
// Created by 35037 on 2026/8/12.
//
#include "Nav_position.h"

/* 当前里程计位姿（世界坐标 m / rad），上电原点 (0,0,0) */
World_Dir_t World_position = {0.0f, 0.0f, 0.0f};

/**
 * @brief 增量式编码器里程计
 * @return 当前世界位姿（同时更新全局 World_position）
 */
World_Dir_t World_position_get(void)
{
    static uint8_t     first = 1;
    static EncoderData prev;
    EncoderData enc;
    float d_fwd, d_side, fwd_mm, side_mm;
    float yaw = siyuan_yaw;             /* IMU 实测航向 rad */

    if (!Mecanum_Read_AllPositions(&enc, 20)) return World_position;
    if (first) { first = 0; prev = enc; return World_position; }

    /* 麦轮正解 */
    d_fwd  = (float)(enc.fl - prev.fl + enc.fr - prev.fr +
                     enc.rl - prev.rl + enc.rr - prev.rr) / 4.0f;
    d_side = (float)(-enc.fl + prev.fl + enc.fr - prev.fr +
                     enc.rl - prev.rl - enc.rr + prev.rr) / 4.0f;
    prev = enc;

    /* 脉冲→mm */
    Odometry_Apply_Calib(d_fwd, d_side, &fwd_mm, &side_mm);
    World_position.x += (fwd_mm * cosf(yaw) - side_mm * sinf(yaw)) / 1000.0f;
    World_position.y += (fwd_mm * sinf(yaw) + side_mm * cosf(yaw)) / 1000.0f;
    World_position.yaw = yaw;

    return World_position;
}