#ifndef __ACKERMANN_H
#define __ACKERMANN_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

    // ************************ 车辆物理参数配置 ************************
#define WHEELBASE        0.147f   // 轴距 m
#define TRACK_WIDTH      0.187f   // 轮距 m
#define WHEEL_RADIUS     0.0425f  // 车轮半径 m
#define SPEED_COEFF      224.058f // 速度换算系数
#define STOP_THRESHOLD   1e-3f    // 静止判断阈值
#define MAX_STEER_ANGLE  0.27f    // 最大转向角 rad
#define LOW_SPEED_LIMIT  0.20f    // 低速阈值 m/s
#define LOW_SPEED_GAIN   2.1f     // 低速放大倍数
#define MIN_MOTOR_SPEED  35.0f    // 电机最小启动转速
#define SERVO_MIN        32.0f    // 舵机最小占空比
#define SERVO_MAX        148.0f   // 舵机最大占空比

    // ************************ 输出结果结构体 ************************
    typedef struct {
        uint16_t left_speed;   // 左轮转速 (0-65535)
        uint16_t right_speed;  // 右轮转速 (0-65535)
        uint8_t  left_dir;     // 左轮方向 1=正转 0=反转
        uint8_t  right_dir;    // 右轮方向 1=正转 0=反转
        float    servo_pwm;    // 舵机控制占空比 (32-148)
    } AckermanResult;

    // ************************ 核心函数声明 ************************
    AckermanResult Ackerman_Calc(float v, float w);

#ifdef __cplusplus
}
#endif

#endif