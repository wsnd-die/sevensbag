#ifndef __MECANUM_MOVE_H
#define __MECANUM_MOVE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MECANUM_PI          3.14159265358979323846f
#define MECANUM_DEG_TO_RAD  (MECANUM_PI / 180.0f)
#define MECANUM_RAD_TO_DEG  (180.0f / MECANUM_PI)

/*
 * 电机地址：
 * 1号：前右 FR
 * 2号：后左 RL
 * 3号：前左 FL
 * 4号：后右 RR
 */
#define MECANUM_ADDR_FR  1U
#define MECANUM_ADDR_RL  2U
#define MECANUM_ADDR_FL  3U
#define MECANUM_ADDR_RR  4U
/**
 * @brief 麦轮底盘参数
 */
typedef struct {
    /* 麦轮半径，单位：m */
    float wheel_radius_m;

    /* 底盘中心到轮轴的前后距离，单位：m */
    float half_length_m;

    /* 底盘中心到轮子的左右距离，单位：m */
    float half_width_m;

    /*
     * 减速比：电机转数 / 轮子转数
     * 直连填写1.0
     */
    float gear_ratio;

    /* 电机转一圈对应的位置指令脉冲数 */
    uint32_t pulse_per_rev;

    /* 最大电机转速，单位：RPM */
    uint16_t max_motor_rpm;

    /* 最短运动时间，单位：s */
    float min_move_time_s;

    /* EMM_V5位置模式加速度参数 */
    uint8_t acceleration;

    /*
     * 轮子向车体前方滚动时，对应驱动器的dir值。
     *
     * forward_dir[1]：前右
     * forward_dir[2]：后左
     * forward_dir[3]：前左
     * forward_dir[4]：后右
     */
    uint8_t forward_dir[5];

} MecanumConfig_t;

/**
 * @brief 单个电机的运动命令
 */
typedef struct {
    /* 带符号的轮缘距离，单位：m */
    float wheel_distance_m;

    /* 带符号的电机转数 */
    float motor_rev;

    /* 带符号的电机转速，单位：RPM */
    float motor_rpm_signed;

    /* 发送给驱动器的速度 */
    uint16_t vel;

    /* 发送给驱动器的位置脉冲数 */
    uint32_t clk;

    /* 发送给驱动器的方向 */
    uint8_t dir;

} MecanumMotorCmd_t;

/**
 * @brief 一次底盘运动的解算结果
 */
typedef struct {
    /*
     * motor[1]：前右
     * motor[2]：后左
     * motor[3]：前左
     * motor[4]：后右
     */
    MecanumMotorCmd_t motor[5];

    /* 四轮理论共同完成时间，单位：s */
    float duration_s;

    /* 是否存在有效运动 */
    bool has_motion;

} MecanumMove_t;




extern MecanumConfig_t g_mecanum_config;


/**
 * @brief 按车体坐标解算组合运动
 *
 * @param config      底盘参数
 * @param body_dx_m   车体前进累计量，向前为正，单位：m
 * @param body_dy_m   车体横移累计量，向左为正，单位：m
 * @param dtheta_rad  旋转角度，逆时针为正，单位：rad
 * @param move        输出解算结果
 */
bool Mecanum_CalculateMove(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    MecanumMove_t *move
);

/**
 * @brief 按世界坐标解算组合运动
 *
 * 使车辆在平移的同时旋转，并在理想情况下最终到达指定的
 * 世界坐标位移和目标角度。
 *
 * @param config           底盘参数
 * @param world_dx_m       世界X方向位移，单位：m
 * @param world_dy_m       世界Y方向位移，单位：m
 * @param start_theta_rad  动作开始时车头的世界航向，单位：rad
 * @param dtheta_rad       本次需要旋转的角度，逆时针为正
 * @param move             输出解算结果
 */

bool Mecanum_CalculateWorldMove(
    const MecanumConfig_t *config,
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    MecanumMove_t *move
);

/**
 * @brief 发送四轮相对位置命令并同步启动
 */
bool Mecanum_ExecuteMove(
    const MecanumConfig_t *config,
    const MecanumMove_t *move
);

/**
 * @brief 立即停止四个电机
 */
void Mecanum_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MECANUM_MOVE_H */
