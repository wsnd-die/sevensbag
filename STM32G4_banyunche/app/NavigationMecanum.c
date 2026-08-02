#include "NavigationMecanum.h"
#include "cmsis_os.h"
#include <math.h>

/* ============================================================
 * 全局变量
 * ============================================================ */

/* 当前自身位姿（世界坐标系），上电默认原点 */
World_Dir Self_Dir = {0.0f, 0.0f, 0.0f};
   
/*
 * 路径点数组（世界坐标系）
 *
 * 字段: { x(m), y(m), yaw(rad) }
 *
 * 角度可使用 MECANUM_DEG_TO_RAD 辅助:
 *   N 度 → N * MECANUM_DEG_TO_RAD
 *
 * 修改此数组内容和你需要的目标点，
 * 同时更新 g_waypoint_count 为实际点数。
 */
World_Dir g_waypoints[NAV_WAYPOINT_MAX] = {

    /* ---- 示例路径（可根据实际修改）---- */

    {      0.00f,       0.00f,   0.0f  * MECANUM_DEG_TO_RAD },  /* 点0: 原点，面朝前 */

    {    0.305f,     0.63f,  0.0f  * MECANUM_DEG_TO_RAD },  /* 点1 */
    {    0.205f,     0.63f,  0.0f * MECANUM_DEG_TO_RAD },  /* 点2 */
    {    0.205f,    0.00f,  0.0f * MECANUM_DEG_TO_RAD },  /* 点3 */
    {    0.0f,    0.0f,   0.0f * MECANUM_DEG_TO_RAD },  /* 点4 */
    {   1187.62f,    1537.93f, -19.79f * MECANUM_DEG_TO_RAD },  /* 点5 */
    {   1581.93f,    1342.86f, -44.30f * MECANUM_DEG_TO_RAD },  /* 点6 */
    {    787.90f,     787.96f,   0.0f  * MECANUM_DEG_TO_RAD },  /* 点7 */
    {    556.89f,     568.52f,   0.0f  * MECANUM_DEG_TO_RAD },  /* 点8 */
    {    795.93f,     -24.56f,   0.0f  * MECANUM_DEG_TO_RAD },  /* 点9 */
    {    553.58f,    -134.45f,   0.0f  * MECANUM_DEG_TO_RAD },  /* 点10 */
    {    589.58f,    -621.70f,   0.0f  * MECANUM_DEG_TO_RAD },  /* 点11 */
};

/* 实际使用的路径点数量 */
int g_waypoint_count = 5;


/* ============================================================
 * 内部辅助函数 / 配置
 * ============================================================ */

/**
 * @brief 将角度归一化到 [-PI, PI]
 */
static float Nav_NormalizeAngle(float angle)
{
    while (angle > MECANUM_PI) {
        angle -= 2.0f * MECANUM_PI;
    }
    while (angle < -MECANUM_PI) {
        angle += 2.0f * MECANUM_PI;
    }
    return angle;
}

/* 梯形速度斜坡默认配置 */
static const MecanumRamp_t g_nav_ramp = {
    .fast_ratio = 0.85f,    /* 85% 路程全速 */
    .slow_speed = 0.35f,    /* 剩余 15% 路程以 35% 速度精停 */
};


/* ============================================================
 * 公开函数
 * ============================================================ */

/**
 * @brief 导航到目标世界坐标（梯形速度斜坡）
 */
bool Nav_GoToWorld(float target_x, float target_y, float target_yaw)
{
    MecanumMove_t move_fast = {0};
    MecanumMove_t move_slow = {0};
    float world_dx, world_dy, dtheta, dist;
    float total_duration_s;
    bool success;

    /* ---- 1. 计算世界坐标系下的差值 ---- */
    world_dx = target_x - Self_Dir.x;
    world_dy = target_y - Self_Dir.y;

    /* 角度差取最短路径 */
    dtheta = Nav_NormalizeAngle(target_yaw - Self_Dir.yaw);

    /* ---- 2. 到位判断 ---- */
    dist = sqrtf(world_dx * world_dx + world_dy * world_dy);
    if (dist < 0.005f && fabsf(dtheta) < 0.01f) {
        /* 已在目标范围内，无需运动 */
        return true;
    }

    /* ---- 3. 梯形速度解算（快速段 + 慢速段）---- */
    success = Mecanum_CalcRampedMoves(
        &g_mecanum_config,
        world_dx,
        world_dy,
        Self_Dir.yaw,          /* 动作开始时的航向 */
        dtheta,                 /* 本次需要旋转的角度 */
        &g_nav_ramp,
        &move_fast,
        &move_slow,
        &total_duration_s
    );

    if (!success) {
        return false;
    }

    /* ---- 4. Phase 1: 快速接近段 ---- */
    if (move_fast.has_motion) {
        success = Mecanum_ExecuteMove(&g_mecanum_config, &move_fast);
        if (!success) {
            return false;
        }
        uint32_t delay_ms = (uint32_t)(move_fast.duration_s * 2000.0f);
        if (delay_ms > 0U) {
            osDelay(delay_ms);
        }
    }

    /* ---- 5. Phase 2: 慢速精停段 ---- */
    if (move_slow.has_motion) {
        success = Mecanum_ExecuteMove(&g_mecanum_config, &move_slow);
        if (!success) {
            return false;
        }
        uint32_t delay_ms = (uint32_t)(move_slow.duration_s * 2000.0f);
        if (delay_ms > 0U) {
            osDelay(delay_ms);
        }
    }

    /* ---- 6. 更新自身位姿 ---- */
    Self_Dir.x   = target_x;
    Self_Dir.y   = target_y;
    Self_Dir.yaw = Nav_NormalizeAngle(Self_Dir.yaw + dtheta);

    return true;
}

/**
 * @brief 依次执行所有路径点
 */
bool Nav_RunWaypoints(void)
{
    for (int i = 0; i < g_waypoint_count; i++) {
        if (!Nav_GoToWorld(g_waypoints[i].x,
                           g_waypoints[i].y,
                           g_waypoints[i].yaw)) {
            return false;
        }
    }
    return true;
}


/* ============================================================
 * 测试函数（保留）
 * ============================================================ */

void Chassis_WorldMoveTest(void)
{
    MecanumMove_t move;
    bool success;

    success = Mecanum_CalculateWorldMove(
        &g_mecanum_config,

        0.00f,                         /* 世界X：前进50 cm */
       0.0f,                          /* 世界Y：向右50 cm */
        0.0f * MECANUM_DEG_TO_RAD,    /* 开始航向0° */
        90.0f * MECANUM_DEG_TO_RAD,   /* 逆时针旋转50° */

        &move
    );

    if (success) {
        Mecanum_ExecuteMove(
            &g_mecanum_config,
            &move
        );
    }
}
