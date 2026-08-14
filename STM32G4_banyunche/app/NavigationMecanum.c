#include "Common_used.h"

/* ============================================================
 * 全局变量
 * ============================================================ */

/* 当前自身位姿（世界坐标系），上电默认原点 */
World_Dir_t Self_Dir = {0.0f, 0.0f, 0.0f};

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
World_Dir_t g_waypoints[NAV_WAYPOINT_MAX] = {

    /* ---- 示例路径（可根据实际修改）---- */


    {0.308f,    0.656f,  0.0f  * MECANUM_DEG_TO_RAD },//奖杯二维码点

    {0.0f,      0.1f,      -90.0f * MECANUM_DEG_TO_RAD },//物料循线点

    {    -0.866f,     -0.436f,   0.0f  * MECANUM_DEG_TO_RAD },  /*a点*/
    {    -0.22f,     -0.20f,   0.0f  * MECANUM_DEG_TO_RAD },/*b点*/
    {    0.3f,     -0.71f,   0.0f  * MECANUM_DEG_TO_RAD },  /*c点 */
    {    -0.23f,     -0.10f,  0.0f  * MECANUM_DEG_TO_RAD },  /*d点*/
    {    0.10f,     -0.52f,   0.0f  * MECANUM_DEG_TO_RAD },  /* e点 */

    {  -0.353f,    0.201f, 0.0f * MECANUM_DEG_TO_RAD },  /* 奖杯二维码点*/

    {   0.08f,    0.0f, 90.0f * MECANUM_DEG_TO_RAD },  /* 奖杯寻线点 */

    {    0.425f,     1.03f,  0.0f * MECANUM_DEG_TO_RAD },  /* 亚军点*/
    {    0.06f,    0.27f,  0.0f * MECANUM_DEG_TO_RAD },  /* 冠军点 */
    {    0.06f,    0.27f,   0.0f * MECANUM_DEG_TO_RAD },  /* 季军点 */


    {-0.513f,-0.0,0},//回家点
    {0,-0.244f,0},
    {-1.164f,0,0},

};

/* 实际使用的路径点数量 */
uint8_t  g_waypoint_count = 13;


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

/* ============================================================
 * 公开函数
 * ============================================================ */

bool Nav_GoToWorld(float target_x, float target_y, float target_yaw)
{
    float world_dx, world_dy, dtheta, dist;
    uint32_t timeout_ms;
    bool success;

    /* ---- 1. 计算世界坐标系下的差值 ---- */
    world_dx = target_x - Self_Dir.x;
    world_dy = target_y - Self_Dir.y;

    /* 角度差取最短路径 */
    dtheta = Nav_NormalizeAngle(target_yaw - Self_Dir.yaw);

    /* ---- 2. 到位判断 ---- */
    dist = sqrtf(world_dx * world_dx + world_dy * world_dy);
    if (dist < 0.005f && fabsf(dtheta) < 0.01f) {
        return true;
    }

    /* ---- 3. 计算超时（预估耗时 × 3，保底 5s）---- */
    {
        float est_s = dist / 0.05f;  /* 0.05 m/s 保守估计最慢速度 */
        if (est_s < 5.0f) est_s = 5.0f;
        timeout_ms = (uint32_t)(est_s * 1000.0f);  /* ×2 安全余量 */
    }

    /* ---- 4. 速度模式 + 编码器反馈，单段走完 ---- */
    success = Mecanum_WorldMoveWithEncoder(
        &g_mecanum_config,
        world_dx, world_dy,
        Self_Dir.yaw,
        dtheta,
        1.0f,          /* 全速 */
        g_mecanum_config.acceleration,           /* acc */
        timeout_ms
    );

    if (!success) {
        return false;
    }

    /* ---- 5. 更新自身位姿 ---- */
    Self_Dir.x   = target_x;
    Self_Dir.y   = target_y;
    Self_Dir.yaw = Nav_NormalizeAngle(Self_Dir.yaw + dtheta);

    return true;
}
/*==============================================
 * 车体坐标运动 (Body-frame)
 *   平移: 麦轮解算 (世界坐标, 不含旋转)
 *   旋转: 交由 FC_TASK 角度环 (串级PID) 执行
 * ============================================================ */


/**
 * @brief 车体坐标运动
 *        forward_m/left_m: 车体坐标平移量 (m), 麦轮单独执行
 *        rotate_rad:       平移到位后用 AngleCtrl 旋转 (rad, CCW+)
 */
bool Nav_MoveBody(float forward_m, float left_m, float rotate_rad)
{
    float dist;
    bool  success;

    dist = sqrtf(forward_m * forward_m + left_m * left_m);
    if (dist < 0.005f && fabsf(rotate_rad) < 0.01f) {
        return true;
    }

    {
        float target_deg = rotate_rad * (180.0f / M_PI);
        float err;
        uint16_t guard = 0;

        g_angle_target_yaw = target_deg;
        g_angle_ctrl_enable = 1;              /* 打开 FC_TASK 角度环 */

        while (guard++ < 800) {               /* 超时约8s, 防角度环异常卡死 */
            err = target_deg - siyuan_yaw * RAD_TO_DEG;
            while (err >  180.0f) err -= 360.0f;
            while (err < -180.0f) err += 360.0f;
            if (fabsf(err) <= 4.0f) break;
            osDelay(10);
        }

        g_angle_ctrl_enable = 0;              /* 转完关闭, 避免与后面平移抢电机 */
        osDelay(20);                          /* 等 FC_TASK 停止输出 */
        Self_Dir.yaw = Nav_NormalizeAngle(rotate_rad);
    }


    /* ---- 3. 平移段: 车体坐标 → 麦轮 (dtheta=0, 不旋转) ---- */
    if (dist >= 0.005f) {
        float est_s = dist / 0.05f;
        if (est_s < 3.0f) est_s = 3.0f;
        uint32_t timeout_ms = (uint32_t)(est_s * 1000.0f);

        success = Mecanum_MoveWithEncoder(
            &g_mecanum_config,
            forward_m, left_m, 0.0f,  /* dtheta=0, 纯平移 */
            1.0f, 80U, timeout_ms
        );
        if (!success) return false;

        /* 更新世界位姿 (仅平移) */
        float cos_y = cosf(Self_Dir.yaw), sin_y = sinf(Self_Dir.yaw);
        Self_Dir.x += forward_m * cos_y - left_m * sin_y;
        Self_Dir.y += forward_m * sin_y + left_m * cos_y;
    }



    return true;
}

/** @brief 前进/后退 (车体坐标) */
bool Nav_MoveForward(float distance_m)
{
    return Nav_MoveBody(distance_m, 0.0f, 0.0f);
}

/** @brief 左移/右移 (车体坐标) */
bool Nav_MoveLeft(float distance_m)
{
    return Nav_MoveBody(0.0f, distance_m, 0.0f);
}

/** @brief 原地旋转 */
bool Nav_Rotate(float angle_rad)
{
    return Nav_MoveBody(0.0f, 0.0f, angle_rad);
}



bool Nav_FeDuanPoint() {
    static uint8_t PontIntex=0;

    if (!Nav_MoveBody(g_waypoints[PontIntex].x,
                          g_waypoints[PontIntex].y,
                          g_waypoints[PontIntex++].yaw)) {
        return false;
                          }
    if (PontIntex==13)
    {
        Nav_MoveBody(g_waypoints[PontIntex].x,
                          g_waypoints[PontIntex].y,
                          g_waypoints[PontIntex++].yaw);

        Nav_MoveBody(g_waypoints[PontIntex].x,
                          g_waypoints[PontIntex].y,
                          g_waypoints[PontIntex++].yaw);

    }
    return true;
}


/**
 * @brief 依次执行所有路径点
 */
bool Nav_RunWaypoints(void)
{
    for (uint8_t i = 0; i < g_waypoint_count; i++) {
        if (!Nav_MoveBody(g_waypoints[i].x,
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













