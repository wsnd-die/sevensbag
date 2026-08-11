#include "Common_used.h"

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


    {0.656f,    0.308f,  90.0f  * MECANUM_DEG_TO_RAD },//奖杯二维码点

    {0.10f,      0.0f,      90.0f * MECANUM_DEG_TO_RAD },//奖杯循线点

      {    0.47f,     1.2f,  0.0f * MECANUM_DEG_TO_RAD },  /* 亚军点*/
      {    0.0f,    0.27f,  0.0f * MECANUM_DEG_TO_RAD },  /* 冠军点 */
      {    0.0f,    0.27f,   0.0f * MECANUM_DEG_TO_RAD },  /* 季军点 */

      {   0.12f,    -0.156f, -90.0f * MECANUM_DEG_TO_RAD },  /* 物料寻线点 */

      {  0.41f,    0.03f, 90.0f * MECANUM_DEG_TO_RAD },  /* 物料二维码点 */

      {    0.7f,     0.0f,   0.0f  * MECANUM_DEG_TO_RAD },  /*a点*/
    {    -0.29f,     -0.20f,   0.0f  * MECANUM_DEG_TO_RAD },/*b点*/
      {    0.25f,     -0.61f,   0.0f  * MECANUM_DEG_TO_RAD },  /*c点 */
    {    -0.26f,     -0.10f,  0.0f  * MECANUM_DEG_TO_RAD },  /*d点*/
      {    0.074f,     -0.48f,   0.0f  * MECANUM_DEG_TO_RAD },  /* e点 */

          {-0.60f,0.81f,0},//回家点

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
        80U,           /* acc */
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
 *   旋转: AngleCtrl 串级 PID, 平移到位后单独执行
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

    /* ---- 1. 到位判断 ---- */
    dist = sqrtf(forward_m * forward_m + left_m * left_m);
    if (dist < 0.005f && fabsf(rotate_rad) < 0.01f) {
        return true;
    }

    /* ---- 2. 旋转段: rotate_rad 为世界绝对角度(rad), AngleCtrl 闭环 ---- */

        float target_deg = rotate_rad*(180.0f/M_PI) ;
        float error_deg   = target_deg - siyuan_yaw*RAD_TO_DEG;
        while (error_deg >  180.0f) error_deg -= 360.0f;
        while (error_deg < -180.0f) error_deg += 360.0f;

        /* 已经到位则跳过 */
        if (fabsf(error_deg) >= 4.0f) {
            AngleCtrl ac;
            Angle_Init(&ac);
            Angle_SetTarget(&ac, target_deg);

            while (!Angle_Arrived(&ac)) {
                Angle_Update(&ac,siyuan_yaw*RAD_TO_DEG ,  -imu660ra_gyro_transition(imu660ra_gyro_x));
                MecanumResult motor = Mecanum_Calc(0.0f, -ac.cmd_w);
                Send_commandmotor(&motor);
                osDelay(30);
            }
            Mecanum_StopAll();
        }

        Self_Dir.yaw = Nav_NormalizeAngle(rotate_rad);


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



    float error_deg_2   = target_deg - siyuan_yaw*RAD_TO_DEG;
    while (error_deg_2 >  180.0f) error_deg_2 -= 360.0f;
    while (error_deg_2 < -180.0f) error_deg_2 += 360.0f;

    /* 已经到位则跳过 */
    if (fabsf(error_deg_2) >= 4.0f) {
        AngleCtrl ac_2;
        Angle_Init(&ac_2);
        Angle_SetTarget(&ac_2, error_deg_2);

        while (!Angle_Arrived(&ac_2)) {
            Angle_Update(&ac_2,siyuan_yaw*RAD_TO_DEG ,  -imu660ra_gyro_transition(imu660ra_gyro_x));
            MecanumResult motor = Mecanum_Calc(0.0f, -ac_2.cmd_w);
            Send_commandmotor(&motor);
            osDelay(30);
        }
        Mecanum_StopAll();
    }

    Self_Dir.yaw = Nav_NormalizeAngle(rotate_rad);


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













