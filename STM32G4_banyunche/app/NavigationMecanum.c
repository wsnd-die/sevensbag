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

    {      0.00f,       0.00f,   0.0f  * MECANUM_DEG_TO_RAD },		/* 点0: 原点，面朝前 */
		{0.308f,    0.656f,  0.0  * MECANUM_DEG_TO_RAD					},

    {    0.308f,     0.656f,  83.0f  * MECANUM_DEG_TO_RAD },  /* 点1 */
    {    0.269f,     0.818f,  83.0f * MECANUM_DEG_TO_RAD },  /* 点2 */
    {    0.616f,    1.16f,  0.0f * MECANUM_DEG_TO_RAD },  /* 点3 */
    {    1.086f,    1.301f,   50.0f * MECANUM_DEG_TO_RAD },  /* 点4 */
    {   1.46f,    1.28f, -50.0f * MECANUM_DEG_TO_RAD },  /* 点5 */
    {  1.85f,    1.09f, -50.0f * MECANUM_DEG_TO_RAD },  /* 点6 */
    {    1.026f,     0.494f,   0.0f  * MECANUM_DEG_TO_RAD },  /*a 点7 */
		{    1.066f,     0.474f,   0.0f  * MECANUM_DEG_TO_RAD },/*a*/
    {    1.066f,     -0.35f,   0.0f  * MECANUM_DEG_TO_RAD },  /*c 点8 */
		{    0.810f,     -0.35f,  0.0f  * MECANUM_DEG_TO_RAD },  /*c 点8 */
    {    0.810f,     0.296f,   0.0f  * MECANUM_DEG_TO_RAD },  /* b点9 */
		{    0.850f,     -0.5f,   0.0f  * MECANUM_DEG_TO_RAD },  /* d点9 */
		{   0.91, -0.96f,  0.0f  * MECANUM_DEG_TO_RAD },  /* e点10 */
		
    {   0.75f,    -0.347f,   0.0f  * MECANUM_DEG_TO_RAD },  /* d点10 */
    {    0.793f,    -0.841f,   0.0f  * MECANUM_DEG_TO_RAD },  /*e 点11 */
};

/* 实际使用的路径点数量 */
uint8_t  g_waypoint_count = 15;

  
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

/**
 * @brief 导航到目标世界坐标（速度模式 + 编码器反馈，单段平顺）
 */
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

/**
 * @brief 依次执行所有路径点
 */
bool Nav_RunWaypoints(void)
{
    for (uint8_t i = 0; i < g_waypoint_count; i++) {
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
