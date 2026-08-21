#include "Common_used.h"
#include "HWT101_iic.h"

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


    {-0.32f,    0.75f,  0.0f  * MECANUM_DEG_TO_RAD },//奖杯二维码点
    {0.23f,      -0.45f,      90.0f * MECANUM_DEG_TO_RAD },//奖杯循线点
    {     0.0f,     0.0f,  0.0f * MECANUM_DEG_TO_RAD },  /* 亚军点：再向左平移 */
	  {    -0.58f,  -1.02f,  0.0f * MECANUM_DEG_TO_RAD },  /* 亚军点：先0度前进 */
    {    -0.15f,    -0.24f,  0.0f * MECANUM_DEG_TO_RAD },  /* 冠军点 */
    {    -0.15f,    -0.26f,   0.0f * MECANUM_DEG_TO_RAD },  /* 季军点 */

      {   1.01f,    -0.63f, 0.0f * MECANUM_DEG_TO_RAD },  /* 物料二维码点 */

      {  0.32f,    0.57f, -90.0f * MECANUM_DEG_TO_RAD },  /* 物料寻线点 */

      {    0.87f,     0.48f,   0.0f  * MECANUM_DEG_TO_RAD },  /*a点*/
    {    0.15f,     0.23f,   0.0f  * MECANUM_DEG_TO_RAD },/*b点*/
      {    -0.38f,     0.64f,   0.0f  * MECANUM_DEG_TO_RAD },  /*c点 */
    {    0.19f,     0.13f,  0.0f  * MECANUM_DEG_TO_RAD },  /*d点*/
      {    -0.17f,     0.52f,   0.0f  * MECANUM_DEG_TO_RAD },  /* e点 */

          {0.355f,     -0.90f,0.0f * MECANUM_DEG_TO_RAD},//回家点

};

/* 实际使用的路径点数量 */
uint8_t  g_waypoint_count = 14;

  
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
 *   旋转: 参考 IMU_FUCTION 的 HWT101 零点校准模式 (角度环)
 * ============================================================ */

/**
 * @brief 按 IMU_FUCTION 的 HWT101 零点校准模式控制转向:
 *        Angle_UpdateTarget + AngleLoop_Update/Angle_Update + Mecanum_Calc(0,cmd_w)
 * @param target_deg 目标世界航向 (deg)
 */
void Nav_TurnToDeg(float target_deg)
{
    AngleCtrl ac;
    Angle_Init(&ac);
    Angle_UpdateTarget(&ac, -target_deg);

    while (!Angle_Arrived(&ac)) {
        AngleLoop_Update(&ac, HWT101_GetZeroYaw(), g_hwt101_gyro_z);
        Angle_Update(&ac, HWT101_GetZeroYaw(), g_hwt101_gyro_z);
        MecanumResult motor = Mecanum_Calc(0.0f, ac.cmd_w);
        Send_commandmotor(&motor);
        osDelay(10);
    }
    Mecanum_StopAll();
}

/**
 * @brief 车体坐标运动
 *        forward_m/left_m: 车体坐标平移量 (m), 麦轮单独执行
 *        rotate_rad:       目标世界航向 (rad, CCW+), 转到该航向后平移
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

    /* ---- 2. 旋转段: 转到目标世界航向 (参考 IMU_FUCTION 角度控制模式) ---- */
    // Nav_TurnToDeg(rotate_rad * (180.0f / MECANUM_PI));
    // Self_Dir.yaw = Nav_NormalizeAngle(rotate_rad);

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

    /* ---- 4. 平移后校准航向 (参考 IMU_FUCTION 角度控制模式) ---- */
    Nav_TurnToDeg(rotate_rad * (180.0f / MECANUM_PI));
    Self_Dir.yaw = Nav_NormalizeAngle(rotate_rad);

    return true;
}

/** @brief 前进/后退 (车体坐标) */
bool Nav_MoveForward(float distance_m)
{
    return Nav_MoveBody(-distance_m, 0.0f, 0.0f);
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
    uint8_t idx = PontIntex;

    if (idx == 2U) {
        Nav_TurnToDeg(0.0f);
        Self_Dir.yaw = 0.0f;
    }

    /* 收集完5个物料、转盘转完45°后, 原地回正到出发时的正北朝向(0°), 再向a点移动 */
    if (idx == 8U) {
        Nav_TurnToDeg(0.0f);
        Self_Dir.yaw = 0.0f;
    }

    /* 奖杯二维码点(g_waypoints[0]): 拆成先走x再走y两步, 避免斜向移动漂移/偏航。
     * 第1步只走x, 第2步只走y; 每步结束 Nav_MoveBody 内部都会回正航向到 0°。 */
    if (idx == 0U) {
        if (!Nav_MoveBody(g_waypoints[0].x, 0.0f, 0.0f)) {          /* 第1步: 只走x */
            return false;
        }
        if (!Nav_MoveBody(0.0f, g_waypoints[0].y, g_waypoints[0].yaw)) {  /* 第2步: 只走y */
            return false;
        }
        PontIntex++;
        return true;
    }

    if (!Nav_MoveBody(g_waypoints[idx].x,
                          g_waypoints[idx].y,
                          g_waypoints[idx].yaw)) {
        return false;
                          }
    PontIntex++;
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













