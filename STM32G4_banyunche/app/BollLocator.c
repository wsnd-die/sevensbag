//
// Created by ZHUHUI on 2026/8/6.
//
// BollLocator — 基于 TB_position 定位器的位置闭环控制器
//
// 闭环链路:
//   TB_position (世界坐标 mm, 串口2定位器)
//   → 位置误差计算 (世界系 dx/dy)
//   → 世界→车体坐标变换 (锁定 imu_yaw)
//   → 梯形速度规划 (缓启/缓停)
//   → Mecanum_Calc_Full(cmd_vx, cmd_vy, 0)
//   → Send_commandmotor() → Emm_V5_Vel_Control() → FDCAN1
//

#include "BollLocator.h"

/* ============================================================
 * 辅助宏
 * ============================================================ */
#define CLAMP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define DEG2RAD(d)        ((d) * 0.01745329252f)
#define RAD2DEG(r)        ((r) * 57.2957795131f)
#define SQ(x)             ((x) * (x))

/* ============================================================
 * 初始化
 * ============================================================ */
void BL_Init(BollLocator *bl)
{
    if (!bl) return;
    memset(bl, 0, sizeof(*bl));
    bl->max_speed = BL_DEFAULT_MAX_SPEED;
    bl->accel     = BL_DEFAULT_ACCEL;
    bl->decel     = BL_DEFAULT_DECEL;
    bl->pos_tol   = BL_DEFAULT_POS_TOL;
    bl->active    = false;
    bl->cur_speed = 0.0f;
}

/* ============================================================
 * 设置目标 — 激活位置环
 * ============================================================ */
void BL_SetTarget(BollLocator *bl, float x_mm, float y_mm)
{
    if (!bl) return;
    bl->target_x  = x_mm;
    bl->target_y  = y_mm;
    bl->active    = true;
    bl->cur_speed = 0.0f;   /* 每次新目标都从 0 开始缓启 */
}

/* ============================================================
 * 停止
 * ============================================================ */
void BL_Stop(BollLocator *bl)
{
    if (!bl) return;
    bl->active    = false;
    bl->cur_speed = 0.0f;
    bl->cmd_vx    = 0.0f;
    bl->cmd_vy    = 0.0f;

    /* 输出停止 */
    MecanumResult motor = Mecanum_Calc_Full(0.0f, 0.0f, 0.0f);
    Send_commandmotor(&motor);
}

/* ============================================================
 * 到达判断
 * ============================================================ */
bool BL_Arrived(const BollLocator *bl)
{
    if (!bl) return true;
    return (!bl->active) || (bl->dist < bl->pos_tol);
}

/* ============================================================
 * 核心: 位置环更新 (每 10ms 调用一次)
 *
 * 算法:
 *   1. 读 TB_position 作为当前世界坐标
 *   2. 计算世界系误差 dx/dy 和剩余距离 dist
 *   3. 梯形速度规划:
 *      - 加速段: cur_speed += accel * DT, 不超过 max_speed
 *      - 减速段: 当 dist < stop_dist 时, cur_speed 递减
 *      - stop_dist = cur_speed² / (2 * decel)  (以当前速度恰好减速到0所需距离)
 *   4. 世界误差 → 车体速度 (锁定 imu_yaw)
 *   5. 逆运动学 + 发送电机命令
 * ============================================================ */
void BL_Update(BollLocator *bl)
{
    if (!bl || !bl->active) return;

    /* ---- 1. 读取当前位置 (世界坐标 mm) ---- */
    float cur_x = TB_position.xdata;
    float cur_y = TB_position.ydata;

    /* ---- 2. 世界系误差 ---- */
    float dx = bl->target_x - cur_x;
    float dy = bl->target_y - cur_y;
    bl->err_x = dx;
    bl->err_y = dy;
    bl->dist  = sqrtf(dx * dx + dy * dy);

    /* ---- 3. 到达判断 ---- */
    if (bl->dist < bl->pos_tol) {
        bl->active   = false;
        bl->cur_speed = 0.0f;
        bl->cmd_vx   = 0.0f;
        bl->cmd_vy   = 0.0f;
        MecanumResult motor = Mecanum_Calc_Full(0.0f, 0.0f, 0.0f);
        Send_commandmotor(&motor);
        return;
    }

    /* ---- 4. 梯形速度规划 (缓启 / 缓停) ---- */

    /*
     * 减速距离: 以当前速度 decel 到 0 所需最小距离 (m)
     * 注意 dist 是 mm, cur_speed 是 m/s
     */
    float dist_m  = bl->dist * 0.001f;
    float stop_dist = (bl->cur_speed * bl->cur_speed) / (2.0f * bl->decel);

    /* 目标速度 */
    float target_speed;

    if (dist_m <= stop_dist) {
        /*
         * 减速段: 按剩余距离计算允许的最大速度
         * v_allow = sqrt(2 * decel * dist)
         * 保证能以 decel 减速度恰好停在目标点
         */
        target_speed = sqrtf(2.0f * bl->decel * dist_m);
        /* 至少保留一个最小速度，避免过早降为 0 */
        if (target_speed < 0.02f) target_speed = 0.02f;
        target_speed = CLAMP(target_speed, 0.0f, bl->max_speed);
    } else {
        /* 巡航段: 加速到最大速度 */
        target_speed = bl->max_speed;
    }

    /* 缓启: 加速度限制 ramp */
    float dv = target_speed - bl->cur_speed;
    float max_dv = bl->accel * BL_DT;
    bl->cur_speed += CLAMP(dv, -max_dv, max_dv);

    /* 下限保护 */
    if (bl->cur_speed < 0.005f) bl->cur_speed = 0.005f;

    /* ---- 5. 世界误差 → 车体速度 (锁定当前 yaw) ---- */

    /* 读取当前偏航角 (deg) */
    float yaw_rad = DEG2RAD(imu_yaw);

    /*
     * 旋转矩阵 [cos  sin; -sin  cos] 将世界系误差转到车体系:
     *   body_x =  dx * cos + dy * sin   (车体前进方向)
     *   body_y = -dx * sin + dy * cos   (车体左移方向)
     */
    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);
    float body_dx =  dx * cos_y + dy * sin_y;
    float body_dy = -dx * sin_y + dy * cos_y;

    /* 归一化方向 */
    float dir_x = (bl->dist > 0.0f) ? (body_dx / bl->dist) : 0.0f;
    float dir_y = (bl->dist > 0.0f) ? (body_dy / bl->dist) : 0.0f;

    /* 输出速度 = 方向 × 当前速度 */
    bl->cmd_vx = dir_x * bl->cur_speed;
    bl->cmd_vy = dir_y * bl->cur_speed;

    /* ---- 6. 逆运动学 + 发送电机命令 ---- */
    MecanumResult motor = Mecanum_Calc_Full(bl->cmd_vx, bl->cmd_vy, 0.0f);
    Send_commandmotor(&motor);
}
