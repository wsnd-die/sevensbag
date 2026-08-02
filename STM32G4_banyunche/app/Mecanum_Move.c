#include "Mecanum_Move.h"
#include "stm32g4xx.h"
#include "emm_5v.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#define MECANUM_SYNC_ADDR  0U
#define MECANUM_EPSILON    1.0e-7f



  MecanumConfig_t g_mecanum_config = {
    .wheel_radius_m = 0.0375f,
    .half_length_m = 0.088f,
    .half_width_m = 0.0782f,
    .gear_ratio = 1.0f,
    .pulse_per_rev =3200 ,
    .max_motor_rpm = 130,
    .min_move_time_s = 0.1f,

    /* 驱动器逻辑方向: 1=正向, 0=反向 */
    .forward_dir[MECANUM_ADDR_FR] = 0U,
    .forward_dir[MECANUM_ADDR_RL] = 1U,
    .forward_dir[MECANUM_ADDR_FL] = 0U,
    .forward_dir[MECANUM_ADDR_RR] = 1U,

    /* 驱动器加速度: 脉冲/秒^2 */
    .acceleration = 80U
};

/* ---- 内部工具函数 ---- */

static float Mecanum_AbsFloat(float value)
{
    return value >= 0.0f ? value : -value;
}

static uint32_t Mecanum_RoundUint32(float value)
{
    return (uint32_t)(value + 0.5f);
}

static uint16_t Mecanum_RoundUint16(float value)
{
    return (uint16_t)(value + 0.5f);
}

/**
 * @brief 将轮子的逻辑方向转换为驱动器dir
 */
static uint8_t Mecanum_GetDriverDir(
    const MecanumConfig_t *config,
    uint8_t addr,
    float signed_distance
)
{
    uint8_t forward_dir;

    forward_dir =
        config->forward_dir[addr] != 0U ? 1U : 0U;

    if (signed_distance >= 0.0f) {
        return forward_dir;
    }

    return forward_dir == 0U ? 1U : 0U;
}

/* ============================================================
 * 核心解算实现（内部函数）
 * ============================================================ */

/**
 * @brief 车体坐标麦轮解算（带速度因子）
 *
 * @param speed_ratio  速度因子，1.0 = 全速，0.35 = 35% 最大 RPM
 */
static bool Mecanum_CalcMoveImpl(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    float speed_ratio,
    MecanumMove_t *move
)
{
    float distance[5] = {0.0f};
    float max_abs_motor_rev = 0.0f;
    float duration_s;
    float k;
    uint8_t addr;

    if ((config == NULL) ||
        (move == NULL) ||
        (config->wheel_radius_m <= 0.0f) ||
        (config->half_length_m < 0.0f) ||
        (config->half_width_m < 0.0f) ||
        (config->gear_ratio <= 0.0f) ||
        (config->pulse_per_rev == 0U) ||
        (config->max_motor_rpm == 0U) ||
        (config->min_move_time_s < 0.0f)) {
        return false;
    }

    /* 速度因子限幅 */
    if (speed_ratio <= 0.0f) {
        speed_ratio = 1.0f;
    }
    if (speed_ratio > 1.0f) {
        speed_ratio = 1.0f;
    }

    k = config->half_length_m +
        config->half_width_m;

    /*
     * X型麦轮距离逆运动学。
     *
     * body_dx > 0：向车头前方
     * body_dy > 0：向车体左方
     * dtheta > 0：逆时针
     */

    /* 1号：前右 FR */
    distance[MECANUM_ADDR_FR] =
        body_dx_m +
        body_dy_m +
        k * dtheta_rad;

    /* 2号：后左 RL */
    distance[MECANUM_ADDR_RL] =
        body_dx_m +
        body_dy_m -
        k * dtheta_rad;

    /* 3号：前左 FL */
    distance[MECANUM_ADDR_FL] =
        body_dx_m -
        body_dy_m -
        k * dtheta_rad;

    /* 4号：后右 RR */
    distance[MECANUM_ADDR_RR] =
        body_dx_m -
        body_dy_m +
        k * dtheta_rad;

    move->duration_s = 0.0f;
    move->has_motion = false;

    for (addr = 1U; addr <= 4U; addr++) {
        float wheel_rev;
        float abs_motor_rev;

        wheel_rev =
            distance[addr] /
            (2.0f *
             MECANUM_PI *
             config->wheel_radius_m);

        move->motor[addr].wheel_distance_m =
            distance[addr];

        move->motor[addr].motor_rev =
            wheel_rev * config->gear_ratio;

        move->motor[addr].motor_rpm_signed = 0.0f;
        move->motor[addr].vel = 0U;
        move->motor[addr].clk = 0U;

        move->motor[addr].dir =
            Mecanum_GetDriverDir(
                config,
                addr,
                distance[addr]
            );

        abs_motor_rev =
            Mecanum_AbsFloat(
                move->motor[addr].motor_rev
            );

        if (abs_motor_rev > max_abs_motor_rev) {
            max_abs_motor_rev = abs_motor_rev;
        }
    }

    /*
     * 所有轮子的运动量都接近0。
     */
    if (max_abs_motor_rev < MECANUM_EPSILON) {
        return true;
    }

    /*
     * 运动量最大的电机按 (max_motor_rpm * speed_ratio) 运行，
     * 以此计算共同完成时间。
     *
     * speed_ratio = 1.0  → 全速，行为不变
     * speed_ratio = 0.35 → 所有电机 RPM 降至 35%，耗时为 1/0.35 倍
     */
    {
        float effective_max_rpm;

        effective_max_rpm =
            (float)config->max_motor_rpm * speed_ratio;

        if (effective_max_rpm < 1.0f) {
            effective_max_rpm = 1.0f;   /* 保底 1 RPM */
        }

        duration_s =
            max_abs_motor_rev *
            60.0f /
            effective_max_rpm;
    }

    if (duration_s < config->min_move_time_s) {
        duration_s = config->min_move_time_s;
    }

    if (duration_s <= 0.0f) {
        return false;
    }

    move->duration_s = duration_s;
    move->has_motion = true;

    for (addr = 1U; addr <= 4U; addr++) {
        float abs_motor_rev;
        float abs_motor_rpm;
        float pulse_float;

        abs_motor_rev =
            Mecanum_AbsFloat(
                move->motor[addr].motor_rev
            );

        move->motor[addr].motor_rpm_signed =
            move->motor[addr].motor_rev *
            60.0f /
            duration_s;

        abs_motor_rpm =
            Mecanum_AbsFloat(
                move->motor[addr].motor_rpm_signed
            );

        pulse_float =
            abs_motor_rev *
            (float)config->pulse_per_rev;

        if ((pulse_float > (float)UINT32_MAX) ||
            (abs_motor_rpm > (float)UINT16_MAX)) {
            return false;
        }

        move->motor[addr].clk =
            Mecanum_RoundUint32(pulse_float);

        if (move->motor[addr].clk > 0U) {
            move->motor[addr].vel =
                Mecanum_RoundUint16(abs_motor_rpm);

            if (move->motor[addr].vel == 0U) {
                move->motor[addr].vel = 1U;
            }
        } else {
            /*
             * 该轮不需要运动。
             * 速度给1，避免部分驱动器不接受零速度。
             */
            move->motor[addr].vel = 1U;
        }
    }

    return true;
}

/* ============================================================
 * 公开 API — 车体坐标
 * ============================================================ */

bool Mecanum_CalculateMove(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    MecanumMove_t *move
)
{
    return Mecanum_CalcMoveImpl(
        config, body_dx_m, body_dy_m, dtheta_rad,
        1.0f,    /* 全速 */
        move
    );
}

bool Mecanum_CalculateMoveEx(
    const MecanumConfig_t *config,
    float body_dx_m,
    float body_dy_m,
    float dtheta_rad,
    float speed_ratio,
    MecanumMove_t *move
)
{
    return Mecanum_CalcMoveImpl(
        config, body_dx_m, body_dy_m, dtheta_rad,
        speed_ratio,
        move
    );
}

/* ============================================================
 * 公开 API — 世界坐标
 * ============================================================ */

/**
 * @brief World->Body 坐标转换（内部复用）
 */
static bool Mecanum_WorldToBody(
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    float *body_dx,
    float *body_dy
)
{
    float cos_start;
    float sin_start;
    float dx_start_body;
    float dy_start_body;

    cos_start = cosf(start_theta_rad);
    sin_start = sinf(start_theta_rad);

    dx_start_body =
        cos_start * world_dx_m +
        sin_start * world_dy_m;

    dy_start_body =
       -sin_start * world_dx_m +
        cos_start * world_dy_m;

    /*
     * 不旋转时，世界位移只需做普通坐标旋转。
     */
    if (Mecanum_AbsFloat(dtheta_rad) < 1.0e-6f) {
        *body_dx = dx_start_body;
        *body_dy = dy_start_body;
    } else {
        float a;
        float b;
        float denominator;

        /*
         * 固定车体速度并匀速旋转时：
         *
         * dx_world_body0 = a*body_dx - b*body_dy
         * dy_world_body0 = b*body_dx + a*body_dy
         */
        a = sinf(dtheta_rad) / dtheta_rad;

        b =
            (1.0f - cosf(dtheta_rad)) /
            dtheta_rad;

        denominator = a * a + b * b;

        if (denominator < 1.0e-8f) {
            return false;
        }

        /*
         * 对上面的关系求逆，得到需要输入给
         * 车体坐标麦轮解算器的累计运动量。
         */
        *body_dx =
            (a * dx_start_body +
             b * dy_start_body) /
            denominator;

        *body_dy =
            (-b * dx_start_body +
              a * dy_start_body) /
            denominator;
    }

    return true;
}

bool Mecanum_CalculateWorldMove(
    const MecanumConfig_t *config,
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    MecanumMove_t *move
)
{
    float body_dx;
    float body_dy;

    if ((config == NULL) || (move == NULL)) {
        return false;
    }

    if (!Mecanum_WorldToBody(
            world_dx_m, world_dy_m,
            start_theta_rad, dtheta_rad,
            &body_dx, &body_dy)) {
        return false;
    }

    return Mecanum_CalcMoveImpl(
        config, body_dx, body_dy, dtheta_rad,
        1.0f,
        move
    );
}

/* ============================================================
 * 公开 API — 梯形速度斜坡
 * ============================================================ */

/**
 * @brief 一次逻辑运动 -> 两段物理 move（快速接近 + 慢速精停）
 */
bool Mecanum_CalcRampedMoves(
    const MecanumConfig_t *config,
    float world_dx_m,
    float world_dy_m,
    float start_theta_rad,
    float dtheta_rad,
    const MecanumRamp_t *ramp,
    MecanumMove_t *move_fast,
    MecanumMove_t *move_slow,
    float *total_duration_s
)
{
    float body_dx, body_dy;
    float fast_ratio, slow_speed;
    float remain_ratio;

    if ((config == NULL) ||
        (ramp == NULL) ||
        (move_fast == NULL) ||
        (move_slow == NULL) ||
        (total_duration_s == NULL)) {
        return false;
    }

    /* ---- 1. World -> Body ---- */
    if (!Mecanum_WorldToBody(
            world_dx_m, world_dy_m,
            start_theta_rad, dtheta_rad,
            &body_dx, &body_dy)) {
        return false;
    }

    /* ---- 2. 参数限幅 ---- */
    fast_ratio = ramp->fast_ratio;
    if (fast_ratio <= 0.0f)  fast_ratio = 0.85f;
    if (fast_ratio >= 1.0f)  fast_ratio = 0.95f;

    slow_speed = ramp->slow_speed;
    if (slow_speed <= 0.0f)  slow_speed = 0.35f;
    if (slow_speed >  1.0f)  slow_speed = 1.0f;

    remain_ratio = 1.0f - fast_ratio;

    /* ---- 3. Phase 1: 快速接近段 ---- */
    *move_fast = (MecanumMove_t){0};
    if (!Mecanum_CalcMoveImpl(
            config,
            body_dx * fast_ratio,
            body_dy * fast_ratio,
            dtheta_rad * fast_ratio,
            1.0f,           /* 全速 */
            move_fast)) {
        return false;
    }

    /* ---- 4. Phase 2: 慢速精停段 ---- */
    *move_slow = (MecanumMove_t){0};
    if (!Mecanum_CalcMoveImpl(
            config,
            body_dx * remain_ratio,
            body_dy * remain_ratio,
            dtheta_rad * remain_ratio,
            slow_speed,     /* 降速 */
            move_slow)) {
        return false;
    }

    *total_duration_s =
        move_fast->duration_s + move_slow->duration_s;

    return true;
}

/* ============================================================
 * 执行与停止
 * ============================================================ */

bool Mecanum_ExecuteMove(
    const MecanumConfig_t *config,
    const MecanumMove_t *move
)
{
    uint8_t addr;

    if ((config == NULL) || (move == NULL)) {
        return false;
    }

    if (!move->has_motion) {
        return true;
    }

    /*
     * 四轮命令暂存，等待同步触发。
     */
    for (addr = 1U; addr <= 4U; addr++) {
        Emm_V5_Pos_Control(
            addr,
            move->motor[addr].dir,
            move->motor[addr].vel,
            config->acceleration,
            move->motor[addr].clk,

            false, /* raF=false：相对位置模式 */
            false  /* snF=false：多机同步 */
        );

    }

    /*
     * 地址0触发四轮同步开始。
     */
    Emm_V5_Synchronous_motion(
        0
    );

    return true;
}

void Mecanum_StopAll(void)
{
    uint8_t addr;

    for (addr = 1U; addr <= 4U; addr++) {
        Emm_V5_Stop_Now(
            addr,
            false
        );
    }
}



void Servo_SetAngle(float Angle)
{
    if(Angle>180){Angle=180;}
    if(Angle<0){Angle=0;}


    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Angle / 180 * 2000 + 500);

}
