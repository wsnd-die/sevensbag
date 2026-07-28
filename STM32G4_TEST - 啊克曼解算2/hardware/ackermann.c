#include "ackermann.h"

// ************************ 【严格复刻你的Python】转向角转舵机PWM ************************
static float angle_to_phid(float theta_deg)
{
    // 完全和你Python一致的参数
    const float a = 0.0011f;
    const float b = 0.0451f;
    const float c = -15.51f;

    // 计算有效角度范围（对应 phid 32 和 148）
    float theta_min = a * 32.0f*32.0f + b * 32.0f + c;
    float theta_max = a * 148.0f*148.0f + b * 148.0f + c;

    // 限幅到标定角度范围
    theta_deg = fmaxf(fminf(theta_deg, theta_max), theta_min);

    // 解二次方程: a*x^2 + b*x + (c - theta) = 0
    float disc = b*b - 4.0f * a * (c - theta_deg);
    // 判别式负数置0
    if(disc < 0.0f) disc = 0.0f;

    // 取正根（函数在区间内单调递增）
    float phid = (-b + sqrtf(disc)) / (2.0f * a);

    // 硬限幅到 [32, 148]
    phid = fmaxf(fminf(phid, 148.0f), 32.0f);

    return phid;
}

// ************************ 核心阿克曼解算函数 ************************
AckermanResult Ackerman_Calc(float v, float w) {
    AckermanResult res = {0, 0, 0, 0, 100.0f}; // 舵机中位初始化

    // 静止直接返回
    if(fabsf(v) < STOP_THRESHOLD) return res;

    // 1. 计算转向角 + 限幅
    float delta = atanf((w * WHEELBASE) / v);
    delta = fmaxf(fminf(delta, MAX_STEER_ANGLE), -MAX_STEER_ANGLE);

    // 2. 左右轮转速计算
    float left_raw, right_raw;
    if(fabsf(delta) < STOP_THRESHOLD) {
        left_raw  = SPEED_COEFF * v;
        right_raw = SPEED_COEFF * v;
    } else {
        float R = WHEELBASE / tanf(delta);
        left_raw  = SPEED_COEFF * v * (R - TRACK_WIDTH/2) / R;
        right_raw = SPEED_COEFF * v * (R + TRACK_WIDTH/2) / R;
    }

    // 3. 低速动力放大
    if(fabsf(v) < LOW_SPEED_LIMIT) {
        left_raw  *= LOW_SPEED_GAIN;
        right_raw *= LOW_SPEED_GAIN;
    }

    // 4. 电机方向与转速处理
    res.left_dir  = left_raw >= 0 ? 1 : 0;
    res.right_dir = right_raw >= 0 ? 1 : 0;
    left_raw  = fabsf(left_raw);
    right_raw = fabsf(right_raw);

    // 最小转速保护 + 限幅
    left_raw  = left_raw > 0 ? fmaxf(left_raw, MIN_MOTOR_SPEED) : 0;
    right_raw = right_raw > 0 ? fmaxf(right_raw, MIN_MOTOR_SPEED) : 0;
    res.left_speed  = (uint16_t)fminf(left_raw, 65535.0f);
    res.right_speed = (uint16_t)fminf(right_raw, 65535.0f);

    // 5. 调用【你的原版】舵机转换函数
    res.servo_pwm = angle_to_phid(delta * 180.0f / (float)M_PI);

    return res;
}