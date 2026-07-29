#include "ackermann1.h"
#include "arm_math.h"
 #include "Common_used.h"
 
 float p1,p2,p3,p4,p5,phid;
 float data_angle;
// ************************ 【严格复刻你的Python】转向角转舵机PWM ************************
/*
 * 左转为正。
 *
 * LEFT_SMALL_GAIN：
 *   小角度左转增益。1.40 表示零度附近最多放大约 40%。
 *
 * LEFT_COMP_END_DEG：
 *   补偿作用范围。达到 10° 后完全使用原拟合。
 */
#define LEFT_SMALL_GAIN       1.40f
#define LEFT_COMP_END_DEG     10.0f

static float angle_to_phid(float theta_deg)
{
    float p1;
    float p2;
    float p3;
    float p4;

#if use_xing_che

    p1 = 0.00075747f;
    p2 = -0.01096567f;
    p3 = 2.39767131f;
    p4 = 103.885585f;

#else
 
    /* 老车拟合参数 */
    // p1 = 0.000307467221f ;
    // p2 = - 0.0106318330f ;
    // p3 = 2.77412983f;
    // p4 = 103.675360f;
    p1 = -0.00196112f;
    p2 = 0.00029658f;
    p3 = -2.07088302f;
    p4 = 92.35285048f;


#endif

    float fit_angle = theta_deg;

    /*
     * 左转为正，只补偿正方向的小角度。
     *
     * theta = 0° 时，增益最大；
     * theta = 10° 时，补偿平滑降为零；
     * theta > 10° 时，保持原拟合结果。
     */
    if ((theta_deg > 0.0f) &&
        (theta_deg < LEFT_COMP_END_DEG)) {

        float ratio =
            theta_deg / LEFT_COMP_END_DEG;

        /*
         * 平滑衰减系数：
         *
         * theta接近0°：fade接近1
         * theta接近10°：fade接近0
         */
        float fade =
            (1.0f - ratio) *
            (1.0f - ratio);

        float gain =
            1.0f +
            (LEFT_SMALL_GAIN - 1.0f) * fade;

        fit_angle =
            theta_deg * gain;
    }

    /*
     * 三次拟合。
     * 使用霍纳形式，运算量更小。
     */
    float phid =
        ((p1 * fit_angle + p2) *
        fit_angle + p3) *
        fit_angle + p4;

    return phid;
}


// ************************ 核心阿克曼解算函数 ************************
AckermanResult Ackerman_Calc(float v, float w) {
    AckermanResult res = {0, 0, 0, 0, 94.35285048f};

    // 静止直接返回
    if(fabsf(v) < STOP_THRESHOLD) return res;

    // 1. 计算转向角 + 限幅
    float delta = atanf((w * WHEELBASE) / v);
		
    delta = fmaxf(fminf(delta, 0.4f), -0.37f);
		data_angle=delta;
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
		#if use_xing_che
    res.servo_pwm = angle_to_phid(delta * 180.0f /PI);
		#else
		res.servo_pwm = angle_to_phid(delta * 180.0f /PI);
		#endif
    return res;
}




