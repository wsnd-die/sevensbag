/**
 * @file    block_basic.c
 * @brief   物块基础机构封装：丝杆升降、双机械臂升降、转盘定位。
 * @note    车型 1 使用丝杆机构；车型 2 使用双机械臂机构。
 */
#include "Common_used.h"

#define CLAMP_FLOAT(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define DEG2RAD(d)              ((d) * 0.01745329252f)
#define RAD2DEG(r)              ((r) * 57.2957795131f)

/**
 * @brief   软件记录的转盘角度，单位 deg。
 * @note    1号位置对应 BLOCK_TURNTABLE_HOME_DEG，后续位置每个间隔 BLOCK_TURNTABLE_STEP_DEG，总共 BLOCK_TURNTABLE_POS_COUNT 个位置。
 */
static float angle_servo = BLOCK_TURNTABLE_HOME_DEG;

/**
 * @brief   物块舵机角度写入,把角度转换成 TIM3 PWM 比较值。
 * @param   channel         TIM3 通道号，CH1/CH2/CH3。
 * @param   angle_deg       目标角度，单位 deg。
 * @param   full_angle_deg  舵机最大角度，180 或 360
 *          TIM_CHANNEL_1 -> 转盘舵机 CH1
 *          TIM_CHANNEL_2 -> 双舵机前级 CH2
 *          TIM_CHANNEL_3 -> 双舵机后级 CH3
 */
static void block_servo_write(uint32_t channel, float angle_deg)
{
    float angle = CLAMP_FLOAT(angle_deg, 0.0f , BLOCK_SERVO_DEG);
    float pulse = 500.0f + (angle / BLOCK_SERVO_DEG) * (2500.0f - 500.0f);

    __HAL_TIM_SET_COMPARE(&htim3, channel, (uint32_t)(pulse + 0.5f));
}

/**
 * @brief   将任意角度归一化到 [0, 360)。
 * @param   angle_deg  输入角度，单位 deg。
 * @return  归一化后的角度，单位 deg。
 */
static float normalize_servo(float angle_deg)
{
    while (angle_deg >= 0.0f) {
        angle_deg -= BLOCK_SERVO_DEG;
    }
    while (angle_deg < 0.0f) {
        angle_deg += BLOCK_SERVO_DEG;
    }
    return angle_deg;
}

/**
 * @brief   物块位置编号转换为转盘角度。
 * @param   block_pos   物块位置编号，合法范围为 1~5。
 * @return  float       转盘角度，单位 deg。
 * @note    1 -> HOME, 2 -> HOME + 72 deg, ..., 5 -> HOME + 288 deg 
 */
static float turntable_target_angle(uint8_t block_pos)
{
    return normalize_servo(BLOCK_TURNTABLE_HOME_DEG +
                         (float)(block_pos - BLOCK_TURNTABLE_FIRST_POS) *
                         BLOCK_TURNTABLE_STEP_DEG);
}

/**
 * @brief   转盘角度写入。
 * @param   angle_deg   目标角度，单位 deg。
 * @note    360 度位置舵机，CH2。
 */
static void turntable_write_angle(float angle_deg)
{
    block_servo_write(TIM_CHANNEL_2, normalize_servo(angle_deg));
}

/**
 * @brief   根据编译期选定的车型执行对应升降机构，并统一返回转盘后退距离。
 * @param   height_mm   目标升高的高度，单位 mm。
 * @return  float       >=0 为转盘相对后退距离，<0 表示参数错误或运动失败。
 */
float BlockBasic_LiftTo(uint8_t dir,float height_mm)
{

#if BLOCK_USE_DUAL_ARM
    /* 双机械臂型：使用 CH2/CH3 两个舵机升降。 */
    if (height_mm > BLOCK_ARM_MAX_HEIGHT_MM) {
        return -1.0f;
    }

    BlockArmResult result = BlockBasic_ArmCalc(height_mm);
    block_servo_write(TIM_CHANNEL_2, result.front_angle_deg, BLOCK_SERVO_180_DEG);
    block_servo_write(TIM_CHANNEL_3, result.rear_angle_deg, BLOCK_SERVO_180_DEG);
    return result.turntable_retreat_mm;
#else
    /* 丝杆型：使用 5 号步进电机升降。 */
    {//43到达底部
        uint32_t pulse = (uint32_t)(height_mm * BLOCK_STEPPER_PULSE_PER_MM );
        if (dir == 0)
        {
            Emm_V5_Pos_Control(5, 0, 300, 50, pulse, 0, 0);
        }
        else
        {
            Emm_V5_Pos_Control(5, 1, 300, 50, pulse, 0, 0);
        }
        return 0.0f;
    }
#endif
}

#if BLOCK_USE_DUAL_ARM
/**
 * @brief   计算双舵机角度和转盘退距。
 * @param   height_mm   目标上升高度，单位 mm。
 * @return  BlockArmResult  计算结果。
 */
BlockArmResult BlockBasic_ArmCalc(float height_mm)
{
    /* 采用数学建模公式：S1(x_t) = 90° + arcsin(sin(-12°) + \frac {x_t} {10.5})
                       S2 = S1 - 9.51° */
    float height = CLAMP_FLOAT(height_mm,
                               BLOCK_ARM_MIN_HEIGHT_MM,
                               BLOCK_ARM_MAX_HEIGHT_MM);
    float height_cm = height / 10.0f; // 将高度从 mm 转换为 cm
    float asin_arg = sinf(DEG2RAD(BLOCK_ARM_INIT_DEG)) +
                     height_cm / BLOCK_ARM_LINK_CM;
    float servo1_deg;

    asin_arg = CLAMP_FLOAT(asin_arg, -1.0f, 1.0f);
    servo1_deg = 90.0f + RAD2DEG(asinf(asin_arg));

    BlockArmResult result;
    result.front_angle_deg = servo1_deg;
    result.rear_angle_deg = servo1_deg - BLOCK_ARM_S2_OFFSET_DEG;
    result.turntable_retreat_mm = height * BLOCK_ARM_RETREAT_PER_MM;
    return result;
}
#endif


/**
 * @brief  将转盘转动到指定位置。
 * @param  block_pos  目标位置，单位个。
 * @return BlockStatus  执行状态。
 */

BlockStatus BlockBasic_TurntableTo(uint8_t block_pos)
{
    if (block_pos < BLOCK_TURNTABLE_FIRST_POS ||
        block_pos >= BLOCK_TURNTABLE_FIRST_POS + BLOCK_TURNTABLE_POS_COUNT) {
        return BLOCK_ERR_PARAM;
    }

    angle_servo = turntable_target_angle(block_pos);
    turntable_write_angle(angle_servo);
    return BLOCK_OK;
}

/**
 * @brief  重置软件记录的转盘当前角度，并立即输出该角度 PWM。
 * @param  angle_deg  当前机械角度，单位 deg；会归一化到 0~360。
 */
void Servo_Angle(float angle_deg)
{
    angle_servo = normalize_servo(angle_deg);
    turntable_write_angle(angle_servo);
}

void Servo_SetAngle(float Angle)
{
    if(Angle>=125){Angle=125;}
    if(Angle<=37){Angle=37;}
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Angle / 180 * 2000 + 500);

}
void Place(char dir)
{
    MecanumMove_t move;

    if (dir == 'O')
    {
        /* 前进 0.05 m（车体坐标：+X 为前进） */
        if (Mecanum_CalculateMove(&g_mecanum_config, 0.05f, 0.0f, 0.0f, &move))
        {
            Mecanum_ExecuteMove(&g_mecanum_config, &move);
            osDelay((uint32_t)(move.duration_s * 1000.0f) + 50U);
        }

        /* 后退 0.05 m（车体坐标：-X 为后退） */
        if (Mecanum_CalculateMove(&g_mecanum_config, -0.05f, 0.0f, 0.0f, &move))
        {
            Mecanum_ExecuteMove(&g_mecanum_config, &move);
            osDelay((uint32_t)(move.duration_s * 1000.0f) + 50U);
        }
    }

}
