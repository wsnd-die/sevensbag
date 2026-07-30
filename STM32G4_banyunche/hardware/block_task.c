/**
 * @file    block_task.c
 * @brief   物块相关任务封装：丝杆/双舵机升降，转盘，颜色传感器
 * @note    物块任务的丝杆升降和双舵机升降分别对应两台车的情况
 */
#include "block_task.h"

#include "stm32g4xx.h"
#include "cmsis_os2.h"
#include "emm_5v.h"
#include "tim.h"

#define CLAMP_FLOAT(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v))) // 浮点数限幅定义


/**
 * @brief   软件记录的转盘角度，单位 deg。
 * @note    1号位置对应 BLOCK_TURNTABLE_HOME_DEG，后续位置每个间隔 BLOCK_TURNTABLE_STEP_DEG，总共 BLOCK_TURNTABLE_POS_COUNT 个位置。
 */
static float angle_servo = BLOCK_TURNTABLE_HOME_DEG; // BLOCK_TURNTABLE_HOME_DEG 对应 1 号位置，单位 deg

/**
 * @brief   取浮点数绝对值。
 * @param   value 
 * @return  float 
 */
static float block_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief   物块舵机角度写入,把角度转换成 TIM3 PWM 比较值。。
 * @param   channel         TIM3 通道号，CH1/CH2/CH3。
 * @param   angle_deg       目标角度，单位 deg。
 * @param   full_angle_deg  舵机最大角度，180 或 360
 *          TIM_CHANNEL_1 -> 转盘舵机 CH1
 *          TIM_CHANNEL_2 -> 双舵机前级 CH2
 *          TIM_CHANNEL_3 -> 双舵机后级 CH3
 */
static void block_servo_write(uint32_t channel, float angle_deg, float full_angle_deg)
{
    float angle = CLAMP_FLOAT(angle_deg, 0.0f, full_angle_deg);
    float pulse = 500.0f + (angle / full_angle_deg) * (2500.0f - 500.0f);// 计算对应的 PWM 脉冲宽度
    // 500.0f 对应 0 度，2500.0f 对应 full_angle_deg 度
    // 使用 TIM3 比较寄存器来设置 PWM 输出的占空比，从而控制舵机的角度。 

    __HAL_TIM_SET_COMPARE(&htim3, channel, (uint32_t)(pulse + 0.5f));
}

/**
 * @brief   将任意角度归一化到 [0, 360)。
 * @param   angle_deg   输入角度，单位 deg。
 * @return  float       归一化后的角度，单位 deg。
 */
static float normalize_360(float angle_deg)
{
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
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
    return normalize_360(BLOCK_TURNTABLE_HOME_DEG +
                         (float)(block_pos - BLOCK_TURNTABLE_FIRST_POS) * BLOCK_TURNTABLE_STEP_DEG);// 计算目标角度，确保在 [0, 360) 范围内
    // 1号位置对应 HOME 角度，后续位置每个间隔 72 度，总共 5 个位置。
    // 例如：block_pos=1 -> HOME, block_pos=2 -> HOME + 72, ..., block_pos=5 -> HOME + 288
}

/**
 * @brief   转盘角度写入。
 * @param   angle_deg   目标角度，单位 deg。
 * @note    360 度位置舵机，CH1。
 */
static void turntable_write_angle(float angle_deg)
{
    block_servo_write(TIM_CHANNEL_1, normalize_360(angle_deg), BLOCK_SERVO_360_DEG);
}

/**
 * @brief   丝杆升降到指定高度。
 * @param   height_mm     目标上升高度，单位 mm；小于 0 会返回参数错误。
 * @param   target_pulse  可选输出，返回换算后的 EMM 位置模式脉冲数；不需要可传 NULL。
 * @retval  BLOCK_OK / BLOCK_ERR_PARAM
 * @note    使用 5 号步进电机位置模式，raF=true 表示绝对位置，snF=false 表示不参与同步触发。
 */
BlockStatus BlockTask_ScrewLiftTo(float height_mm, uint32_t *target_pulse)
{
    if (height_mm < 0.0f) {
        return BLOCK_ERR_PARAM;
    }
    uint32_t pulse = (uint32_t)(height_mm * BLOCK_STEPPER_PULSE_PER_MM + 0.5f);
    if (target_pulse != NULL) {
        *target_pulse = pulse;
    }

    /* raF=true  : 绝对位置模式。
     * snF=false : 不使用多机同步触发，满足“不同步触发”的要求。
     */
    Emm_V5_Pos_Control(5, 1, 300, 50 , pulse, 0 , 0);
    return BLOCK_OK;
}

/**
 * @brief   计算双舵机角度和转盘退距。
 * @param   height_mm   目标上升高度，单位 mm。
 * @return  BlockArmResult  计算结果。
 */
BlockArmResult BlockTask_ArmCalc(float height_mm)
{
    /* 当前只搭建可标定框架：
     * - 先把高度限制在机构允许范围。
     * - 再按高度比例线性插值两个舵机角度。
     *
     * 后续如果拿到连杆长度、安装角、零位角，可在这里替换成几何反解。
     * 如果用实测数据，也可以改成查表 + 插值，不影响外部接口。
     */
    float height = CLAMP_FLOAT(height_mm,
                               BLOCK_ARM_MIN_HEIGHT_MM,
                               BLOCK_ARM_MAX_HEIGHT_MM);
    float span = BLOCK_ARM_MAX_HEIGHT_MM - BLOCK_ARM_MIN_HEIGHT_MM;
    float ratio = (span > 0.0f) ? ((height - BLOCK_ARM_MIN_HEIGHT_MM) / span) : 0.0f;

    BlockArmResult result;
    /* CH2: 前级舵机角度。 */
    result.front_angle_deg = BLOCK_ARM_FRONT_MIN_DEG +
                             ratio * (BLOCK_ARM_FRONT_MAX_DEG - BLOCK_ARM_FRONT_MIN_DEG);
    /* CH3: 后级舵机角度。 */
    result.rear_angle_deg = BLOCK_ARM_REAR_MIN_DEG +
                            ratio * (BLOCK_ARM_REAR_MAX_DEG - BLOCK_ARM_REAR_MIN_DEG);
    /* 转盘相对后退距离。目前用线性系数占位。 */
    result.turntable_retreat_mm = height * BLOCK_ARM_RETREAT_PER_MM;
    return result;
}

float BlockTask_ArmLiftTo(float height_mm)
{
    BlockArmResult result = BlockTask_ArmCalc(height_mm);

    block_servo_write(TIM_CHANNEL_2, result.front_angle_deg, BLOCK_SERVO_180_DEG);
    block_servo_write(TIM_CHANNEL_3, result.rear_angle_deg, BLOCK_SERVO_180_DEG);

    return result.turntable_retreat_mm;
}

BlockStatus BlockTask_TurntableTo(uint8_t block_pos)
{
    if (block_pos < BLOCK_TURNTABLE_FIRST_POS ||
        block_pos >= BLOCK_TURNTABLE_FIRST_POS + BLOCK_TURNTABLE_POS_COUNT) {
        return BLOCK_ERR_PARAM;
    }

    float target = turntable_target_angle(block_pos);
    float delta = target - angle_servo;

    /* 分段逼近目标角度。
     *
     * 一些 360 度位置舵机在收到跨度较大的目标角时，会自动选择最短路径。
     * 例如从 10deg 到 300deg，舵机可能反向转 70deg，而不是正向转 290deg。
     * 这里把一次大转动拆成多个小步，尽量让每一步都落在可预期方向上。
     *
     * 如果最终要求“只能单方向转动”，还需要根据实际舵机协议或外部反馈进一步加强。
     */
    while (block_absf(delta) > BLOCK_TURNTABLE_STEP_LIMIT_DEG) {
        angle_servo += (delta > 0.0f) ?
                                 BLOCK_TURNTABLE_STEP_LIMIT_DEG :
                                 -BLOCK_TURNTABLE_STEP_LIMIT_DEG;
        turntable_write_angle(angle_servo);
        block_delay_ms(BLOCK_TURNTABLE_STEP_DELAY_MS);
        delta = target - angle_servo;
    }

    angle_servo = target;
    turntable_write_angle(angle_servo);
    return BLOCK_OK;
}

/**
 * @brief  重置软件记录的转盘当前角度，并立即输出该角度 PWM。
 * @param  angle_deg  当前机械角度，单位 deg；会归一化到 0~360。
 * @note   上电后如果转盘实际位置不在 BLOCK_TURNTABLE_HOME_DEG，应先调用本函数同步软件状态。
 */
void servo_angle(float angle_deg)
{
    angle_servo = normalize_360(angle_deg);
    turntable_write_angle(angle_servo);
}
