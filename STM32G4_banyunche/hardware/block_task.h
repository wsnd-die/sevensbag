#ifndef BLOCK_TASK_H
#define BLOCK_TASK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== block task configuration =========================
 *
 * 负责物块相关机构的上层换算和动作下发：
 * 1. 丝杆车型：把上升高度(mm)换算成 5 号 EMM 步进电机位置模式脉冲。
 * 2. 双舵机车型：把上升高度(mm)换算成 CH2/CH3 两个舵机角度。
 * 3. 转盘机构：把 1~5 号物块位置换算成 CH1 的 360 度舵机角度。
 */

/* 丝杆每上升 1mm 需要的 EMM 位置模式脉冲数，需标定。 */
#define BLOCK_STEPPER_PULSE_PER_MM       320.0f

/* TIM3 PWM 参数。当前 TIM3 周期为 20ms，比较值 500~2500 对应 0.5~2.5ms。 */
#define BLOCK_SERVO_180_DEG              180.0f // 180 度位置舵机
#define BLOCK_SERVO_360_DEG              360.0f // 360 度位置舵机

/* 双舵机机械臂框架参数：CH2 为前级舵机，CH3 为后级舵机。 */
#define BLOCK_ARM_MIN_HEIGHT_MM          0.0f
#define BLOCK_ARM_MAX_HEIGHT_MM          100.0f
#define BLOCK_ARM_FRONT_MIN_DEG          30.0f
#define BLOCK_ARM_FRONT_MAX_DEG          120.0f
#define BLOCK_ARM_REAR_MIN_DEG           150.0f
#define BLOCK_ARM_REAR_MAX_DEG           60.0f
/* 上升高度导致转盘需要相对后退的距离系数；真实值需由机构几何确定。 */
#define BLOCK_ARM_RETREAT_PER_MM         0.0f

/* 转盘位置编号从 1 开始，用户传入 1~5，内部再换算成 0~4 的角度序号。 */
#define BLOCK_TURNTABLE_FIRST_POS        1u
#define BLOCK_TURNTABLE_POS_COUNT        5u
#define BLOCK_TURNTABLE_HOME_DEG         0.0f // 1号位置对应的 HOME 角度，单位 deg
#define BLOCK_TURNTABLE_STEP_DEG         (BLOCK_SERVO_360_DEG / BLOCK_TURNTABLE_POS_COUNT)
/* 单次最大角度步长。分段移动用于降低 360 度位置舵机自动走最短路径的风险。 */
#define BLOCK_TURNTABLE_STEP_LIMIT_DEG   60.0f
#define BLOCK_TURNTABLE_STEP_DELAY_MS    80u

/* 返回值区分成功和参数错误 */
typedef enum {
    BLOCK_OK = 0,
    BLOCK_ERR_PARAM = 1
} BlockStatus;

/* 双舵机高度换算结果 */
typedef struct {
    float front_angle_deg;
    float rear_angle_deg;
    float turntable_retreat_mm;
} BlockArmResult;

/**
 * @brief  丝杆升降到指定高度。
 * @param  height_mm     目标上升高度，单位 mm；小于 0 会返回参数错误。
 * @param  target_pulse  可选输出，返回换算后的 EMM 位置模式脉冲数；不需要可传 NULL。
 * @retval BLOCK_OK / BLOCK_ERR_PARAM
 *
 * @note   使用 5 号步进电机位置模式，raF=true 表示绝对位置，snF=false 表示不参与同步触发。
 */
BlockStatus BlockTask_ScrewLiftTo(float height_mm, uint32_t *target_pulse);

/**
 * @brief  双舵机机械臂升降到指定高度。
 * @param  height_mm  目标上升高度，单位 mm。
 * @retval 转盘需要相对后退的距离，单位 mm。
 *
 * @note   当前是线性映射框架。真实机械臂如果是连杆机构，应把 BlockTask_ArmCalc()
 *         内部替换为几何反解或标定表。
 */
float BlockTask_ArmLiftTo(float height_mm);

/**
 * @brief  只计算双舵机高度对应关系，不实际输出 PWM。
 * @param  height_mm  目标上升高度，单位 mm。
 * @retval 前级角度、后级角度、转盘退距。
 */
BlockArmResult BlockTask_ArmCalc(float height_mm);

/**
 * @brief  转盘转到指定物块位置。
 * @param  block_pos  物块位置编号，合法范围为 1~5。
 * @retval BLOCK_OK / BLOCK_ERR_PARAM
 *
 * @note   CH1 按 360 度位置型舵机处理。若实际是连续旋转速度型 360 舵机，
 *         本接口只能作为框架，不能保证绝对角度定位。
 */
BlockStatus BlockTask_TurntableTo(uint8_t block_pos);

/**
 * @brief  重置软件记录的转盘当前角度，并立即输出该角度 PWM。
 * @param  angle_deg  当前机械角度，单位 deg；会归一化到 0~360。
 * @note   上电后如果转盘实际位置不在 BLOCK_TURNTABLE_HOME_DEG，应先调用本函数同步软件状态。
 */
void servo_angle(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif
