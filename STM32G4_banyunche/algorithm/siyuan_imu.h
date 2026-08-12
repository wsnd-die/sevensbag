#ifndef __SIYUAN_IMU_H__
#define __SIYUAN_IMU_H__

#include <math.h>
#include <stdint.h>

/* ============================================================
 * Mahony AHRS 状态结构体
 * ============================================================ */

typedef struct {
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
    float twoKp;
    float twoKi;
} SiyuanAHRS_t;

/* 默认参数 */
#define SIYUAN_KP_DEFAULT   1.5f    /* twoKp=3.0, 与 ahrs_mahony Kp=3 对齐 */
#define SIYUAN_KI_DEFAULT   0.0f

/* ============================================================
 * 加速度模长拒绝阈值
 * ============================================================ */

#define ACC_NORM_MIN        3400.0f
#define ACC_NORM_MAX        4600.0f
#define ACC_REJECT_COUNT    20

/* ============================================================
 * 误差限幅 & 逐轴增益
 * ============================================================ */

#define ERR_LIMIT           0.10f

#define KP_X_SCALE          0.20f
#define KP_Y_SCALE          1.00f
#define KP_Z_SCALE          1.00f

#define KI_X_SCALE          0.00f
#define KI_Y_SCALE          1.00f
#define KI_Z_SCALE          1.00f

#define INTEGRAL_LIMIT      0.05f

/* ============================================================
 * 陀螺仪 Z 轴校准 & 滤波
 * ============================================================ */

#define GYRO_CALIB_NUM      300
#define GYRO_RATE_LPF_ALPHA 0.25f
#define GYRO_STATIC_THRESH  0.02f

extern float siyuan_gyro_z_bias;
extern float siyuan_gyro_z_rate;
extern float siyuan_roll, siyuan_pitch, siyuan_yaw;

/* ============================================================
 * API
 * ============================================================ */

void siyuan_ahrs_init(void);

void siyuan_ahrs_update(SiyuanAHRS_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt);

void siyuan_quat_to_euler(float q0, float q1, float q2, float q3,
                          float *roll, float *pitch, float *yaw);

void siyuan_gyro_calibrate(void);

float siyuan_update_gyro_rate(float gz);

void siyuan_degree_update(float *yaw, float *pitch, float *roll);

void siyuan_imu_task(void);

#endif
