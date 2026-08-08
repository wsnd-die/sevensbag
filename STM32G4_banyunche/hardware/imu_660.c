/**
 * @file    imu660.c
 * @brief   IMU660RA 驱动实现（STM32G4 适配版）
 *
 * 基于逐飞 TC264 开源库 IMU660RA 驱动，移植适配到 STM32G4 平台。
 * 包含 IMU660RA 传感器初始化、数据读取与 Mahony 互补滤波姿态解算。
 *
 * 原始来源：逐飞科技 zf_device_imu660ra（GPL3.0）
 */

#include "Common_used.h"

/******************************************************************************
 *                              全局变量定义
 ******************************************************************************/

IMU660RA_ConfigType imu_cfg = {0};
IMU660RA_DataType   imu_data = {0};

// 逐飞风格全局变量
int16_t imu660ra_gyro_x = 0, imu660ra_gyro_y = 0, imu660ra_gyro_z = 0;
int16_t imu660ra_acc_x  = 0, imu660ra_acc_y  = 0, imu660ra_acc_z  = 0;
float   imu660ra_transition_factor[2] = {4096.0f, 16.384f};  // 默认 ±8G / ±2000dps

/******************************************************************************
 *                          姿态解算静态变量
 ******************************************************************************/

#define DEG_TO_RAD      0.017453292519943295f
#define RAD_TO_DEG      57.29577951308232f

#define MAHONY_KP       1.5f
#define MAHONY_KI       0.003f
#define INTEGRAL_LIMIT  0.15f

static float att_q[4]        = {1.0f, 0.0f, 0.0f, 0.0f};
static float integral_fb[3]  = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3]    = {0.0f, 0.0f, 0.0f};
static float acc_lpf[3]      = {0.0f, 0.0f, 1.0f};

/******************************************************************************
 *                          SPI 底层通信函数
 ******************************************************************************/

/**
 * @brief  读 IMU660RA 单个寄存器
 * @param  reg  寄存器地址（bit7=0 会自动添加读标志）
 * @return 寄存器值
 */
uint8_t IMU660RA_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = {reg | IMU660RA_SPI_R, 0xFF};
    uint8_t rx[2] = {0, 0};

    IMU660RA_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 1000);
    IMU660RA_CS_HIGH();

    return rx[1];
}

/**
 * @brief  写 IMU660RA 单个寄存器
 * @param  reg   寄存器地址（bit7 会被清零以确保写操作）
 * @param  data  要写入的数据
 */
void IMU660RA_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg & 0x7F, data};

    IMU660RA_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx, 2, 1000);
    IMU660RA_CS_HIGH();
}

/**
 * @brief  读 IMU660RA 16 位寄存器对（小端序，reg_low 为低字节地址）
 * @param  reg_low  低字节寄存器地址
 * @return 16 位寄存器值
 */
uint16_t IMU660RA_ReadReg16b(uint8_t reg_low)
{
    uint8_t data_l = IMU660RA_ReadReg(reg_low);
    uint8_t data_h = IMU660RA_ReadReg(reg_low + 1);
    return (uint16_t)((data_h << 8) | data_l);
}

/**
 * @brief  批量读 IMU660RA 寄存器（自动地址递增）
 * @param  reg   起始寄存器地址
 * @param  buf   接收缓冲区
 * @param  len   要读取的字节数
 */
void IMU660RA_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[len + 1];
    tx[0] = reg | IMU660RA_SPI_R;
    for (uint8_t i = 0; i < len; i++) {
        tx[i + 1] = 0xFF;   // dummy 字节
    }

    uint8_t rx[len + 1];
    IMU660RA_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len + 1, 1000);
    IMU660RA_CS_HIGH();

    memcpy(buf, &rx[1], len);
}

/******************************************************************************
 *                       IMU660RA 传感器驱动函数
 ******************************************************************************/

/**
 * @brief  初始化 IMU660RA
 * @return 1 = 成功，0 = 失败
 *
 * 初始化流程：
 *  1. 软件复位
 *  2. 校验芯片 ID（期望 0x69）
 *  3. 配置电源模式
 *  4. 执行内部初始化
 *  5. 配置加速度计量程和输出频率
 *  6. 配置陀螺仪量程和输出频率
 */
uint8_t imu660ra_init(void)
{
    uint8_t whoami;

    // ---- 1. 软件复位 ----
    IMU660RA_WriteReg(IMU660RA_PWR_CTRL, 0xB6);
    HAL_Delay(50);

    // ---- 2. 校验芯片 ID ----
    whoami = IMU660RA_ReadReg(IMU660RA_CHIP_ID);
    printf("[IMU660RA] WHO_AM_I = 0x%02X\r\n", whoami);
    if (whoami != IMU660RA_DEV_ADDR)    // 期望 0x69
    {
        printf("[IMU660RA] Init failed! Expected 0x%02X\r\n", IMU660RA_DEV_ADDR);
        return 0;
    }

    // ---- 3. 电源配置 ----
    // PWR_CONF: 使能温度传感器，关闭休眠模式
    IMU660RA_WriteReg(IMU660RA_PWR_CONF, 0x02);
    HAL_Delay(30);

    // ---- 4. 内部初始化序列 ----
    // 写入 INIT_CTRL 启动初始化
    IMU660RA_WriteReg(IMU660RA_INIT_CTRL, 0x00);
    // 写入初始化配置数据（根据芯片手册设置，以下为典型值）
    IMU660RA_WriteReg(IMU660RA_INIT_DATA, 0x05);
    HAL_Delay(30);

    // ---- 5. 配置加速度计 ----
    {
        imu660ra_acc_range_t  acc_range  = IMU660RA_ACC_SAMPLE_DEFAULT;
        float                 acc_sens   = 4096.0f;   // 默认 ±8G: 4096 LSB/g

        // 根据量程设置灵敏度系数
        switch (acc_range) {
            case IMU660RA_ACC_SGN_2G:  acc_sens = 16384.0f; break;
            case IMU660RA_ACC_SGN_4G:  acc_sens =  8192.0f; break;
            case IMU660RA_ACC_SGN_8G:  acc_sens =  4096.0f; break;
            case IMU660RA_ACC_SGN_16G: acc_sens =  2048.0f; break;
        }

        // ACC_CONF: 加速度计 ODR = 200Hz, 正常模式, 带宽 ~40Hz
        // 具体位定义需参考芯片手册，以下为典型配置值
        IMU660RA_WriteReg(IMU660RA_ACC_CONF,  0xA7);     // ODR=200Hz, BW=normal
        IMU660RA_WriteReg(IMU660RA_ACC_RANGE, (uint8_t)acc_range);

        imu_cfg.acc_range       = acc_range;
        imu_cfg.acc_sensitivity = acc_sens;
        imu660ra_transition_factor[0] = acc_sens;
    }

    // ---- 6. 配置陀螺仪 ----
    {
        imu660ra_gyro_range_t gyro_range = IMU660RA_GYRO_SAMPLE_DEFAULT;
        float                 gyro_sens  = 16.384f;  // 默认 ±2000dps: 16.384 LSB/(°/s)

        switch (gyro_range) {
            case IMU660RA_GYRO_SGN_125DPS:  gyro_sens = 262.144f; break;
            case IMU660RA_GYRO_SGN_250DPS:  gyro_sens = 131.072f; break;
            case IMU660RA_GYRO_SGN_500DPS:  gyro_sens =  65.536f; break;
            case IMU660RA_GYRO_SGN_1000DPS: gyro_sens =  32.768f; break;
            case IMU660RA_GYRO_SGN_2000DPS: gyro_sens =  16.384f; break;
        }

        // GYR_CONF: 陀螺仪 ODR = 200Hz, 正常模式, 带宽 ~32Hz
        // 具体位定义需参考芯片手册，以下为典型配置值
        IMU660RA_WriteReg(IMU660RA_GYR_CONF,  0xA7);    // ODR=200Hz, BW=normal
        IMU660RA_WriteReg(IMU660RA_GYR_RANGE, (uint8_t)gyro_range);

        imu_cfg.gyro_range       = gyro_range;
        imu_cfg.gyro_sensitivity = gyro_sens;
        imu660ra_transition_factor[1] = gyro_sens;
    }

    printf("[IMU660RA] Init OK (ACC: %d, GYRO: %d)\r\n",
           (int)imu_cfg.acc_range, (int)imu_cfg.gyro_range);
    return 1;
}

/**
 * @brief  读取加速度计原始数据
 *
 * 从 IMU660RA_ACC_ADDRESS (0x0C) 开始连续读取 6 字节，
 * 依次为 X、Y、Z 三轴的低/高字节（小端序）。
 * 同时更新 imu660ra_acc_x/y/z 和 imu_data 中的原始值与换算值。
 */
void imu660ra_get_acc(void)
{
    uint8_t buf[6];

    IMU660RA_ReadMulti(IMU660RA_ACC_ADDRESS, buf, 6);

    // 小端序：buf[0]=低字节, buf[1]=高字节
    imu660ra_acc_x = (int16_t)((buf[1] << 8) | buf[0]);
    imu660ra_acc_y = (int16_t)((buf[3] << 8) | buf[2]);
    imu660ra_acc_z = (int16_t)((buf[5] << 8) | buf[4]);

    // 同步到 imu_data 结构体
    imu_data.ax_raw = imu660ra_acc_x;
    imu_data.ay_raw = imu660ra_acc_y;
    imu_data.az_raw = imu660ra_acc_z;

    // 换算为物理单位 (g)
    float factor = imu660ra_transition_factor[0];
    imu_data.ax = (float)imu660ra_acc_x / factor;
    imu_data.ay = (float)imu660ra_acc_y / factor;
    imu_data.az = (float)imu660ra_acc_z / factor;
}

/**
 * @brief  读取陀螺仪原始数据
 *
 * 从 IMU660RA_GYRO_ADDRESS (0x12) 开始连续读取 6 字节，
 * 依次为 X、Y、Z 三轴的低/高字节（小端序）。
 * 同时更新 imu660ra_gyro_x/y/z 和 imu_data 中的原始值与换算值。
 */
void imu660ra_get_gyro(void)
{
    uint8_t buf[6];

    IMU660RA_ReadMulti(IMU660RA_GYRO_ADDRESS, buf, 6);

    imu660ra_gyro_x = (int16_t)((buf[1] << 8) | buf[0]);
    imu660ra_gyro_y = (int16_t)((buf[3] << 8) | buf[2]);
    imu660ra_gyro_z = (int16_t)((buf[5] << 8) | buf[4]);

    // 同步到 imu_data 结构体
    imu_data.gx_raw = imu660ra_gyro_x;
    imu_data.gy_raw = imu660ra_gyro_y;
    imu_data.gz_raw = imu660ra_gyro_z;

    // 换算为物理单位 (°/s)
    float factor = imu660ra_transition_factor[1];
    imu_data.gx = (float)imu660ra_gyro_x / factor;
    imu_data.gy = (float)imu660ra_gyro_y / factor;
    imu_data.gz = (float)imu660ra_gyro_z / factor;
}

/******************************************************************************
 *                         辅助工具函数
 ******************************************************************************/

static float LimitFloat(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float AbsFloat(float x)
{
    return x >= 0.0f ? x : -x;
}

static void NormalizeQuaternion(void)
{
    float norm = sqrtf(att_q[0] * att_q[0] +
                       att_q[1] * att_q[1] +
                       att_q[2] * att_q[2] +
                       att_q[3] * att_q[3]);

    if (norm < 0.001f) {
        att_q[0] = 1.0f;
        att_q[1] = 0.0f;
        att_q[2] = 0.0f;
        att_q[3] = 0.0f;
        return;
    }

    norm = 1.0f / norm;
    att_q[0] *= norm;
    att_q[1] *= norm;
    att_q[2] *= norm;
    att_q[3] *= norm;
}

/**
 * @brief  四元数转欧拉角（ZYX 顺序）
 * @param  q[4]   输入四元数 (q0=qw, q1=qx, q2=qy, q3=qz)
 * @param  roll   输出横滚角（度）
 * @param  pitch  输出俯仰角（度）
 * @param  yaw    输出偏航角（度，0~360）
 */
void quat_to_euler(float q[4], float *roll, float *pitch, float *yaw)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // Roll (绕 X 轴)
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    // Pitch (绕 Y 轴) — 限制 asinf 入参范围防止数值溢出
    float sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = LimitFloat(sin_pitch, -1.0f, 1.0f);
    *pitch = asinf(sin_pitch) * RAD_TO_DEG;

    // Yaw (绕 Z 轴)
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

    // 归一化到 0~360 度
    if (*yaw < 0.0f) {
        *yaw += 360.0f;
    }
}

/******************************************************************************
 *                       Mahony 互补滤波姿态解算
 ******************************************************************************/

/**
 * @brief  Mahony 互补滤波姿态更新
 * @param  dt  距离上次更新的时间（秒），典型值 0.01（10ms 周期）
 *
 * 使用加速度计修正陀螺仪积分的 roll/pitch 漂移。
 * 6 轴 IMU 无磁力计，yaw 方向没有绝对参考，长期使用会有漂移。
 */
void IMU660RA_AttitudeUpdate(float dt)
{
    float gx, gy, gz;
    float ax, ay, az;
    float acc_norm;
    float q0, q1, q2, q3;
    float vx, vy, vz;
    float ex, ey, ez;
    float q_dot0, q_dot1, q_dot2, q_dot3;
    uint8_t is_static;

    if (dt <= 0.0f || dt > 0.05f) {
        return;   // dt 异常，跳过本次更新
    }

    // ---- 1. 读取陀螺仪数据 ----
    imu660ra_get_gyro();
    gx = imu_data.gx - gyro_bias[0];   // °/s
    gy = imu_data.gy - gyro_bias[1];
    gz = imu_data.gz - gyro_bias[2];

    // ---- 2. 读取加速度计数据并低通滤波 ----
    imu660ra_get_acc();

    acc_lpf[0] = 0.90f * acc_lpf[0] + 0.10f * imu_data.ax;
    acc_lpf[1] = 0.90f * acc_lpf[1] + 0.10f * imu_data.ay;
    acc_lpf[2] = 0.90f * acc_lpf[2] + 0.10f * imu_data.az;

    ax = acc_lpf[0];
    ay = acc_lpf[1];
    az = acc_lpf[2];

    acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    // ---- 3. 静止检测 ----
    // 条件：加速度模接近 1g 且三轴角速度都很小
    if ((acc_norm > 0.95f && acc_norm < 1.05f) &&
        (AbsFloat(gx) < 1.0f) &&
        (AbsFloat(gy) < 1.0f) &&
        (AbsFloat(gz) < 1.0f))
    {
        is_static = 1;
    }
    else
    {
        is_static = 0;
    }

    // ---- 4. 静止时在线估计陀螺仪零偏 ----
    if (is_static)
    {
        gyro_bias[0] = 0.9995f * gyro_bias[0] + 0.0005f * imu_data.gx;
        gyro_bias[1] = 0.9995f * gyro_bias[1] + 0.0005f * imu_data.gy;
        gyro_bias[2] = 0.9995f * gyro_bias[2] + 0.0005f * imu_data.gz;

        gx = imu_data.gx - gyro_bias[0];
        gy = imu_data.gy - gyro_bias[1];
        gz = imu_data.gz - gyro_bias[2];
    }

    // ---- 5. 小角速度死区处理 ----
    if (AbsFloat(gx) < 0.05f) gx = 0.0f;
    if (AbsFloat(gy) < 0.05f) gy = 0.0f;
    if (AbsFloat(gz) < 0.05f) gz = 0.0f;

    // °/s → rad/s
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    q0 = att_q[0];
    q1 = att_q[1];
    q2 = att_q[2];
    q3 = att_q[3];

    // ---- 6. 加速度修正（仅当加速度模接近 1g 时） ----
    if (acc_norm > 0.85f && acc_norm < 1.15f)
    {
        // 归一化加速度向量
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;

        // 当前四元数姿态下的重力方向估计
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 叉积 = 加速度测量值 × 估计重力方向 → 姿态误差
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        // PI 积分项
        integral_fb[0] += MAHONY_KI * ex * dt;
        integral_fb[1] += MAHONY_KI * ey * dt;
        integral_fb[2] += MAHONY_KI * ez * dt;

        // 积分限幅
        integral_fb[0] = LimitFloat(integral_fb[0], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        integral_fb[1] = LimitFloat(integral_fb[1], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        integral_fb[2] = LimitFloat(integral_fb[2], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        // PI 修正角速度
        gx += MAHONY_KP * ex + integral_fb[0];
        gy += MAHONY_KP * ey + integral_fb[1];
        gz += MAHONY_KP * ez + integral_fb[2];
    }

    // ---- 7. 四元数更新（一阶龙格-库塔） ----
    q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    q_dot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    q_dot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    att_q[0] += q_dot0 * dt;
    att_q[1] += q_dot1 * dt;
    att_q[2] += q_dot2 * dt;
    att_q[3] += q_dot3 * dt;

    // 归一化
    NormalizeQuaternion();

    // ---- 8. 输出到 imu_data ----
    imu_data.q[0] = att_q[0];
    imu_data.q[1] = att_q[1];
    imu_data.q[2] = att_q[2];
    imu_data.q[3] = att_q[3];

    quat_to_euler(att_q, &imu_data.roll, &imu_data.pitch, &imu_data.yaw);
}

/**
 * @brief  姿态解算初始化
 *
 * 采集陀螺仪零偏（800 次平均）和加速度初始姿态，
 * 以确定初始四元数和 roll/pitch 初值。
 * 调用此函数前请保持 IMU 静止。
 */
void IMU660RA_AttitudeInit(void)
{
    int i;
    float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    float ax, ay, az, norm;
    float roll, pitch, yaw;
    float cy, sy, cp, sp, cr, sr;

    HAL_Delay(200);   // 等待 IMU 稳定

    // ---- 1. 采集陀螺仪零偏（800 次） ----
    for (i = 0; i < 800; i++)
    {
        imu660ra_get_gyro();
        gx_sum += imu_data.gx;
        gy_sum += imu_data.gy;
        gz_sum += imu_data.gz;
        HAL_Delay(2);
    }

    gyro_bias[0] = gx_sum / 800.0f;
    gyro_bias[1] = gy_sum / 800.0f;
    gyro_bias[2] = gz_sum / 800.0f;

    // ---- 2. 加速度计初始化 Roll / Pitch（200 次） ----
    for (i = 0; i < 200; i++)
    {
        imu660ra_get_acc();
        ax_sum += imu_data.ax;
        ay_sum += imu_data.ay;
        az_sum += imu_data.az;
        HAL_Delay(2);
    }

    ax = ax_sum / 200.0f;
    ay = ay_sum / 200.0f;
    az = az_sum / 200.0f;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.01f) {
        ax = 0.0f; ay = 0.0f; az = 1.0f;
    } else {
        ax /= norm; ay /= norm; az /= norm;
    }

    // 初始化加速度低通滤波器状态
    acc_lpf[0] = ax;
    acc_lpf[1] = ay;
    acc_lpf[2] = az;

    // ---- 3. 计算初始欧拉角 ----
    roll  = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    yaw   = 0.0f;   // 6 轴 IMU 无磁力计，yaw 初始化为 0

    // ---- 4. 欧拉角转四元数 ----
    cy = cosf(yaw   * 0.5f);
    sy = sinf(yaw   * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);
    cr = cosf(roll  * 0.5f);
    sr = sinf(roll  * 0.5f);

    att_q[0] = cr * cp * cy + sr * sp * sy;
    att_q[1] = sr * cp * cy - cr * sp * sy;
    att_q[2] = cr * sp * cy + sr * cp * sy;
    att_q[3] = cr * cp * sy - sr * sp * cy;

    NormalizeQuaternion();

    // ---- 5. 清积分项并输出初始状态 ----
    integral_fb[0] = 0.0f;
    integral_fb[1] = 0.0f;
    integral_fb[2] = 0.0f;

    imu_data.q[0] = att_q[0];
    imu_data.q[1] = att_q[1];
    imu_data.q[2] = att_q[2];
    imu_data.q[3] = att_q[3];

    imu_data.roll  = roll  * RAD_TO_DEG;
    imu_data.pitch = pitch * RAD_TO_DEG;
    imu_data.yaw   = 0.0f;

    printf("[IMU660RA] AttitudeInit OK: roll=%.2f, pitch=%.2f, gyro_bias=(%.3f,%.3f,%.3f)\r\n",
           imu_data.roll, imu_data.pitch,
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}
