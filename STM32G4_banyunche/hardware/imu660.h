/**
 * @file    imu660.h
 * @brief   IMU660RA 驱动头文件（STM32G4 适配版）
 *
 * 基于逐飞 TC264 开源库 IMU660RA 驱动，移植适配到 STM32G4 平台。
 * 原始版权声明见本文件末尾注释。
 *
 * 硬件连接（STM32G4）：
 *   SCL/SPC  → PB13 (SPI2_SCK)
 *   SDA/MOSI → PB15 (SPI2_MOSI)
 *   SA0/MISO → PB14 (SPI2_MISO)
 *   CS       → PB12 (SPI2_CS, 软件控制)
 *   VCC      → 3.3V
 *   GND      → GND
 *   其余引脚悬空
 */

#ifndef _IMU660RA_H_
#define _IMU660RA_H_

#include "main.h"

// ==================================================== 引脚定义 ====================================================
#define IMU660RA_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define IMU660RA_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

// ==================================================== SPI 协议定义 ====================================================
#define IMU660RA_SPI_W              (0x00)     // SPI 写标志
#define IMU660RA_SPI_R              (0x80)     // SPI 读标志（bit7 = 1）
#define IMU660RA_DEV_ADDR           (0x69)     // SA0 上拉：0x69；SA0 接地：0x68（模块默认上拉）

// ==================================================== 寄存器地址定义 ====================================================
#define IMU660RA_CHIP_ID            (0x00)     // 芯片 ID 寄存器（期望值 0x69）
#define IMU660RA_PWR_CONF           (0x7C)     // 电源配置
#define IMU660RA_PWR_CTRL           (0x7D)     // 电源控制
#define IMU660RA_INIT_CTRL          (0x59)     // 初始化控制
#define IMU660RA_INIT_DATA          (0x5E)     // 初始化数据
#define IMU660RA_INT_STA            (0x21)     // 中断状态
#define IMU660RA_ACC_ADDRESS        (0x0C)     // 加速度计数据起始地址（6 字节）
#define IMU660RA_GYRO_ADDRESS       (0x12)     // 陀螺仪数据起始地址（6 字节）
#define IMU660RA_ACC_CONF           (0x40)     // 加速度计配置
#define IMU660RA_ACC_RANGE          (0x41)     // 加速度计量程
#define IMU660RA_GYR_CONF           (0x42)     // 陀螺仪配置
#define IMU660RA_GYR_RANGE          (0x43)     // 陀螺仪量程

// ==================================================== 超时定义 ====================================================
#define IMU660RA_TIMEOUT_COUNT      (0x00FF)

// ==================================================== 量程枚举 ====================================================
// 加速度计量程
//   ACC = Accelerometer 加速度计
//   SGN = signum 带符号数（正负范围）
//   G   = g 重力加速度（≈ 9.80 m/s²）
typedef enum {
    IMU660RA_ACC_SGN_2G  = 0x00,    // ±2G
    IMU660RA_ACC_SGN_4G  = 0x01,    // ±4G
    IMU660RA_ACC_SGN_8G  = 0x02,    // ±8G
    IMU660RA_ACC_SGN_16G = 0x03,    // ±16G
} imu660ra_acc_range_t;

// 陀螺仪量程
//   GYRO = Gyroscope 陀螺仪
//   DPS  = Degree Per Second 角速度单位 °/s
typedef enum {
    IMU660RA_GYRO_SGN_125DPS  = 0x00,   // ±125°/s
    IMU660RA_GYRO_SGN_250DPS  = 0x01,   // ±250°/s
    IMU660RA_GYRO_SGN_500DPS  = 0x02,   // ±500°/s
    IMU660RA_GYRO_SGN_1000DPS = 0x03,   // ±1000°/s
    IMU660RA_GYRO_SGN_2000DPS = 0x04,   // ±2000°/s
} imu660ra_gyro_range_t;

// ==================================================== 默认量程 ====================================================
#define IMU660RA_ACC_SAMPLE_DEFAULT     (IMU660RA_ACC_SGN_8G)
#define IMU660RA_GYRO_SAMPLE_DEFAULT    (IMU660RA_GYRO_SGN_2000DPS)

// ==================================================== 灵敏度(转换系数)定义 ====================================================
// 加速度计灵敏度系数（单位：LSB/g），数值随量程变化：
//   ±2G:  16384 LSB/g
//   ±4G:   8192 LSB/g
//   ±8G:   4096 LSB/g
//   ±16G:  2048 LSB/g
// 陀螺仪灵敏度系数（单位：LSB/(°/s)），数值随量程变化：
//   ±125dps:   262.144 LSB/(°/s)
//   ±250dps:   131.072 LSB/(°/s)
//   ±500dps:    65.536 LSB/(°/s)
//   ±1000dps:   32.768 LSB/(°/s)
//   ±2000dps:   16.384 LSB/(°/s)
// 以上为标准 IMU 的典型值，实际使用时需根据 IMU660RA 芯片手册校准。

// ==================================================== 数据结构 ====================================================
// 配置结构体
typedef struct {
    imu660ra_acc_range_t  acc_range;          // 当前加速度计量程
    imu660ra_gyro_range_t gyro_range;         // 当前陀螺仪量程
    float                  acc_sensitivity;    // 加速度灵敏度系数 (LSB/g)
    float                  gyro_sensitivity;   // 陀螺仪灵敏度系数 (LSB/(°/s))
} IMU660RA_ConfigType;

// 数据存储结构体
typedef struct {
    int16_t ax_raw, ay_raw, az_raw;    // 加速度计原始值
    int16_t gx_raw, gy_raw, gz_raw;    // 陀螺仪原始值
    float   ax, ay, az;                // 加速度换算值 (g)
    float   gx, gy, gz;                // 角速度换算值 (°/s)
    float   roll, pitch, yaw;          // 欧拉角（度）
    float   q[4];                      // 四元数
} IMU660RA_DataType;

// ==================================================== 全局变量声明 ====================================================
extern IMU660RA_ConfigType imu_cfg;
extern IMU660RA_DataType   imu_data;

// 逐飞风格全局变量（直接存储原始数据）
extern int16_t imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;
extern int16_t imu660ra_acc_x,  imu660ra_acc_y,  imu660ra_acc_z;
extern float   imu660ra_transition_factor[2];   // [0] = acc系数, [1] = gyro系数

// ==================================================== 数据转换宏 ====================================================
// 将加速度计原始数据转换为实际物理值（g）
//   示例：float ax_g = imu660ra_acc_transition(imu660ra_acc_x);
#define imu660ra_acc_transition(acc_value)   ((float)(acc_value) / imu660ra_transition_factor[0])

// 将陀螺仪原始数据转换为实际物理值（°/s）
//   示例：float gx_dps = imu660ra_gyro_transition(imu660ra_gyro_x);
#define imu660ra_gyro_transition(gyro_value) ((float)(gyro_value) / imu660ra_transition_factor[1])

// ==================================================== 函数声明 ====================================================

// ---------- IMU660RA 基础驱动（逐飞风格） ----------
void    imu660ra_get_acc(void);                           // 读取加速度计原始数据
void    imu660ra_get_gyro(void);                          // 读取陀螺仪原始数据
uint8_t imu660ra_init(void);                              // 初始化 IMU660RA，返回 1=成功 0=失败

// ---------- SPI 底层通信 ----------
uint8_t IMU660RA_ReadReg(uint8_t reg);                    // 读单个寄存器
void    IMU660RA_WriteReg(uint8_t reg, uint8_t data);     // 写单个寄存器
uint16_t IMU660RA_ReadReg16b(uint8_t reg_low);            // 读 16 位寄存器对
void    IMU660RA_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len); // 批量读寄存器

// ---------- 姿态解算（Mahony 互补滤波） ----------
void quat_to_euler(float q[4], float *roll, float *pitch, float *yaw);
void IMU660RA_AttitudeUpdate(float dt);
void IMU660RA_AttitudeInit(void);

#endif /* _IMU660RA_H_ */

/*********************************************************************************************************************
* 原始版权声明（源自 逐飞科技 TC264 开源库）：
*
* TC264 Opensource Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件内容改编自 TC264 开源库中的 zf_device_imu660ra 模块
*
* TC264 开源库 是免费软件，使用 GPL3.0 开源许可证协议
* 更多细节请参见 GPL（<https://www.gnu.org/licenses/>）
********************************************************************************************************************/
