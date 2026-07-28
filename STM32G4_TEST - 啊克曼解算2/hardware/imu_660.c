//
// Created by 35037 on 2026/4/30.
//
#include "imu660.h"

#include <stdio.h>

#include "spi.h"
#include "string.h"
#include "math.h"

IMU660RC_ConfigType imu_cfg = {0};
IMU660RC_DataType  imu_data = {0};

/* 姿态解算专用变量 */
#define DEG_TO_RAD      0.017453292519943295f
#define RAD_TO_DEG      57.29577951308232f

#define MAHONY_KP       1.5f
#define MAHONY_KI       0.003f

#define INTEGRAL_LIMIT  0.15f

static float att_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float integral_fb[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

static float acc_lpf[3] = {0.0f, 0.0f, 1.0f};

//void Acc_Bias_Init(void)
//{
//    float sx = 0.0f;
//    float sy = 0.0f;
//    float sz = 0.0f;

//    for (int i = 0; i < 500; i++)
//    {
//        IMU660RC_ReadAcc();

//        sx += imu_data.ax;
//        sy += imu_data.ay;
//        sz += imu_data.az;

//        HAL_Delay(2);
//    }

//    acc_bias_x = sx / 500.0f;
//    acc_bias_y = sy / 500.0f;

//    /*
//       如果你把 az 修正成静止时 +1g，则这样：
//    */
//    acc_bias_z = sz / 500.0f - 1.0f;
//}


void IMU660RC_Init(void)
{
    uint8_t whoami = IMU660RC_ReadRegs(IMU660RC_CHIP_ID);
		printf("who_ami:%x\n",whoami);
    if (whoami != 0x70)
    {
        printf("IMU660RC_Init() 失败, WHO_AM_I = 0x%02X\r\n", whoami);
        return;
    }

    // 1. 软件复位 (逐飞方式：FUNC_CFG_ACCESS 写 0x04)
    IMU660RC_WriteRegs(IMU660RC_FUNC_CFG_ACCESS, 0x04);
    HAL_Delay(30);

    // 2. 启用块更新及地址自增
    IMU660RC_WriteRegs(IMU660RC_CTRL3, 0x44);

    // 3. 配置加速度计量程 (写入 CTRL8)
    //    同时设置 acc_sensitivity (原始值 / 灵敏度 =  g 值)
    //    可选 ±2g/±4g/±8g/±16g，这里默认用 ±16g
    {
        uint8_t acc_range_val = 0x03;   // 0x00:±2g  0x01:±4g  0x02:±8g  0x03:±16g
        float   acc_sens = 2049.18f;    // 对应 ±16g 的灵敏度
        // 如果想改量程，手动修改上面两行即可
        IMU660RC_WriteRegs(IMU660RC_CTRL8, acc_range_val);
        imu_cfg.acc_fs = acc_range_val;
        imu_cfg.acc_sensitivity = acc_sens;
    }

    // 4. 配置陀螺仪量程 (写入 CTRL6)
    //    可选 ±125/±250/±500/±1000/±2000/±4000 dps，这里默认用 ±2000dps
    {
        uint8_t gyro_range_val = 0x04;  // 0x00:±125  0x01:±250  0x02:±500  0x03:±1000  0x04:±2000  0x0C:±4000
        float   gyro_sens = 14.2857f;   // 对应 ±2000dps 的灵敏度
        IMU660RC_WriteRegs(IMU660RC_CTRL6, gyro_range_val);
        imu_cfg.gyro_fs = gyro_range_val;
        imu_cfg.gyro_sensitivity = gyro_sens;
    }

    // 5. 设置传感器工作模式与输出频率
    IMU660RC_WriteRegs(IMU660RC_CTRL1, 0x15);   // 加速度计：高精度模式，208 Hz (可根据需要调)
    IMU660RC_WriteRegs(IMU660RC_CTRL2, 0x18);   // 陀螺仪：  高精度模式，208 Hz

    // 6. 开启低通滤波 (LPF1 / LPF2)
    IMU660RC_WriteRegs(IMU660RC_CTRL7, 0x01);   // 使能陀螺仪 LPF1
    IMU660RC_WriteRegs(IMU660RC_CTRL9, 0x08);   // 使能加速度计 LPF2

    printf("IMU660RC_Init() 成功\r\n");
}

// void IMU660RC_Init_SFLP(void)
// {
//     // 基础初始化不变...
//     IMU660RC_Init();
//
//     // 进入嵌入功能配置
//     IMU660RC_WriteRegs(CTRL5_C, 0x80);            // FUNC_CFG_ACCESS = 0x80
//     IMU660RC_WriteRegs(PAGE_SEL, 0x80);        // PAGE_SEL = 0x80
//     IMU660RC_WriteRegs(PAGE_RW, 0x01);         // 先进入主嵌入页 (重要: 这里可能需要 0x01)
//     // 写入嵌入功能寄存器
//     IMU660RC_WriteRegs(EMB_FUNC_INIT_A, 0x01);// 复位 SFLP
//     HAL_Delay(1);
//     IMU660RC_WriteRegs(EMB_FUNC_CFG, 0x30);    // 0x30, 不是 0x20
//     IMU660RC_WriteRegs(EMB_FUNC_EN_A, 0x02);   // 0x02, 不是 0x01
//     IMU660RC_WriteRegs(SFLP_ODR, 0x5B);        // 120Hz (0x43 | (3<<3))
//
//     IMU660RC_WriteRegs(0x0E, 0x80);          // INT2_CTRL: 使能 INT2 输出数据就绪
//     IMU660RC_WriteRegs(CTRL4_C, 0x08);       // CTRL4_C: 使能 SFLP 数据就绪信号路由到 INT2
//     IMU660RC_WriteRegs(CTRL1_XL, 0x16);      // 加速度计 ODR 改为 120Hz (匹配 SFLP)
//     IMU660RC_WriteRegs(CTRL2_G,  0x16);
//     // 退出
//     IMU660RC_WriteRegs(PAGE_RW, 0x00);
//     IMU660RC_WriteRegs(PAGE_SEL, 0x00);
//     IMU660RC_WriteRegs(CTRL5_C, 0x00);
//     printf("IMU660RC_Init_SFLP() 成功\n");
// }

void IMU660RC_ReadAcc(void)
{
    uint8_t buf[6];
    IMU660RC_ReadMultiRegs(IMU660RC_OUTX_L_A, buf, 6);  // 使用正确的批量读函数
    imu_data.ax_raw = (int16_t)((buf[1]<<8) | buf[0]);
    imu_data.ay_raw = (int16_t)((buf[3]<<8) | buf[2]);
    imu_data.az_raw = (int16_t)((buf[5]<<8) | buf[4]);
    imu_data.ax = imu_data.ax_raw / imu_cfg.acc_sensitivity;
    imu_data.ay = imu_data.ay_raw / imu_cfg.acc_sensitivity;
    imu_data.az = imu_data.az_raw / imu_cfg.acc_sensitivity;


}
void IMU660RC_ReadGyro(void) {
    uint8_t buf[6];
    IMU660RC_ReadMultiRegs(IMU660RC_OUTX_L_G, buf, 6);
    imu_data.gx_raw = (int16_t)((buf[1]<<8) | buf[0]);
    imu_data.gy_raw = (int16_t)((buf[3]<<8) | buf[2]);
    imu_data.gz_raw = (int16_t)((buf[5]<<8) | buf[4]);
    imu_data.gx = imu_data.gx_raw / imu_cfg.gyro_sensitivity;
    imu_data.gy = imu_data.gy_raw / imu_cfg.gyro_sensitivity;
    imu_data.gz = imu_data.gz_raw / imu_cfg.gyro_sensitivity;
		//printf("%.2f  %.2f  %.2f  \n",imu_data.gx,imu_data.gy,imu_data.gz);
}
void IMU660RC_WriteRegs(uint8_t reg,uint8_t data)
{
    uint8_t tx[2] = {reg&0x7F,data};
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_Transmit(&hspi2,tx,2,1000);
    SPI_IMU660RC_CS_HIGH();
}
// void IMU660RC_ReadEuler(void) {
//     uint8_t buf[6];
//     IMU660RC_ReadMultiRegs(ANGLE_ROLL_L, buf, 6);
//     int16_t roll  = (int16_t)((buf[1]<<8) | buf[0]);
//     int16_t pitch = (int16_t)((buf[3]<<8) | buf[2]);
//     int16_t yaw   = (int16_t)((buf[5]<<8) | buf[4]);
//     imu_data.roll  = roll / 100.0f;
//     imu_data.pitch = pitch / 100.0f;
//     imu_data.yaw   = yaw / 100.0f;
// }

// void IMU660RC_ReadQuat(void) {
//     uint8_t i;
//     uint16_t buff[4];
//     uint8_t *buff_ptr = (uint8_t *)buff;
//
//     // 进入嵌入功能页
//     IMU660RC_WriteRegs(0x01, 0x80);      // FUNC_CFG_ACCESS
//     IMU660RC_WriteRegs(PAGE_RW, 0x20);   // 写 0x20
//     IMU660RC_WriteRegs(PAGE_SEL, 0x31);  // 写 0x31
//
//     for(i = 0; i < 4; i++) {
//         IMU660RC_WriteRegs(0x08, (uint8_t)(0x4C + i*2 + 0));
//         buff_ptr[i*2 + 0] = IMU660RC_ReadRegs(0x09);   // 低字节
//         IMU660RC_WriteRegs(0x08, (uint8_t)(0x4C + i*2 + 1));
//         buff_ptr[i*2 + 1] = IMU660RC_ReadRegs(0x09);   // 高字节
//     }
//
//     // 退出嵌入页面
//     IMU660RC_WriteRegs(PAGE_RW, 0x00);
//     IMU660RC_WriteRegs(0x01, 0x00);
//
//     // 半精度转浮点 + 归一化
//     float temp[4];
//     *(uint32_t*)(&temp[0]) = fp16_to_float(buff[0]);
//     *(uint32_t*)(&temp[1]) = fp16_to_float(buff[1]);
//     *(uint32_t*)(&temp[2]) = fp16_to_float(buff[2]);
//     *(uint32_t*)(&temp[3]) = fp16_to_float(buff[3]);
//
//     float n = sqrtf(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3]);
//     if (n > 0.001f) {
//         n = temp[3] < 0.0f ? -n : n;   // 逐飞库的特殊处理
//         imu_data.q[0] = temp[1] / n;
//         imu_data.q[1] = temp[2] / n;
//         imu_data.q[2] = temp[0] / n;
//         imu_data.q[3] = temp[3] / n;
//     } else {
//         imu_data.q[0] = 0.0f;
//         imu_data.q[1] = 0.0f;
//         imu_data.q[2] = 0.0f;
//         imu_data.q[3] = 1.0f;
//     }
//     //printf("buff: %04X %04X %04X %04X\n", buff[0], buff[1], buff[2], buff[3]);
// }
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
    float norm;

    norm = sqrtf(att_q[0] * att_q[0] +
                 att_q[1] * att_q[1] +
                 att_q[2] * att_q[2] +
                 att_q[3] * att_q[3]);

    if (norm < 0.001f)
    {
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

uint8_t IMU660RC_ReadRegs(uint8_t reg)
{
    uint8_t tx[2] = {reg|0x80,0xFF};//tx[0]1号位为1是读寄存器后七位为地址 tx[1]为填充也可以只用一个
    uint8_t rx[2] = {0,0};
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2,tx,rx,2,1000);
    SPI_IMU660RC_CS_HIGH();
    return rx[1];
}

uint16_t IMU660RC_ReadReg16b(uint8_t reg_low)//输入的是低位地址,+1为高位
{
    uint8_t data_l=IMU660RC_ReadRegs(reg_low);
    uint8_t data_h=IMU660RC_ReadRegs(reg_low+1);
    return (uint16_t)((data_h<<8)|data_l);
}
void IMU660RC_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t tx[len + 1];
    tx[0] = reg | 0x80;
    for (int i = 0; i < len; i++) tx[i + 1] = 0xFF;   // dummy 字节

    uint8_t rx[len + 1];
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len + 1, 1000);
    SPI_IMU660RC_CS_HIGH();

    memcpy(buf, &rx[1], len);   // 忽略第一个接收字节
}

void quat_to_euler(float q[4], float *roll, float *pitch, float *yaw)
{
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    float sin_pitch;

    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                   1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    sin_pitch = LimitFloat(sin_pitch, -1.0f, 1.0f);

    *pitch = asinf(sin_pitch) * RAD_TO_DEG;

    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                  1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

    if (*yaw < 0.0f)
    {
        *yaw += 360.0f;
    }
}

static uint32_t fp16_to_float(uint16_t h)
{
    uint32_t f_sgn;
    uint16_t h_exp;
    uint32_t f_exp;
    uint32_t f_sig;

    h_exp = (h & 0x7c00u);
    f_sgn = ((uint32_t)h & 0x8000u) << 16;
    switch (h_exp)
    {
    case 0x0000u:   // 0 or subnormal
        {
            uint16_t h_sig = (h & 0x03ffu);
            // Signed zero
            if (h_sig == 0)
            {
                return f_sgn;
            }
            // Subnormal
            h_sig <<= 1;
            while ((h_sig & 0x0400u) == 0)
            {
                h_sig <<= 1;
                h_exp++;
            }
            f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
            f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
            return f_sgn + f_exp + f_sig;
        }
    case 0x7c00u: // inf or NaN
        {
            // All-ones exponent and a copy of the significand
            return f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
        }
    default: // normalized
        {
            // Just need to adjust the exponent and shift
            return f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
        }
    }
}
/*
 * @brief Mahony 互补滤波姿态更新
 * @param dt 距离上次更新的时间（秒），若不传则内部用 HAL_GetTick 计算
 */
void IMU660RC_AttitudeUpdate(float dt)
{
    float gx, gy, gz;
    float ax, ay, az;
    float acc_norm;

    float q0, q1, q2, q3;
    float vx, vy, vz;
    float ex, ey, ez;

    float q_dot0, q_dot1, q_dot2, q_dot3;

    uint8_t is_static;

    if (dt <= 0.0f || dt > 0.05f)
    {
        return;
    }

    /* 1. 读取陀螺仪 */
    IMU660RC_ReadGyro();

    gx = imu_data.gx - gyro_bias[0];   // dps
    gy = imu_data.gy - gyro_bias[1];
    gz = imu_data.gz - gyro_bias[2];

    /* 2. 读取加速度 */
    IMU660RC_ReadAcc();

    /*
       加速度低通滤波。
       0.90 越大越稳但响应越慢；
       0.80 响应快一点但更抖。
    */
    acc_lpf[0] = 0.90f * acc_lpf[0] + 0.10f * imu_data.ax;
    acc_lpf[1] = 0.90f * acc_lpf[1] + 0.10f * imu_data.ay;
    acc_lpf[2] = 0.90f * acc_lpf[2] + 0.10f * imu_data.az;

    ax = acc_lpf[0];
    ay = acc_lpf[1];
    az = acc_lpf[2];

    acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    /*
       静止判断：
       1. 加速度模长接近 1g
       2. 三轴角速度都很小
    */
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

    /*
       静止时在线修正陀螺仪零偏。
       注意：这里只能慢慢修，不能修太快。
    */
    if (is_static)
    {
        gyro_bias[0] = 0.9995f * gyro_bias[0] + 0.0005f * imu_data.gx;
        gyro_bias[1] = 0.9995f * gyro_bias[1] + 0.0005f * imu_data.gy;
        gyro_bias[2] = 0.9995f * gyro_bias[2] + 0.0005f * imu_data.gz;

        gx = imu_data.gx - gyro_bias[0];
        gy = imu_data.gy - gyro_bias[1];
        gz = imu_data.gz - gyro_bias[2];
    }

    /*
       小角速度死区。
       防止静止时微小噪声不断积分。
    */
    if (AbsFloat(gx) < 0.05f) gx = 0.0f;
    if (AbsFloat(gy) < 0.05f) gy = 0.0f;
    if (AbsFloat(gz) < 0.05f) gz = 0.0f;

    /* dps -> rad/s */
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;

    q0 = att_q[0];
    q1 = att_q[1];
    q2 = att_q[2];
    q3 = att_q[3];

    /*
       加速度模长接近 1g 时才用它修正姿态。
       小车加速、震动、撞击时，不要强行相信加速度。
    */
    if (acc_norm > 0.85f && acc_norm < 1.15f)
    {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;

        /* 当前姿态估计出来的重力方向 */
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* 叉乘误差 */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        /*
           积分修正。
           注意：6轴 IMU 没有磁力计，ez 对 yaw 的绝对修正能力很弱。
        */
        integral_fb[0] += MAHONY_KI * ex * dt;
        integral_fb[1] += MAHONY_KI * ey * dt;
        integral_fb[2] += MAHONY_KI * ez * dt;

        integral_fb[0] = LimitFloat(integral_fb[0], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        integral_fb[1] = LimitFloat(integral_fb[1], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        integral_fb[2] = LimitFloat(integral_fb[2], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        gx += MAHONY_KP * ex + integral_fb[0];
        gy += MAHONY_KP * ey + integral_fb[1];
        gz += MAHONY_KP * ez + integral_fb[2];
    }

    /* 四元数积分 */
    q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    q_dot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    q_dot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    att_q[0] += q_dot0 * dt;
    att_q[1] += q_dot1 * dt;
    att_q[2] += q_dot2 * dt;
    att_q[3] += q_dot3 * dt;

    NormalizeQuaternion();

    imu_data.q[0] = att_q[0];
    imu_data.q[1] = att_q[1];
    imu_data.q[2] = att_q[2];
    imu_data.q[3] = att_q[3];

    quat_to_euler(att_q, &imu_data.roll, &imu_data.pitch, &imu_data.yaw);
}


/**
 * @brief Mahony 互补滤波姿态更新
 * @param dt 距离上次更新的时间间隔（秒）
 */
/**
 * @brief 互补滤波姿态初始化（获取零偏并设置初始四元数）
 */
void IMU660RC_AttitudeInit(void)
{
    int i;

    float gx_sum = 0.0f;
    float gy_sum = 0.0f;
    float gz_sum = 0.0f;

    float ax_sum = 0.0f;
    float ay_sum = 0.0f;
    float az_sum = 0.0f;

    float ax, ay, az;
    float norm;

    float roll;
    float pitch;
    float yaw;

    float cy, sy;
    float cp, sp;
    float cr, sr;

    HAL_Delay(200);

    /* 陀螺仪零偏 */
    for (i = 0; i < 800; i++)
    {
        IMU660RC_ReadGyro();

        gx_sum += imu_data.gx;
        gy_sum += imu_data.gy;
        gz_sum += imu_data.gz;

        HAL_Delay(2);
    }

    gyro_bias[0] = gx_sum / 800.0f;
    gyro_bias[1] = gy_sum / 800.0f;
    gyro_bias[2] = gz_sum / 800.0f;

    /* 加速度初始化 roll / pitch */
    for (i = 0; i < 200; i++)
    {
        IMU660RC_ReadAcc();

        ax_sum += imu_data.ax;
        ay_sum += imu_data.ay;
        az_sum += imu_data.az;

        HAL_Delay(2);
    }

    ax = ax_sum / 200.0f;
    ay = ay_sum / 200.0f;
    az = az_sum / 200.0f;

    norm = sqrtf(ax * ax + ay * ay + az * az);

    if (norm < 0.01f)
    {
        ax = 0.0f;
        ay = 0.0f;
        az = 1.0f;
    }
    else
    {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    acc_lpf[0] = ax;
    acc_lpf[1] = ay;
    acc_lpf[2] = az;

    roll  = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    yaw   = 0.0f;

    cy = cosf(yaw * 0.5f);
    sy = sinf(yaw * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);
    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);

    att_q[0] = cr * cp * cy + sr * sp * sy;
    att_q[1] = sr * cp * cy - cr * sp * sy;
    att_q[2] = cr * sp * cy + sr * cp * sy;
    att_q[3] = cr * cp * sy - sr * sp * cy;

    NormalizeQuaternion();

    integral_fb[0] = 0.0f;
    integral_fb[1] = 0.0f;
    integral_fb[2] = 0.0f;

    imu_data.q[0] = att_q[0];
    imu_data.q[1] = att_q[1];
    imu_data.q[2] = att_q[2];
    imu_data.q[3] = att_q[3];

    imu_data.roll  = roll * RAD_TO_DEG;
    imu_data.pitch = pitch * RAD_TO_DEG;
    imu_data.yaw   = 0.0f;
}
