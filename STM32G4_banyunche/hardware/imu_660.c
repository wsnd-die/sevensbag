//
// Created by 35037 on 2026/4/30.
//
#include "Common_used.h"

IMU660RC_ConfigType imu_cfg = {0};
IMU660RC_DataType  imu_data = {0};

/* ��̬����ר�ñ��� */
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
//       ������ az �����ɾ�ֹʱ +1g����������
//    */
//    acc_bias_z = sz / 500.0f - 1.0f;
//}


void IMU660RC_Init(void)
{
    uint8_t whoami = IMU660RC_ReadRegs(IMU660RC_CHIP_ID);
		printf("who_ami:%x\n",whoami);
    if (whoami != 0x70)
    {
        printf("IMU660RC_Init() failed, WHO_AM_I = 0x%02X\r\n", whoami);
        return;
    }

    // 1. ������λ (��ɷ�ʽ��FUNC_CFG_ACCESS д 0x04)
    IMU660RC_WriteRegs(IMU660RC_FUNC_CFG_ACCESS, 0x04);
    HAL_Delay(30);

    // 2. ���ÿ���¼���ַ����
    IMU660RC_WriteRegs(IMU660RC_CTRL3, 0x44);

    // 3. ���ü��ٶȼ����� (д�� CTRL8)
    //    ͬʱ���� acc_sensitivity (ԭʼֵ / ������ =  g ֵ)
    //    ��ѡ ��2g/��4g/��8g/��16g������Ĭ���� ��16g
    {
        uint8_t acc_range_val = 0x03;   // 0x00:��2g  0x01:��4g  0x02:��8g  0x03:��16g
        float   acc_sens = 2049.18f;    // ��Ӧ ��16g ��������
        // ���������̣��ֶ��޸��������м���
        IMU660RC_WriteRegs(IMU660RC_CTRL8, acc_range_val);
        imu_cfg.acc_fs = acc_range_val;
        imu_cfg.acc_sensitivity = acc_sens;
    }

    // 4. �������������� (д�� CTRL6)
    //    ��ѡ ��125/��250/��500/��1000/��2000/��4000 dps������Ĭ���� ��2000dps
    {
        uint8_t gyro_range_val = 0x04;  // 0x00:��125  0x01:��250  0x02:��500  0x03:��1000  0x04:��2000  0x0C:��4000
        float   gyro_sens = 14.2857f;   // ��Ӧ ��2000dps ��������
        IMU660RC_WriteRegs(IMU660RC_CTRL6, gyro_range_val);
        imu_cfg.gyro_fs = gyro_range_val;
        imu_cfg.gyro_sensitivity = gyro_sens;
    }

    // 5. ���ô���������ģʽ�����Ƶ��
    IMU660RC_WriteRegs(IMU660RC_CTRL1, 0x15);   // ���ٶȼƣ��߾���ģʽ��208 Hz (�ɸ�����Ҫ��)
    IMU660RC_WriteRegs(IMU660RC_CTRL2, 0x18);   // �����ǣ�  �߾���ģʽ��208 Hz

    // 6. ������ͨ�˲� (LPF1 / LPF2)
    IMU660RC_WriteRegs(IMU660RC_CTRL7, 0x01);   // ʹ�������� LPF1
    IMU660RC_WriteRegs(IMU660RC_CTRL9, 0x08);   // ʹ�ܼ��ٶȼ� LPF2

    printf("IMU660RC_Init() ok\r\n");
}

// void IMU660RC_Init_SFLP(void)
// {
//     // ������ʼ������...
//     IMU660RC_Init();
//
//     // ����Ƕ�빦������
//     IMU660RC_WriteRegs(CTRL5_C, 0x80);            // FUNC_CFG_ACCESS = 0x80
//     IMU660RC_WriteRegs(PAGE_SEL, 0x80);        // PAGE_SEL = 0x80
//     IMU660RC_WriteRegs(PAGE_RW, 0x01);         // �Ƚ�����Ƕ��ҳ (��Ҫ: ���������Ҫ 0x01)
//     // д��Ƕ�빦�ܼĴ���
//     IMU660RC_WriteRegs(EMB_FUNC_INIT_A, 0x01);// ��λ SFLP
//     HAL_Delay(1);
//     IMU660RC_WriteRegs(EMB_FUNC_CFG, 0x30);    // 0x30, ���� 0x20
//     IMU660RC_WriteRegs(EMB_FUNC_EN_A, 0x02);   // 0x02, ���� 0x01
//     IMU660RC_WriteRegs(SFLP_ODR, 0x5B);        // 120Hz (0x43 | (3<<3))
//
//     IMU660RC_WriteRegs(0x0E, 0x80);          // INT2_CTRL: ʹ�� INT2 ������ݾ���
//     IMU660RC_WriteRegs(CTRL4_C, 0x08);       // CTRL4_C: ʹ�� SFLP ���ݾ����ź�·�ɵ� INT2
//     IMU660RC_WriteRegs(CTRL1_XL, 0x16);      // ���ٶȼ� ODR ��Ϊ 120Hz (ƥ�� SFLP)
//     IMU660RC_WriteRegs(CTRL2_G,  0x16);
//     // �˳�
//     IMU660RC_WriteRegs(PAGE_RW, 0x00);
//     IMU660RC_WriteRegs(PAGE_SEL, 0x00);
//     IMU660RC_WriteRegs(CTRL5_C, 0x00);
//     printf("IMU660RC_Init_SFLP() �ɹ�\n");
// }

void IMU660RC_ReadAcc(void)
{
    uint8_t buf[6];
    IMU660RC_ReadMultiRegs(IMU660RC_OUTX_L_A, buf, 6);  // ʹ����ȷ������������
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
//     // ����Ƕ�빦��ҳ
//     IMU660RC_WriteRegs(0x01, 0x80);      // FUNC_CFG_ACCESS
//     IMU660RC_WriteRegs(PAGE_RW, 0x20);   // д 0x20
//     IMU660RC_WriteRegs(PAGE_SEL, 0x31);  // д 0x31
//
//     for(i = 0; i < 4; i++) {
//         IMU660RC_WriteRegs(0x08, (uint8_t)(0x4C + i*2 + 0));
//         buff_ptr[i*2 + 0] = IMU660RC_ReadRegs(0x09);   // ���ֽ�
//         IMU660RC_WriteRegs(0x08, (uint8_t)(0x4C + i*2 + 1));
//         buff_ptr[i*2 + 1] = IMU660RC_ReadRegs(0x09);   // ���ֽ�
//     }
//
//     // �˳�Ƕ��ҳ��
//     IMU660RC_WriteRegs(PAGE_RW, 0x00);
//     IMU660RC_WriteRegs(0x01, 0x00);
//
//     // �뾫��ת���� + ��һ��
//     float temp[4];
//     *(uint32_t*)(&temp[0]) = fp16_to_float(buff[0]);
//     *(uint32_t*)(&temp[1]) = fp16_to_float(buff[1]);
//     *(uint32_t*)(&temp[2]) = fp16_to_float(buff[2]);
//     *(uint32_t*)(&temp[3]) = fp16_to_float(buff[3]);
//
//     float n = sqrtf(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3]);
//     if (n > 0.001f) {
//         n = temp[3] < 0.0f ? -n : n;   // ��ɿ�����⴦��
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
    uint8_t tx[2] = {reg|0x80,0xFF};//tx[0]1��λΪ1�Ƕ��Ĵ�������λΪ��ַ tx[1]Ϊ���Ҳ����ֻ��һ��
    uint8_t rx[2] = {0,0};
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2,tx,rx,2,1000);
    SPI_IMU660RC_CS_HIGH();
    return rx[1];
}

uint16_t IMU660RC_ReadReg16b(uint8_t reg_low)//������ǵ�λ��ַ,+1Ϊ��λ
{
    uint8_t data_l=IMU660RC_ReadRegs(reg_low);
    uint8_t data_h=IMU660RC_ReadRegs(reg_low+1);
    return (uint16_t)((data_h<<8)|data_l);
}
void IMU660RC_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t tx[len + 1];
    tx[0] = reg | 0x80;
    for (int i = 0; i < len; i++) tx[i + 1] = 0xFF;   // dummy �ֽ�

    uint8_t rx[len + 1];
    SPI_IMU660RC_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, len + 1, 1000);
    SPI_IMU660RC_CS_HIGH();

    memcpy(buf, &rx[1], len);   // ���Ե�һ�������ֽ�
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
 * @brief Mahony �����˲���̬����
 * @param dt �����ϴθ��µ�ʱ�䣨�룩�����������ڲ��� HAL_GetTick ����
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

    /* 1. ��ȡ������ */
    IMU660RC_ReadGyro();

    gx = imu_data.gx - gyro_bias[0];   // dps
    gy = imu_data.gy - gyro_bias[1];
    gz = imu_data.gz - gyro_bias[2];

    /* 2. ��ȡ���ٶ� */
    IMU660RC_ReadAcc();

    /*
       ���ٶȵ�ͨ�˲���
       0.90 Խ��Խ�ȵ���ӦԽ����
       0.80 ��Ӧ��һ�㵫������
    */
    acc_lpf[0] = 0.90f * acc_lpf[0] + 0.10f * imu_data.ax;
    acc_lpf[1] = 0.90f * acc_lpf[1] + 0.10f * imu_data.ay;
    acc_lpf[2] = 0.90f * acc_lpf[2] + 0.10f * imu_data.az;

    ax = acc_lpf[0];
    ay = acc_lpf[1];
    az = acc_lpf[2];

    acc_norm = sqrtf(ax * ax + ay * ay + az * az);

    /*
       ��ֹ�жϣ�
       1. ���ٶ�ģ���ӽ� 1g
       2. ������ٶȶ���С
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
       ��ֹʱ����������������ƫ��
       ע�⣺����ֻ�������ޣ�������̫�졣
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
       С���ٶ�������
       ��ֹ��ֹʱ΢С�������ϻ��֡�
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
       ���ٶ�ģ���ӽ� 1g ʱ������������̬��
       С�����١��𶯡�ײ��ʱ����Ҫǿ�����ż��ٶȡ�
    */
    if (acc_norm > 0.85f && acc_norm < 1.15f)
    {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;

        /* ��ǰ��̬���Ƴ������������� */
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        /* ������ */
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        /*
           ����������
           ע�⣺6�� IMU û�д����ƣ�ez �� yaw �ľ�����������������
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

    /* ��Ԫ������ */
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
 * @brief Mahony �����˲���̬����
 * @param dt �����ϴθ��µ�ʱ�������룩
 */
/**
 * @brief �����˲���̬��ʼ������ȡ��ƫ�����ó�ʼ��Ԫ����
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

    /* ��������ƫ */
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

    /* ���ٶȳ�ʼ�� roll / pitch */
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
