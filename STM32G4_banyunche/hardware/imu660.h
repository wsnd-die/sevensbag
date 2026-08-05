#ifndef INC_33333_SPI_IMU660RC_H
#define INC_33333_SPI_IMU660RC_H

#include "main.h"

// ----------------------- ���Ŷ��� -----------------------
#define SPI_IMU660RC_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define SPI_IMU660RC_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define SPI_IMU660RC_PORT       GPIOB
#define SPI_IMU660RC_SCLK_PIN   GPIO_PIN_13
#define SPI_IMU660RC_MISO_PIN   GPIO_PIN_14
#define SPI_IMU660RC_MOSI_PIN   GPIO_PIN_15

// ----------------------- �Ĵ�����ַ -----------------------
#define IMU660RC_FUNC_CFG_ACCESS    ( 0x01 )
#define IMU660RC_INT2_CTRL          ( 0x0E )
#define IMU660RC_CHIP_ID            ( 0x0F )
#define IMU660RC_CTRL1              ( 0x10 )
#define IMU660RC_CTRL2              ( 0x11 )
#define IMU660RC_CTRL3              ( 0x12 )
#define IMU660RC_CTRL4              ( 0x13 )
#define IMU660RC_CTRL5              ( 0x14 )
#define IMU660RC_CTRL6              ( 0x15 )
#define IMU660RC_CTRL7              ( 0x16 )
#define IMU660RC_CTRL8              ( 0x17 )
#define IMU660RC_CTRL9              ( 0x18 )
#define IMU660RC_CTRL10             ( 0x19 )
#define IMU660RC_CTRL_STATUS        ( 0x1A )
#define IMU660RC_STATUS_REG         ( 0x1E )
#define IMU660RC_OUT_TEMP_L         ( 0x20 )
#define IMU660RC_OUT_TEMP_H         ( 0x21 )
#define IMU660RC_OUTX_L_G           ( 0x22 )
#define IMU660RC_OUTX_H_G           ( 0x23 )
#define IMU660RC_OUTY_L_G           ( 0x24 )
#define IMU660RC_OUTY_H_G           ( 0x25 )
#define IMU660RC_OUTZ_L_G           ( 0x26 )
#define IMU660RC_OUTZ_H_G           ( 0x27 )
#define IMU660RC_OUTX_L_A           ( 0x28 )
#define IMU660RC_OUTX_H_A           ( 0x29 )
#define IMU660RC_OUTY_L_A           ( 0x2A )
#define IMU660RC_OUTY_H_A           ( 0x2B )
#define IMU660RC_OUTZ_L_A           ( 0x2C )
#define IMU660RC_OUTZ_H_A           ( 0x2D )

#define IMU660RC_PAGE_SEL           ( 0x02 )
#define IMU660RC_EMB_FUNC_EN_A      ( 0x04 )
#define IMU660RC_PAGE_RW            ( 0x17 )
#define IMU660RC_SFLP_ODR           ( 0x5E )
#define IMU660RC_EMB_FUNC_CFG       ( 0x63 )

// ----------------------- ���̺궨�� -----------------------
// ���ٶȼ����̣�CTRL1_XL �� FS_XL[1:0]��
#define ACCEL_FS_2G         0x00
#define ACCEL_FS_4G         0x02
#define ACCEL_FS_8G         0x03
#define ACCEL_FS_16G        0x01

// ���������̣�CTRL6_C �� FS_G[3:0]��λ�� bit7~4��
#define GYRO_FS_125DPS      0x00
#define GYRO_FS_250DPS      0x10    // 0001 0000
#define GYRO_FS_500DPS      0x20
#define GYRO_FS_1000DPS     0x30
#define GYRO_FS_2000DPS     0x40
#define GYRO_FS_4000DPS     0xC0    // ����ֵ 1100 0000

// Ĭ������
#define IMU660RC_ACC_FS_DEFAULT   ACCEL_FS_16G
#define IMU660RC_GYRO_FS_DEFAULT  GYRO_FS_4000DPS

// ----------------------- ���ݽṹ -----------------------
typedef struct {
    uint8_t acc_fs;     // ��ǰ���ٶȼ���������ֵ
    uint8_t gyro_fs;    // ��ǰ��������������ֵ
    float   acc_sensitivity;   // ���ٶ�ϵ����4096 ��Ӧ ��8g��
    float   gyro_sensitivity;  // ������ϵ����16.384 ��Ӧ ��2000dps��
} IMU660RC_ConfigType;

typedef struct {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    float   ax, ay, az;      // ���ٶ� g ֵ
    float   gx, gy, gz;      // ���ٶ� dps
    float   roll, pitch, yaw;// ŷ���ǣ��ȣ�
    float   q[4];            // ��Ԫ��
} IMU660RC_DataType;


extern IMU660RC_ConfigType imu_cfg;
extern IMU660RC_DataType  imu_data;

// ----------------------- �������� -----------------------
uint8_t IMU660RC_ReadRegs(uint8_t reg);
void    IMU660RC_WriteRegs(uint8_t reg, uint8_t data);
uint16_t    IMU660RC_ReadReg16b(uint8_t reg_low);
void IMU660RC_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint8_t len);

void    IMU660RC_Init(void);           // ��ͨ��ʼ��
void    IMU660RC_Init_SFLP(void);      // ����������̬����
void    IMU660RC_ReadAcc(void);
void    IMU660RC_ReadGyro(void);
//void    IMU660RC_ReadEuler(void);      // �� SFLP ��ȡŷ����
//void    IMU660RC_ReadQuat(void);       // ��ȡ��Ԫ��
void quat_to_euler(float q[4], float *roll, float *pitch, float *yaw);
void IMU660RC_AttitudeUpdate(float dt);
void IMU660RC_AttitudeInit(void);

#endif
