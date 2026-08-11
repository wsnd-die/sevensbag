/**
 * @file    Common_used.h
 * @brief   项目统一公共头文件 — 汇总所有常用 include、extern 声明与宏定义
 *
 * 使用方式：每个 .c 文件仅需 #include "Common_used.h"，
 *          即可获得 HAL、FreeRTOS、外设驱动、硬件模块等全部常用头文件。
 *
 * 注意：该文件会被 STM32CubeMX 自动生成的 main.h 间接包含的场景所依赖，
 *       请勿在 CubeMX 重新生成时覆盖本文件。
 */

#ifndef _COMMON_USED_
#define _COMMON_USED_

/* ============================================================
 * 1. 标准 C 库
 * ============================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <string.h>
#include <stdarg.h>
#include "block_basic.h"
#include "QRcode.h"

/* ============================================================
 * 2. STM32G4 HAL / CMSIS
 * ============================================================ */
#include "main.h"               /* → stm32g4xx_hal.h + GPIO Pin 宏 */
#include "stm32g4xx.h"          /* CMSIS Device Header */

/* ============================================================
 * 3. FreeRTOS / CMSIS-RTOS V2
 * ============================================================ */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "semphr.h"

/* ============================================================
 * 4. STM32CubeMX 外设头文件
 * ============================================================ */
#include "gpio.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* ============================================================
 * 5. 硬件驱动模块
 * ============================================================ */
#include "emm_5v.h"
#include "oled.h"
#include "oled_data.h"
#include "imu660.h"
#include "ahrs_mahony.h"
#include "siyuan_imu.h"
// #include "spi_imu660rc.h"  // 已切换为 IMU660RA 版本，旧版不再使用
#include "mecanum.h"
#include "uart2_tbop10.h"
#include "angle_ctrl.h"
#include "block_basic.h"
#include "Trace_base.h"
#include "Circle_base.h"
#include "color.h"
#include "collect_ir.h"
#include "ColorIdentif.h"
#include "QRcode.h"
/*
 * HWT101_iic.h 不放入公共头文件 —— 其通过 wit_protocol.h 引入 q0/q1/q2/q3
 * 等短宏名，会与 imu_660.c 等模块的局部变量名冲突。
 */
#include "key.h"
#include "k230.h"
#include "sw_uart.h"
#include "waypoint.h"
#include "can.h"
#include "pid.h"
/*
 * wit_protocol.h 不放入公共头文件 —— 其 q0/q1/q2/q3 等短宏名
 * 会与其他模块的局部变量名冲突（如 imu_660.c 的局部变量 q0~q3）。
 * 需要 wit_protocol 的文件请单独 include。
 */

/* ARM DSP 库（麦轮 / IMU 使用） */
#include "arm_math.h"

/* ============================================================
 * 6. 应用层模块
 * ============================================================ */
#include "banyuntask.h"
#include "Mecanum_Move.h"
#include "NavigationMecanum.h"
#include "block_basic.h"

/* ============================================================
 * 7. 项目全局宏定义
 * ============================================================ */
#define use_xing_che   0
#define ni_he_mode     0

/* 串口1 接收电机数据 */
#define RX_BUF_SIZE 10

/* ============================================================
 * 8. 项目全局 extern 变量声明
 * ============================================================ */

/* --- 串口标志 --- */
extern uint8_t FlagOFMotor, FlagOFYuyin;
extern uint8_t Data_uart1[25], Data_uart3[20];

/* --- 串口3 语音数据 --- */
extern char *buffer[23];

/* --- 电机数据 --- */
extern uint16_t left_vel, right_vel;
extern uint8_t left_acc, left_dir;
extern uint8_t right_acc, right_dir;
extern float motor_v, motor_w, data_angle;
extern volatile float front_angle;

/* --- 语音数据 --- */
extern uint8_t buffer_flag;
extern uint8_t buf;

/* ============================================================
 * 9. 项目全局函数原型
 * ============================================================ */
void Uart3_deel(void);
void Uart1_DMA_IDLE_Start(void);
void shell_print(uint8_t *x);        /* 解析上位机电机数据 */
void shell_print3(uint8_t *x);       /* 解析上位机语音数据 */
void Send_commendyu(void);           /* 发送电机命令 */
void Send_commandmotor(MecanumResult *data); /* 发送电机命令（麦轮） */
void Servo_SetAngle(float Angle);
void UART3_Send(uint8_t *DATA, uint8_t len);
void Guan_dao(float DT);

#endif /* _COMMON_USED_ */