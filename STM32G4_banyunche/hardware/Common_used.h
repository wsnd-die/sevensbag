#ifndef _COMMON_USED_
#define _COMMON_USED_
#include "FreeRTOS.h"
#include <stdio.h>
#include "task.h"
#include "usart.h"
#include "Send_yuyin.h"
#include "emm_5v.h"
#include "oled.h"
#include "imu660.h"
#include "ackermann1.h"
#include "mecanum.h"
#include "uart2_tbop10.h"
#include "navigation.h"

#define use_xing_che   0
#define ni_he_mode     0
//==串口1接收电机数据==//
#define RX_BUF_SIZE 10
extern uint8_t FlagOFMotor,FlagOFYuyin;
extern uint8_t Data_uart1[25],Data_uart3[20];

//==串口3接收语音数据==//
extern char *buffer[23];

//==电机数据==//
extern uint16_t left_vel,right_vel;
extern uint8_t left_acc,left_dir;
extern uint8_t right_acc,right_dir;
extern float motor_v,motor_w,data_angle;
extern volatile float front_angle;
//==语音数据==//
extern uint8_t buffer_flag;
extern uint8_t buf;
void Uart3_deel(void);
void Uart1_DMA_IDLE_Start(void);

void shell_print(uint8_t *x);//解析上位机的电机数据
void shell_print3(uint8_t *x);//解析上位机的语音数据

void Send_commendyu(void);//发送电机命令
void Send_commandmotor(MecanumResult *data);//发送电机命令
void Servo_SetAngle(float Angle);
void UART3_Send(uint8_t *DATA, uint8_t len);
void Guan_dao(float DT);
#endif


