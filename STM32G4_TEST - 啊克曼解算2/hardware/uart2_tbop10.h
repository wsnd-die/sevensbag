//
// Created by 35037 on 2026/7/25.
//

#ifndef STM32G4_TEST_TBOP10_H
#define STM32G4_TEST_TBOP10_H

#include "stm32g4xx.h"
#include "Common_used.h"
#include <String.h>

#define TB_RX_BUFF_SIZE	64
#define FRAME_TOTAL_LEN 28U
#define TB_TX_BUFF_SIZE	64

#define TB_Header1 0XAA
#define TB_Header2 0XCC
#define TB_Tail1 0XBB
#define TB_Tail2 0XDD

#define DMA_RX_BUF_SIZE 64

#define TBOP_Send(Buffer,DataLen) UART2_Send(Buffer,DataLen)

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

typedef struct
{
    float xdata;
    float ydata;
} TBData_t;
extern TBData_t TB_position;
extern TBData_t TB_speed;

typedef enum
{
    WAIT_HEADER1,
    WAIT_HEADER2,
    RECV_DATA,
    CHECK_TAIL1,
    CHECK_TAIL2,
} UART_STATE;
extern UART_STATE fsm_state;

typedef union
{
    uint8_t data[4];
    float val;
} FloatUnion;

extern float imu_gz;
extern float imu_yaw;

extern uint8_t Flag_TBOFdata;
extern uint8_t frame_buf[FRAME_TOTAL_LEN];
extern uint8_t frame_index;

extern uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];

void UART2_FSM_Parse_Byte(uint8_t byte);
void UART2_SendCode(uint8_t *DATA, uint8_t len);
void UART2_StartDMAReceive(void);


#endif //STM32G4_TEST_TBOP10_H