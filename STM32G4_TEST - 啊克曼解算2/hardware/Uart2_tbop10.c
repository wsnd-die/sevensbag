//
// Created by 35037 on 2026/7/25.
//
#include "uart2_tbop10.h"

/* ======== 全局变量定义 ======== */
DMA_HandleTypeDef hdma_usart2_rx;
TBData_t TB_position = {0};
TBData_t TB_speed    = {0};
UART_STATE fsm_state  = WAIT_HEADER1;
float imu_gz          = 0;
float imu_yaw         = 0;
uint8_t Flag_TBOFdata = 0;
uint8_t frame_buf[FRAME_TOTAL_LEN];
uint8_t frame_index   = 0;
uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];

/* ======== FSM 逐字节解析 ======== */
void UART2_FSM_Parse_Byte(uint8_t byte)
{
    switch(fsm_state)
    {
        case WAIT_HEADER1:
        if (byte == TB_Header1)
        {
            frame_buf[0]=byte;
            fsm_state=WAIT_HEADER2;
        }
        break;
        case WAIT_HEADER2:
        if (byte == TB_Header2)
        {
            frame_buf[1]=byte;
            frame_index=2;
            fsm_state=RECV_DATA;
        }
        else
        {
            fsm_state=WAIT_HEADER1;
        }
        break;
        case RECV_DATA:
        frame_buf[frame_index++]=byte;
        if (frame_index==FRAME_TOTAL_LEN-2)
        {
            fsm_state=CHECK_TAIL1;
        }
        break;
        case CHECK_TAIL1:
            if (byte == TB_Tail1)
            {
                frame_buf[FRAME_TOTAL_LEN-2]=byte;
                fsm_state=CHECK_TAIL2;
            }
            else
            {
                frame_index=0;
                fsm_state=WAIT_HEADER1;
            }
            break;
        case CHECK_TAIL2:
            {
                if (byte == TB_Tail2)
                {
                    FloatUnion tmp;
                    memcpy(tmp.data, &frame_buf[2],  4);  TB_position.xdata = tmp.val;
                    memcpy(tmp.data, &frame_buf[6],  4);  TB_position.ydata = tmp.val;
                    memcpy(tmp.data, &frame_buf[10], 4);  TB_speed.xdata   = tmp.val;
                    memcpy(tmp.data, &frame_buf[14], 4);  TB_speed.ydata   = tmp.val;
                    memcpy(tmp.data, &frame_buf[18], 4);  imu_gz           = tmp.val;
                    memcpy(tmp.data, &frame_buf[22], 4);  imu_yaw          = tmp.val;
                    Flag_TBOFdata = 1;
                }
                frame_index=0;
                fsm_state=WAIT_HEADER1;
            }
            break;
    }
}

/* ======== HAL 回调：DMA+IDLE * ======== */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        for (uint16_t i = 0; i < Size; i++)
            UART2_FSM_Parse_Byte(dma_rx_buf[i]);

        /* 重新启动 DMA 接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
    }
}

/* ======== 启动 DMA 接收 ======== */
void UART2_StartDMAReceive(void)
{
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
}

/* ======== 发送函数 ======== */
void UART2_Send(uint8_t *DATA, uint8_t len)
{
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);

    while(len--)
    {
        HAL_UART_Transmit(&huart2, DATA++, 1, HAL_MAX_DELAY);
    }
}

void UART2_SendCode(uint8_t *DATA, uint8_t len)
{
    UART2_Send(DATA, len);
}
