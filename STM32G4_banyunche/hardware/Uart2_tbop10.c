#include "Common_used.h"

TBData_t TB_position = {0};
TBData_t TB_speed = {0};
float imu_gz = 0;
float imu_yaw = 0;
uint8_t Flag_TBOFdata = 0;

#if LEGACY_USART2_ODOM_ENABLE
UART_STATE fsm_state = WAIT_HEADER1;
uint8_t frame_buf[FRAME_TOTAL_LEN];
uint8_t frame_index = 0;
uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];

void UART2_FSM_Parse_Byte(uint8_t byte)
{
    switch (fsm_state) {
    case WAIT_HEADER1:
        if (byte == TB_Header1) {
            frame_buf[0] = byte;
            fsm_state = WAIT_HEADER2;
        }
        break;
    case WAIT_HEADER2:
        if (byte == TB_Header2) {
            frame_buf[1] = byte;
            frame_index = 2;
            fsm_state = RECV_DATA;
        } else {
            fsm_state = WAIT_HEADER1;
        }
        break;
    case RECV_DATA:
        frame_buf[frame_index++] = byte;
        if (frame_index == FRAME_TOTAL_LEN - 2) {
            fsm_state = CHECK_TAIL1;
        }
        break;
    case CHECK_TAIL1:
        if (byte == TB_Tail1) {
            frame_buf[FRAME_TOTAL_LEN - 2] = byte;
            fsm_state = CHECK_TAIL2;
        } else {
            frame_index = 0;
            fsm_state = WAIT_HEADER1;
        }
        break;
    case CHECK_TAIL2:
        if (byte == TB_Tail2) {
            FloatUnion tmp;
            memcpy(tmp.data, &frame_buf[2], 4);
            TB_position.xdata = tmp.val;
            memcpy(tmp.data, &frame_buf[6], 4);
            TB_position.ydata = tmp.val;
            memcpy(tmp.data, &frame_buf[10], 4);
            TB_speed.xdata = tmp.val;
            memcpy(tmp.data, &frame_buf[14], 4);
            TB_speed.ydata = tmp.val;
            memcpy(tmp.data, &frame_buf[18], 4);
            imu_gz = tmp.val;
            memcpy(tmp.data, &frame_buf[22], 4);
            imu_yaw = tmp.val;
            Flag_TBOFdata = 1;
        }
        frame_index = 0;
        fsm_state = WAIT_HEADER1;
        break;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        for (uint16_t i = 0; i < Size; i++) {
            UART2_FSM_Parse_Byte(dma_rx_buf[i]);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
    }
}
#endif

void UART2_StartDMAReceive(void)
{
#if LEGACY_USART2_ODOM_ENABLE
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
#endif
}

/* 构造并发送单字节指令帧 */
static void UART2_SendCmd(uint8_t cmd)
{
    uint8_t tx_buf[4];
    uint8_t idx = 0;
    tx_buf[idx++] = TB_Header1;
    tx_buf[idx++] = TB_Header2;
    tx_buf[idx++] = cmd;
    tx_buf[idx++] = TB_Tail1;
    tx_buf[idx++] = TB_Tail2;
    UART2_Send(tx_buf, idx);
}

/* 驱动步进电机（麦轮底盘）按车体坐标平移，并等待动作完成 */
static void Mecanum_MoveBlocking(float body_dx_m, float body_dy_m)
{
    MecanumMove_t move;
    if (Mecanum_CalculateMove(&g_mecanum_config, body_dx_m, body_dy_m, 0.0f, &move))
    {
        Mecanum_ExecuteMove(&g_mecanum_config, &move);
        osDelay((uint32_t)(move.duration_s * 1000.0f) + 50U);
    }
}

static void UART2_Send(uint8_t *DATA, uint8_t len)
{
    HAL_UART_Transmit(&huart2, DATA, len, HAL_MAX_DELAY);

    UART2_SendCmd('x');                       /* AA CC x BB DD */
    osDelay(1000U);                           /* 等待 1 秒 */

    Mecanum_MoveBlocking(1.0f, 0.0f);         /* 前进 1m（车体 +X 为前） */

    UART2_SendCmd('z');                       /* AA CC z BB DD */
    osDelay(1000U);                           /* 等待 1 秒 */

    UART2_SendCmd('y');                       /* AA CC y BB DD */
    osDelay(1000U);                           /* 等待 1 秒 */

    Mecanum_MoveBlocking(0.0f, -1.0f);        /* 水平向右 1m（车体 +Y 为左，故右为 -Y） */
    osDelay(1000U);

    UART2_SendCmd('z');                       /* AA CC z BB DD */
    osDelay(1000U);                           /* 等待 1 秒 */
}

