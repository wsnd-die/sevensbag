#include "Common_used.h"

TBData_t TB_position = {0};
TBData_t TB_speed = {0};
float imu_gz = 0;
float imu_yaw = 0;
uint8_t Flag_TBOFdata = 0;

/* ---- OpenMV 颜色帧接收 (UART2, AA L A B DD 5字节) ---- */
volatile uint8_t g_uart2_color_ready = 0;
uint8_t g_uart2_color_l, g_uart2_color_a, g_uart2_color_b;

static void UART2_ScanColorFrame(const uint8_t *data, uint16_t len)
{
    /* 扫描 AA L A B DD 模式 */
    for (uint16_t i = 0; i + 4 < len; i++) {
        if (data[i] == 0xAA && data[i + 4] == 0xDD) {
            uint8_t l = data[i + 1];
            if (l <= 100) {  /* Lab L 上限校验 */
                g_uart2_color_l = l;
                g_uart2_color_a = data[i + 2];
                g_uart2_color_b = data[i + 3];
                g_uart2_color_ready = 1;
            }
        }
    }
}

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

volatile uint32_t dbg_rx_cb = 0;  /* CALLBACK 入口计数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    dbg_rx_cb++;
    if (huart->Instance == USART2) {
        /* 扫描 OpenMV 颜色帧 AA L A B DD */
        UART2_ScanColorFrame(dma_rx_buf, Size);
        /* 扫描里程计帧 AA CC ... BB DD */
        for (uint16_t i = 0; i < Size; i++) {
            UART2_FSM_Parse_Byte(dma_rx_buf[i]);
        }
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);
    }
    else if (huart->Instance == USART3) {
        K230_RxRestart();
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

/* ���첢���͵��ֽ�ָ��֡ */
static void UART2_SendCmd(uint8_t cmd)
{
    uint8_t tx_buf[4];
    uint8_t idx = 0;
    tx_buf[idx++] = TB_Header1;
    tx_buf[idx++] = TB_Header2;
    tx_buf[idx++] = cmd;
    tx_buf[idx++] = TB_Tail1;
    tx_buf[idx++] = TB_Tail2;
    HAL_UART_Transmit(&huart2, tx_buf, idx, HAL_MAX_DELAY);
}

/* ����������������ֵ��̣�����������ƽ�ƣ����ȴ�������� */
static void Mecanum_MoveBlocking(float body_dx_m, float body_dy_m)
{
    MecanumMove_t move;
    if (Mecanum_CalculateMove(&g_mecanum_config, body_dx_m, body_dy_m, 0.0f, &move))
    {
        Mecanum_ExecuteMove(&g_mecanum_config, &move);
        osDelay((uint32_t)(move.duration_s * 1000.0f) + 50U);
    }
}

void UART2_calibrate(void)
{

    UART2_SendCmd('x');                       /* AA CC x BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */

    Mecanum_MoveBlocking(1.0f, 0.0f);         /* ǰ�� 1m������ +X Ϊǰ�� */

    UART2_SendCmd('z');                       /* AA CC z BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */

    UART2_SendCmd('y');                       /* AA CC y BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */

    Mecanum_MoveBlocking(0.0f, -1.0f);        /* ˮƽ���� 1m������ +Y Ϊ�󣬹���Ϊ -Y�� */
    osDelay(1000U);

    UART2_SendCmd('z');                       /* AA CC z BB DD */
    osDelay(1000U);                           /* �ȴ� 1 �� */
}

