#include "Common_used.h"

static uint8_t qrcode_rx_byte;
static uint8_t qrcode_rx_buf[QRCODE_RX_BUF_SIZE];
static volatile uint16_t qrcode_rx_len=0;
uint8_t QR_Flag;

void QRcode_Start(void)
{
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);
}

void QRcode_Clear(void)
{
    qrcode_rx_len = 0;
    memset(qrcode_rx_buf, 0, sizeof(qrcode_rx_buf));
}

const uint8_t *QRcode_GetBuffer(uint16_t *len)
{
    if (len != NULL) {
        *len = qrcode_rx_len;
    }
    return qrcode_rx_buf;
}

HAL_StatusTypeDef QRcode_Send(const uint8_t *data, uint16_t len, uint32_t timeout)
{
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, len, timeout);
}

uint8_t Qr_Get() {

    while (Read_QrFlag()==0) {
        osDelay(50);
    }

    return QR_deel();
}


uint8_t QR_deel()
{
    uint8_t P,i;
    static uint8_t 	result=0;

    for(i=0;i<3;i++)
    {
        P=qrcode_rx_buf[i];
        if(P==0x0D)
        {
            break;
        }
        result=result*10+(P-0x30);
    }
    HAL_UART_Transmit_IT(&huart1,&result ,1 );
    result=0;i=result;
    return i;
}
uint8_t Read_QrFlag()
{
    uint8_t i;
    i=QR_Flag;
    QR_Flag=0;
    return i;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART1) {
        if (qrcode_rx_len >= QRCODE_RX_BUF_SIZE) {
            qrcode_rx_len = 0;
        }

        qrcode_rx_buf[qrcode_rx_len++] = qrcode_rx_byte;
        if(qrcode_rx_byte==0x0d)
        {
            QR_Flag=1;
            qrcode_rx_len=0;
        }

        HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);



    }

}
void QRcode_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);
}
