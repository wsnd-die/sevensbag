#include "QRcode.h"
#include "usart.h"
#include <string.h>

static uint8_t qrcode_rx_byte;
static uint8_t qrcode_rx_buf[QRCODE_RX_BUF_SIZE];
static volatile uint16_t qrcode_rx_len;

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

void QRcode_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    if (qrcode_rx_len >= QRCODE_RX_BUF_SIZE) {
        qrcode_rx_len = 0;
    }
    qrcode_rx_buf[qrcode_rx_len++] = qrcode_rx_byte;
    HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);
}

void QRcode_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);
}
