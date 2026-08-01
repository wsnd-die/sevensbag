/**
 * @file QRcode.h
 * @author your name (you@domain.com)
 * @brief 在此写QRcode的相关代码
 *        PA9=TX, PA10=RX
 *        其中RX使用中断接收，TX使用阻塞发送
 * @version 0.1
 * @date 2026-08-02
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef QRCODE_H
#define QRCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define QRCODE_RX_BUF_SIZE 128U

void QRcode_Start(void);
void QRcode_Clear(void);
const uint8_t *QRcode_GetBuffer(uint16_t *len);
HAL_StatusTypeDef QRcode_Send(const uint8_t *data, uint16_t len, uint32_t timeout);
void QRcode_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void QRcode_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
