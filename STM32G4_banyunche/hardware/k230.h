/**
 * @file k230.h
 * @brief 在此写k230相关代码
 *        PB10=TX, PB11=RX
 *        其中RX使用DMA接收，TX使用阻塞发送
 * @version 0.1
 * @date 2026-08-02
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef K230_H
#define K230_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define K230_RX_BUF_SIZE           1

void K230_Start(void);
void K230_Clear(void);
const uint8_t *K230_GetBuffer(uint16_t *len);
HAL_StatusTypeDef K230_Send(const uint8_t *data, uint16_t len, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif
