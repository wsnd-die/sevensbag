/**
 * @file HWT101.h
 * @brief 在此书写HWT101的底层代码
 *        PB9=SDA, PB8=SCL
 *        其中SDA使用DMA接收，SCL使用阻塞发送
 * @version 0.1
 * @date 2026-08-02
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef HWT101_H
#define HWT101_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define HWT101_I2C_ADDR (0x50U << 1)

HAL_StatusTypeDef HWT101_IsReady(void);
HAL_StatusTypeDef HWT101_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef HWT101_WriteReg(uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
