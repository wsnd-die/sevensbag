/**
 * @file key.h
 * @brief 在此书写key代码
 *        PC13=KEY
 *        其中KEY使用阻塞读取
 * @version 0.1
 * @date 2026-08-02
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef KEY_H
#define KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

uint8_t KEY_Read(void);
uint8_t KEY_IsPressed(void);

#ifdef __cplusplus
}
#endif

#endif
