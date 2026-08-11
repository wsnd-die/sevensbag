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
#include "color.h"

#define QRCODE_RX_BUF_SIZE 8

typedef enum {
    champion=1,
    second_place=2,
    third_place=3,

}Jang_type;

typedef struct {
    Color_TypeDef Color_xu[5];
} Qr_Yantitl;
typedef struct {
    uint8_t Jang[3];

}Qr_Jantitl;

void QRcode_Start(void);
void QRcode_Clear(void);
const uint8_t *QRcode_GetBuffer(uint16_t *len);
/* 从串口直接写入二维码数据 (USART2 扫码模块), 置 QR_Flag */
void QRcode_SetData(const uint8_t *buf, uint16_t len);
HAL_StatusTypeDef QRcode_Send(const uint8_t *data, uint16_t len, uint32_t timeout);
uint8_t Slop_dirjang(Jang_type jang);

uint8_t Read_QrFlag(void);
uint8_t QR_deel(void);
uint8_t Qr_Get(void);
#ifdef __cplusplus
}
#endif

#endif
