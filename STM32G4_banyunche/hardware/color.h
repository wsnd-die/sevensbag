/**
 * @file color.h
 * @brief GY-33 颜色传感器驱动接口和颜色识别结果定义。
 * @version 0.1
 * @date 2026-08-01
 * @copyright Copyright (c) 2026
 * @note 本模块只负责颜色传感器读取和颜色判断，不主动接入主循环或任务。
 *       1. 底层通信使用 CubeMX 生成的 I2C3：PA8=SCL，PB5=SDA。
 *       2. 单次颜色由 RGB 阈值判断，稳定颜色由多次采样投票得到。
 *       3. 调用可在任务中按需调用 Color_DetectDominant()。
 */

 
/**
 * @brief  调用流程参考。
 *
 * 1. CubeMX 生成的初始化保持在 main.c 中：
 *      MX_GPIO_Init();
 *      MX_I2C3_Init();
 *
 * 2. 初始化阶段或任务开始时调用一次：
 *      if (Color_Init() != HAL_OK) {
 *          // 传感器未响应，可做错误提示或稍后重试
 *      }
 *
 * 3. 在主循环或 FreeRTOS 任务中检测稳定颜色：
 *      Color_TypeDef color = Color_DetectDominant();
 *      if (color != COLOR_UNKNOWN) {
 *          const char *name = Color_ToString(color);
 *      }
 *
 * 4. 如需查看原始 RGB：
 *      Color_DataTypeDef data;
 *      if (Color_ReadData(&data) == HAL_OK) {
 *          // data.red / data.green / data.blue
 *      }
 */

#ifndef __COLOR_H
#define __COLOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * 颜色传感器芯片选择：
 *   0 = GY-33 模块（内置 MCU，I2C 地址 0x5A，上电即用）
 *   1 = TCS34725（纯芯片，I2C 地址 0x29，需要初始化 PON+AEN+ATIME）
 */
#define COLOR_SENSOR_CHIP          0U

/* I2C 地址（7 位地址左移 1 位） */
#if COLOR_SENSOR_CHIP == 0
  #define COLOR_SENSOR_I2C_ADDR    (0x5AU << 1)   /* GY-33 */
#else
  #define COLOR_SENSOR_I2C_ADDR    (0x29U << 1)   /* TCS34725 */
#endif

/* 颜色识别结果。 */
typedef enum
{
    COLOR_UNKNOWN = 0,  /* 未识别或本次投票不稳定。 */
    COLOR_RED,          /* 红色。 */
    COLOR_GREEN,        /* 绿色。 */
    COLOR_BLUE,         /* 蓝色。 */
    COLOR_WHITE,        /* 白色。 */
    COLOR_BLACK,        /* 黑色。 */
    COLOR_COUNT         /* 颜色枚举数量，仅用于数组边界。 */
} Color_TypeDef;

/* GY-33 一帧传感器数据。 */
typedef struct
{
    uint16_t raw_red;             /* 原始红色通道。 */
    uint16_t raw_green;           /* 原始绿色通道。 */
    uint16_t raw_blue;            /* 原始蓝色通道。 */
    uint16_t raw_clear;           /* 原始透明/清光通道。 */
    uint16_t lux;                 /* 照度。 */
    uint16_t color_temperature;   /* 色温。 */
    uint8_t red;                  /* 8 位红色通道。 */
    uint8_t green;                /* 8 位绿色通道。 */
    uint8_t blue;                 /* 8 位蓝色通道。 */
    uint8_t sensor_color;         /* 模块内部识别出的颜色位。 */
    uint8_t online;               /* 1=本次读取成功，0=读取失败。 */
} Color_DataTypeDef;

/**
 * @brief  检查 GY-33 颜色传感器是否在线。
 * @retval HAL_OK     传感器有响应。
 * @retval HAL_ERROR  传感器无响应或 I2C 通信失败。
 */
HAL_StatusTypeDef Color_Init(void);

/**
 * @brief  读取 GY-33 一帧颜色数据。
 * @param  data  颜色数据结构体指针。
 * @retval HAL_OK     读取成功，data->online 会置 1。
 * @retval HAL_ERROR  参数错误或读取失败，data->online 会置 0。
 */
HAL_StatusTypeDef Color_ReadData(Color_DataTypeDef *data);

/**
 * @brief  设置 GY-33 板载 LED 亮度等级。
 * @param  level  亮度等级，合法范围 0~10；数值越小通常越亮。
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef Color_SetLedLevel(uint8_t level);

/**
 * @brief  执行一次 GY-33 白平衡。
 * @retval HAL_OK / HAL_ERROR
 * @note   调用前应让传感器正对标准白色目标，并保持环境光稳定。
 */
HAL_StatusTypeDef Color_WhiteBalance(void);

/**
 * @brief  根据单帧 RGB 数据判断颜色。
 * @param  data  已由 Color_ReadData() 成功填充的数据。
 * @return Color_TypeDef  单次颜色判断结果。
 */
Color_TypeDef Color_Judge(const Color_DataTypeDef *data);

/**
 * @brief  多次采样并投票，返回稳定颜色。
 * @return Color_TypeDef  稳定颜色；票数不足时返回 COLOR_UNKNOWN。
 * @note   本函数内部包含 HAL_Delay()，不要在高频或硬实时路径中调用。
 */
Color_TypeDef Color_DetectDominant(void);

/**
 * @brief  将颜色枚举转换成调试字符串。
 * @param  color  颜色枚举。
 * @return const char*  颜色名称字符串。
 */
const char *Color_ToString(Color_TypeDef color);

#ifdef __cplusplus
}
#endif

#endif
