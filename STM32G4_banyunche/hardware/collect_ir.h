/**
 * @file    collect_ir.h
 * @brief   红外对射开关判断物体进入 + 颜色读取 — 收集判断
 */
#ifndef COLLECT_IR_H
#define COLLECT_IR_H

#include "color.h"
#include <stdbool.h>

/* 红外对射开关引脚 (按实际接线修改) */
#define IR_PORT    GPIOB
#define IR_PIN     GPIO_PIN_6
#define IR_ACTIVE  0        /* 0=遮光低电平, 1=遮光高电平 */

void          IR_Init(void);
bool          IR_ObjectPresent(void);   /* 当前是否有物体遮光 */
bool          IR_ObjectEntered(void);   /* 物体刚进入(边沿, 自动复位) */
Color_TypeDef Collect_WaitObject(void); /* 等一个物体进入并读色, 返回颜色 */

#endif
