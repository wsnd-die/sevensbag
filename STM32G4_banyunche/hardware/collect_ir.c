/**
 * @file    collect_ir.c
 * @brief   红外对射开关 + 颜色传感器收集判断
 */
#include "Common_used.h"
#include "collect_ir.h"

static bool ir_last = false;

void IR_Init(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = IR_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IR_PORT, &g);
}

bool IR_ObjectPresent(void)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(IR_PORT, IR_PIN);
#if IR_ACTIVE == 0
    return (s == GPIO_PIN_RESET);   /* 遮光=低电平 */
#else
    return (s == GPIO_PIN_SET);     /* 遮光=高电平 */
#endif
}

bool IR_ObjectEntered(void)
{
    bool now = IR_ObjectPresent();
    bool edge = (now && !ir_last);  /* 物体进入边沿 (自动复位) */
    ir_last = now;
    return edge;
}

Color_TypeDef Collect_WaitObject(void)
{
    while (!IR_ObjectEntered()) { osDelay(10); }   /* 等物体进入 */
    Color_DataTypeDef d;
    if (Color_ReadData(&d) != HAL_OK) return COLOR_UNKNOWN;
    return Color_Judge(&d);
}