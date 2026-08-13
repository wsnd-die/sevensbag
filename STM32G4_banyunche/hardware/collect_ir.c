/**
 * @file    collect_ir.c
 * @brief   红外对射开关 + 颜色传感器收集判断
 */
#include "Common_used.h"
#include "collect_ir.h"

static bool ir_last = false;

void IR_Init(void)
{
#if defined(PWR_CR3_UCPD_DBDIS)
    /* PB6 带 UCPD 死电池 5.1kΩ 下拉, 复位后可能激活, 会干扰红外读数 → 关闭 */
    SET_BIT(PWR->CR3, PWR_CR3_UCPD_DBDIS);
#endif

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
    if (now && !ir_last) {
        /* 防抖: 检测到进入后等 20ms 再确认, 滤掉沿上的抖动 */
        osDelay(5);
        now = IR_ObjectPresent();
    }
    bool edge = (now && !ir_last);  /* 物体进入边沿 (自动复位) */
    ir_last = now;
    return edge;
}

bool Collect_WaitEnter(void)
{
    while (!IR_ObjectEntered()) { osDelay(5); }   /* 等物体进入(带防抖) */
    return true;
}

Color_TypeDef Collect_ReadColor(void)
{
    /* 多帧采样取平均, 提高颜色识别准确度 (不追求快) */
    uint32_t rs = 0, gs = 0, bs = 0;
    int n = 0;
    for (int i = 0; i < 40 && n < 3; i++) {   /* 最多 ~160ms, 采满 3 帧即停 */
        if (g_uart2_gy33_ready) {
            g_uart2_gy33_ready = 0;
            rs += g_uart2_gy33_r;
            gs += g_uart2_gy33_g;
            bs += g_uart2_gy33_b;
            n++;
        }
        osDelay(4);
    }
    if (n == 0) return COLOR_UNKNOWN;

    Color_DataTypeDef d;
    d.red   = (uint8_t)(rs / n);
    d.green = (uint8_t)(gs / n);
    d.blue  = (uint8_t)(bs / n);
    d.online = 1U;
    return Color_Judge(&d);
}

Color_TypeDef Collect_WaitObject(void)
{
    Collect_WaitEnter();
    return Collect_ReadColor();
}

