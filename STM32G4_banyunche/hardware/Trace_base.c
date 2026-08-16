/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制 — 8路灰度传感器 + 麦轮
 *        灰度误差 err → 角速度 w → Mecanum_Calc → 电机
 *
 * @note  原方案为 K230 视觉 + 串级 PID(位置环→角度环)，已注释保留。
 *        现改用 8 路灰度传感器：
 *          引脚: PA4 = GRAY_CLK (CLK 输出), PB7 = GRAY_DAT (DAT 输入, 上拉)
 *          误差: 按《8路循迹模块》Track_Err 映射表由灰度字节计算
 *          控制: err × 增益 → 角速度 w (与差速小车左慢右快的等效转向)
 */

#include "Common_used.h"
#include "HWT101_iic.h"
#include "stdlib.h"

/* ======================== 内部状态 ======================== */
/* 原 K230 串级 PID 状态(已注释)：
 * static pid_type_def g_pid_angle;
 * static pid_type_def g_pid_pos;
 * static uint8_t g_pid_inited = 0;
 */

float g_trace_v      = 0.0f;
float g_trace_w      = 0.0f;
float g_trace_angle  = 0.0f;   /* 现为 8 路灰度误差 err */
float g_trace_posx   = 0.0f;   /* 现为 8 路灰度原始值 TrackN */
float g_trace_target = 0.0f;   /* 目标(恒为 0) */

/* ======================== 8路灰度传感器驱动 ========================
 * 移植自《8路循迹模块》Hardware/Track.c (STM32F1, 标准库)
 *   - 原代码: Track_DAT=PA4(输入), Track_SCL=PA5(输出)
 *   - 本车接线: PA4 = CLK 输出, PB7 = DAT 输入(上拉)
 * 时序(与原件一致): CLK 拉低 → 读 DAT → CLK 拉高 → 延时 6us, 重复 8 次取回 1 字节。
 * 说明: 时序为电平保持型, 期间即使被中断打断也不会错位, 故无需关中断。
 * ======================== */

#define GRAY_CLK_PIN    GPIO_PIN_4
#define GRAY_CLK_PORT   GPIOA
#define GRAY_DAT_PIN    GPIO_PIN_7
#define GRAY_DAT_PORT   GPIOB

static uint8_t s_track_init = 0;   /* 灰度 GPIO 是否已初始化 */
static uint8_t s_track_data = 0;   /* 滤波后的灰度字节 */

/* ---- 微秒延时 (CPU 周期循环, 与 sw_uart.c 同款, 不受优化等级影响) ---- */
static void Trace_DelayUs(uint32_t us)
{
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    if (cycles < 8U) return;
    cycles /= 3U;               /* Cortex-M4: subs(1)+bne(2) ≈ 3 cycles/iter */
    __ASM volatile (
        "1: subs %0, %0, #1\n"
        "   bne  1b\n"
        : "+r" (cycles)
        :
        : "cc"
    );
}

/* ---- GPIO 初始化 (幂等: 只执行一次) ---- */
void Trace_Gray_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    if (s_track_init) return;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CLK (PA4): 推挽输出 */
    gpio.Pin   = GRAY_CLK_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GRAY_CLK_PORT, &gpio);

    /* DAT (PB7): 上拉输入 */
    gpio.Pin   = GRAY_DAT_PIN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GRAY_DAT_PORT, &gpio);

    s_track_init = 1;
}

/* ---- 读取 1 位: CLK 拉低 → 读 DAT → CLK 拉高 → 延时 6us ---- */
static uint8_t Trace_Gray_ReadBit(void)
{
    uint8_t bit = 0;
    HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_RESET);
    bit = (uint8_t)HAL_GPIO_ReadPin(GRAY_DAT_PORT, GRAY_DAT_PIN);
    HAL_GPIO_WritePin(GRAY_CLK_PORT, GRAY_CLK_PIN, GPIO_PIN_SET);
    Trace_DelayUs(6);
    return bit;
}

/* ---- 读取 8 路灰度 (含均值滤波, 与 Read_Track_DATA 一致) ---- */
void Trace_Gray_ReadData(uint8_t *data)
{
    uint8_t n = 0;
    uint8_t raw[8] = {0};
    uint8_t current = 0;
    static uint8_t last_track = 0;   /* 上一次的灰度原始值 */

    for (n = 0; n < 8; n++) {
        raw[n] = Trace_Gray_ReadBit();   /* 读回 1 位 */
    }
    /* 8 位合并为 1 字节 (raw[7] 为最低位) */
    current = raw[7] + raw[6]*2 + raw[5]*4 + raw[4]*8 +
              raw[3]*16 + raw[2]*32 + raw[1]*64 + raw[0]*128;

    /* 简单均值滤波 */
    s_track_data = (uint8_t)((current + last_track) / 2);
    last_track = current;
    if (data) *data = s_track_data;
}

/* ---- 灰度误差映射 (与 Track_Err 表一致) ---- */
float Trace_Gray_Error(void)
{
    float err = 0.0f;
    switch (s_track_data) {
        case 0xe7: err = 0.0f;  break;      /* 中间 */
        case 0xcf: err = 3.5f;  break;      /* 右侧小偏差 */
        case 0x9f: err = 5.0f;  break;      /* 右侧中等偏差 */
        case 0x3f: err = 6.0f;  break;      /* 右侧较大偏差 */
        case 0xf3: err = -3.5f; break;      /* 左侧小偏差 */
        case 0xf9: err = -5.0f; break;      /* 左侧中等偏差 */
        case 0xfc: err = -6.0f; break;      /* 左侧较大偏差 */
        case 0xef: err = 2.0f;  break;      /* 右侧极微偏差 */
        case 0xdf: err = 3.0f;  break;      /* 右侧微小偏差 */
        case 0xbf: err = 3.5f;  break;      /* 右侧微小偏差 */
        case 0x7f: err = 7.0f;  break;      /* 右侧极限偏差 */
        case 0xf7: err = -2.0f; break;      /* 左侧极微偏差 */
        case 0xfb: err = -3.0f; break;      /* 左侧微小偏差 */
        case 0xfd: err = -4.5f; break;      /* 左侧较大偏差 */
        case 0xfe: err = -7.0f; break;      /* 左侧极限偏差 */
        case 0x1f: err = 8.0f;  break;      /* 右侧极限偏差 */
        case 0xf8: err = -3.0f; break;      /* 左侧微小偏差 */
        case 0x8f: err = 9.0f;  break;      /* 右侧极限偏差 */
        default:   err = 0.0f;  break;      /* 未识别 / 全白(丢线) / 全黑 */
    }
    return err;
}

/* ======================== 循线跟随主函数 ======================== */

void Trace_LineFollow(void)
{
    float v, w;
    MecanumResult motor;
    uint8_t track = 0;
    float err;

    /* ---- 0. 首次调用时初始化灰度 GPIO ---- */
    Trace_Gray_Init();

    /* ---- 1. 读取 8 路灰度 + 计算误差 ---- */
    Trace_Gray_ReadData(&track);
    err = Trace_Gray_Error();

    /* ---- 2. 误差 → 角速度 w ----
     * 原 K230 方案为串级 PID:
     *   位置环: target_angle = PID_calc(&g_pid_pos, k230_posx, 0.0f);
     *   角度环: w = PID_calc(&g_pid_angle, k230_angle, target_angle);
     * 现改为直接比例: 误差>0(线在右) → 左轮慢/右轮快(等效差速小车), 即 w>0。
     * 若实车转向与期望相反, 将 GRAY_ERR_TO_W_GAIN 取负即可。
     */
    w = err * GRAY_ERR_TO_W_GAIN;
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 3. 速度自适应: 误差大时减速 ---- */
    v = TRACE_BASE_SPEED;
    {
        float slow_level = fabsf(err) / GRAY_ERR_MAX;
        static float v_smooth = TRACE_BASE_SPEED;

        if (slow_level > 1.0f) slow_level = 1.0f;
        v = TRACE_BASE_SPEED * (1.0f - 0.38f * slow_level);
        v_smooth += (v < v_smooth ? 0.55f : 0.18f) * (v - v_smooth);
        v = v_smooth;
    }

    /* ---- 4. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = err;
    g_trace_posx   = (float)track;
    g_trace_target = 0.0f;

    /* ---- 5. 麦轮解算 + 发送 ---- */
    motor = Mecanum_Calc(v, w);
    /* ---- 5.1 实时打印巡线状态 ---- */
    // printf("[TRACE] v=%.3f w=%.3f err=%.2f track=0x%02X | FL=%u FR=%u RL=%u RR=%u\r\n",
    //        g_trace_v, g_trace_w, g_trace_angle, (uint8_t)g_trace_posx,
    //        motor.fl_speed, motor.fr_speed, motor.rl_speed, motor.rr_speed);
    Send_commandmotor(&motor);
}

void Trace_LineTask(void) {

    while (1) {
        Trace_LineFollow();
        osDelay(20);
    }

}
