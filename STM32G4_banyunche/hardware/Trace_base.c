/**
 * @file  Trace_base.c
 * @brief 循迹底盘控制
 *        默认(FIND_LINE_ONLY_TASK=0): K230 串级 PID 循线
 *            外环: 位置 → PID → 目标角度
 *            内环: 角度 → PID → 角速度 w → Mecanum_Calc → 电机
 *        找线模式(FIND_LINE_ONLY_TASK=1): 8路灰度"只找线"
 *            原地旋转扫线, 任意一路压到线即停车
 */

#include "Common_used.h"
#include "HWT101_iic.h"
#include "stdlib.h"
/* ======================== 内部状态 ======================== */

static pid_type_def g_pid_angle;
static pid_type_def g_pid_pos;
static uint8_t g_pid_inited = 0;

float g_trace_v      = 0.0f;
float g_trace_w      = 0.0f;
float g_trace_angle  = 0.0f;
float g_trace_posx   = 0.0f;
float g_trace_target = 0.0f;

#define FIND_LINE_ONLY_TASK 0
#if FIND_LINE_ONLY_TASK

/* ======================== 8路灰度传感器驱动 ========================
 * 时序(与原件一致): CLK 拉低 → 读 DAT → CLK 拉高 → 延时 6us, 重复 8 次取回 1 字节。
 * 说明: 时序为电平保持型, 期间即使被中断打断也不会错位, 故无需关中断。
 * ======================== */

#define GRAY_CLK_PIN    GPIO_PIN_4
#define GRAY_CLK_PORT   GPIOA
#define GRAY_DAT_PIN    GPIO_PIN_7
#define GRAY_DAT_PORT   GPIOB

static uint8_t s_track_init = 0;   /* 灰度 GPIO 是否已初始化 */

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

/* ---- 读取 8 路灰度原始字节 ----
 * 不再做字节均值滤波: 对字节求平均会得到查表表外的"假对中"值
 * (如 0xE7 与 0xCF 平均 → 0xDB), 反而放大开关式振荡。
 * 平滑交给控制环节的误差低通 (Trace_LineFollow)。
 */
void Trace_Gray_ReadData(uint8_t *data)
{
    uint8_t n = 0;
    uint8_t raw[8] = {0};
    uint8_t current = 0;

    for (n = 0; n < 8; n++) {
        raw[n] = Trace_Gray_ReadBit();   /* 读回 1 位 */
    }
    /* 8 位合并为 1 字节 (raw[7] 为最低位) */
    current = raw[7] + raw[6]*2 + raw[5]*4 + raw[4]*8 +
              raw[3]*16 + raw[2]*32 + raw[1]*64 + raw[0]*128;
    if (data) *data = current;
}

/* ---- 8路加权质心误差 ----
 * 每个传感器按物理位置赋权重, 传感器3/4之间为0 (与 0xE7=对中 的语义一致):
 *    传感器:  0    1    2    3    4    5    6    7
 *    权重:   -7   -5   -3   -1   +1   +3   +5   +7      (左 - / 右 +)
 * 对压线传感器(bit=0)求位置加权平均 → 连续误差。
 * 相比原离散查表(手工标定、数值互相矛盾, 如 0xbf→3.5 而 0x7f→7.0),
 * 质心误差单调自洽, 不会出现"0→满幅"的跳变。
 * 返回范围约 ±7。全白(0xFF)/全黑(0x00)由调用方单独处理。
 */
static float Trace_Gray_Centroid(uint8_t data)
{
    static const float s_pos[8] = { -7.0f, -5.0f, -3.0f, -1.0f,
                                    +1.0f, +3.0f, +5.0f, +7.0f };
    float sum = 0.0f;
    uint8_t cnt = 0;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if (!(data & (1U << i))) {      /* bit=0 → 该传感器压在线上 */
            sum += s_pos[i];
            cnt++;
        }
    }
    if (cnt == 0U) return 0.0f;         /* 全白: 丢线, 由调用方处理 */
    if (cnt >= 7U) return 0.0f;         /* 全黑/大面积压线: 视为无效 */
    return sum / (float)cnt;
}

/* ======================== 循线跟随主函数 ======================== */

void Trace_LineFollow(void)
{
    MecanumResult motor;
    uint8_t track = 0;
    float err_raw = 0.0f;
    float err     = 0.0f;          /* 低通后的误差 */
    float w, v;
    static float   s_err_f     = 0.0f;  /* 误差低通值 */
    static float   s_err_prev  = 0.0f;  /* 上一周期误差 (PD 微分) */
    static uint8_t s_lost_cnt  = 0;     /* 连续丢线周期计数 */
    static uint32_t s_dbg_cnt  = 0;

    /* ---- 0. 首次调用时初始化灰度 GPIO ---- */
    Trace_Gray_Init();

    /* ---- 1. 读取 8 路灰度 ---- */
    Trace_Gray_ReadData(&track);

    /* ---- 2. 计算误差(质心) + 低通 ----
     * 丢线(全白 0xFF)或全黑(0x00): 保持上次误差一小段时间, 避免
     * "假对中 → 停止修正 → 继续漂 → 突然满打" 的开关振荡;
     * 超时后误差归零, 车恢复直行。 */
    if (track == 0xFFU || track == 0x00U) {
        if (s_lost_cnt < GRAY_LOST_HOLD_CYCLES) {
            s_lost_cnt++;
            err = s_err_f;               /* 保持上次误差 */
        } else {
            s_err_f    = 0.0f;
            s_err_prev = 0.0f;
            err        = 0.0f;
        }
    } else {
        s_lost_cnt = 0;
        err_raw = Trace_Gray_Centroid(track);
        s_err_f += GRAY_ERR_LP * (err_raw - s_err_f);   /* 一阶低通 */
        err = s_err_f;
    }

    /* ---- 3. 低通误差 → PD 角速度 ----
     *   w = KP*err + KD*(err - err_prev)   (KD 按 ~10ms 周期标定)
     *   P: 小误差成比例修正, 消除原满幅开关;
     *   D: 误差增大时提前补转, 误差回落时抑制过冲。
     * 若实车转向与期望相反, 将 GRAY_P_KP 取负即可。 */
    w = GRAY_P_KP * err + GRAY_P_KD * (err - s_err_prev);
    s_err_prev = err;
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 4. 速度自适应: 误差大时减速 ---- */
    v = TRACE_BASE_SPEED;
    {
        float slow_level = fabsf(err) / GRAY_ERR_MAX;
        static float v_smooth = TRACE_BASE_SPEED;

        if (slow_level > 1.0f) slow_level = 1.0f;
        v = TRACE_BASE_SPEED * (1.0f - 0.38f * slow_level);
        v_smooth += (v < v_smooth ? 0.55f : 0.18f) * (v - v_smooth);
        v = v_smooth;
    }

    /* ---- 5. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = err;
    g_trace_posx   = (float)track;
    g_trace_target = 0.0f;

    /* ---- 6. 麦轮解算 + 发送 ----
     * 调试打印降频到每 20 周期一次(~200ms), 避免阻塞控制周期。 */
    motor = Mecanum_Calc(v, w);
    if (++s_dbg_cnt >= 20U) {
        s_dbg_cnt = 0;
        printf("[TRACE] v=%.3f w=%.3f err=%.2f track=0x%02X | FL=%u FR=%u RL=%u RR=%u\r\n",
               g_trace_v, g_trace_w, g_trace_angle, (uint8_t)g_trace_posx,
               motor.fl_speed, motor.fr_speed, motor.rl_speed, motor.rr_speed);
    }
    Send_commandmotor(&motor);
}

void Trace_LineTask(void) {

    while (1) {
        Trace_LineFollow();
        osDelay(10);            /* 20ms→10ms, 提高控制带宽 */
    }

}

#else

/* ======================== 循线跟随主函数 ======================== */

void Trace_LineFollow(void)
{
    float v, w;
    MecanumResult motor;

    /* ---- 1. 获取 K230 角度 + 位置 ---- */
    float k230_angle, k230_posx = 0.0f;
    if (!K230_GetLineAngle(&k230_angle)) {
        return;
    }
    int has_pos = K230_GetPosition(&k230_posx, NULL);


    /* ---- 2. PID 初始化 ---- */
    if (!g_pid_inited) {
        const fp32 ak[3] = { ANGLE_KP, ANGLE_KI, ANGLE_KD };
        const fp32 pk[3] = { POS_KP,   POS_KI,   POS_KD };
        PID_init(&g_pid_angle, PID_POSITION, ak, TRACE_W_MAX, 10.0f);       /* 内环输出 w, 限幅 TRACE_W_MAX */
        PID_init(&g_pid_pos,   PID_POSITION, pk, ANGLE_OUT_MAX, POS_INTEGRAL_MAX); /* 外环输出角度, 限幅 ANGLE_OUT_MAX */
        g_pid_inited = 1;
    }

    /* ---- 3. 外环: 位置 → 目标角度 (期望=0, 反馈=位置) ---- */
    /* PID_calc(pid, ref, set): ref=反馈(实际位置), set=目标(0) */
    float target_angle = PID_calc(&g_pid_pos, k230_posx, 0.0f);
    float curve_comp = -k230_angle * TRACE_CURVE_OUTER_GAIN;
    if (curve_comp >  TRACE_CURVE_OUTER_MAX_DEG) curve_comp =  TRACE_CURVE_OUTER_MAX_DEG;
    if (curve_comp < -TRACE_CURVE_OUTER_MAX_DEG) curve_comp = -TRACE_CURVE_OUTER_MAX_DEG;
    target_angle += curve_comp;
    if (target_angle >  ANGLE_OUT_MAX) target_angle =  ANGLE_OUT_MAX;
    if (target_angle < -ANGLE_OUT_MAX) target_angle = -ANGLE_OUT_MAX;

    /* ---- 4. 内环: 角度 → 角速度 w (期望=target_angle, 反馈=角度) ---- */
    /* PID_calc(pid, ref, set): ref=反馈(实际角度), set=目标(target_angle) */
    w = PID_calc(&g_pid_angle, k230_angle, target_angle);
    if (w >  TRACE_W_MAX) w =  TRACE_W_MAX;
    if (w < -TRACE_W_MAX) w = -TRACE_W_MAX;

    /* ---- 5. 速度自适应 ---- */
    v = TRACE_BASE_SPEED;
    {
        float abs_angle = fabsf(k230_angle);
        float abs_posx = fabsf(k230_posx);
        float curve_level = abs_angle / 35.0f;
        float pos_level = abs_posx / 6.0f;
        float slow_level = curve_level > pos_level ? curve_level : pos_level;
        static float v_smooth = TRACE_BASE_SPEED;

        if (slow_level > 1.0f) slow_level = 1.0f;
        v = TRACE_BASE_SPEED * (1.0f - 0.25f * slow_level);
        v_smooth += (v < v_smooth ? 0.55f : 0.18f) * (v - v_smooth);
        v = v_smooth;
    }

    /* ---- 6. 保存供打印 ---- */
    g_trace_v      = v;
    g_trace_w      = w;
    g_trace_angle  = k230_angle;
    g_trace_posx   = k230_posx;
    g_trace_target = target_angle;

    /* ---- 7. 麦轮解算 + 发送 ---- */
    motor = Mecanum_Calc(v, w);
    /* ---- 7.1 实时打印巡线状态 ---- */
    // printf("[TRACE] v=%.3f w=%.3f angle=%.2f posx=%.3f target=%.2f comp=%.2f | FL=%u FR=%u RL=%u RR=%u | hwt_yaw=%.2f hwt_gz=%.2f\r\n",
    //        g_trace_v, g_trace_w, g_trace_angle, g_trace_posx, g_trace_target, curve_comp,
    //        motor.fl_speed, motor.fr_speed, motor.rl_speed, motor.rr_speed,
    //        HWT101_GetZeroYaw(), g_hwt101_gyro_z);
    Send_commandmotor(&motor);
}

void Trace_LineTask(void) {

    while (1) {
        Trace_LineFollow();
        osDelay(20);
    }

}

#endif /* FIND_LINE_ONLY_TASK */
