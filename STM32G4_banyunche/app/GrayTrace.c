/**
 * @file GrayTrace.c
 * @brief 八通道灰度循迹控制器 — 重心偏差 + 项目 PID 库 + 麦轮输出
 *
 * 偏差解算参考 3519 项目 follow_line.c 的"连续黑线聚类 + 重心法":
 *   - 遍历 8 通道, 相邻黑线视为同一段, 取该段加权均值
 *   - 多段时选离上一帧偏差最近的一段 (抗十字/过线干扰)
 *   - 全白(无黑线)时保持上一帧偏差
 *
 * PID 用项目 pid.h 库 (位置式):
 *   PID_calc(&pid, ref=error, set=0)  →  out = -Kp*error - ...
 *   error>0 需左转 → 角速度取负号输出 (方向按实际走线可调)
 */

#include "Common_used.h"
#include "GrayTrace.h"

/* 全局灰度 PID 句柄: 供 trace_tune 在线调参直接改增益 (g_pid_gray.Kp/Ki/Kd) */
pid_type_def g_pid_gray;

void GrayTrace_Init(GrayTrace_t *gt)
{
    if (gt == NULL) return;

    Grayscale_Init(&gt->sensor);

    const fp32 pid[3] = { g_tune_gray_kp, g_tune_gray_ki, g_tune_gray_kd };
    PID_init(&g_pid_gray, PID_POSITION, pid, GRAY_W_MAX, 0.0f);
    gt->last_error = 0.0f;   /* 首次丢线前无历史, 偏差 0 */
    gt->w_smooth   = 0.0f;   /* 角速度滤波初值 */
    gt->inited = 1U;
}

float GrayTrace_Calc_Error(GrayTrace_t *gt)
{
    static const float weight[8] = GRAY_CH_WEIGHTS;
    float   prev;                      /* 上一帧真实偏差 */
    uint8_t dig;
    float   cluster_sum = 0.0f;
    float   best_error  = 0.0f;
    float   best_dist   = 1e6f;
    uint8_t cluster_cnt = 0U;
    uint8_t has         = 0U;
    int     i;

    if (gt == NULL) return 0.0f;
    prev = gt->last_error;             /* 存真实偏差, 不是 pid.error[1](负值) */
    dig = Grayscale_Get_Digital(&gt->sensor);

    /* 连续黑线 (Bit=0) 视为一段, 多段选离上一帧最近 */
    for (i = 0; i <= 8; i++) {
        uint8_t is_black = (i < 8) && ((dig & (1U << (7 - i))) == 0U);

        if (is_black) {
            cluster_sum += weight[i];
            cluster_cnt++;
        } else if (cluster_cnt > 0U) {
            float c_err = cluster_sum / (float)cluster_cnt;
            float dist = c_err - prev;  /* 离上一帧偏差最近的一段 */
            if (dist < 0.0f) dist = -dist;

            if (!has || dist < best_dist) {
                best_error = c_err;
                best_dist  = dist;
                has = 1U;
            }
            cluster_sum  = 0.0f;
            cluster_cnt  = 0U;
        }
    }

    if (!has) {
        /* 全白 (丢线): 保持上一帧真实偏差, 不猛打方向 */
        return prev;
    }

    if (best_error >  GRAY_ERR_MAX) best_error =  GRAY_ERR_MAX;
    if (best_error < -GRAY_ERR_MAX) best_error = -GRAY_ERR_MAX;
    return best_error;
}

void GrayTrace_Update(GrayTrace_t *gt)
{
    float v, w, error;
    MecanumResult motor;

    if (gt == NULL || !gt->inited) return;

    Trace_Tune_Service();                 /* 同步在线调参增益 + 处理 # 命令 (同 Trace_LineFollow) */
    Grayscale_Update(&gt->sensor);        /* 读 8 通道 */
    error = GrayTrace_Calc_Error(gt);     /* 重心偏差 */
    gt->last_error = error;               /* 记录本帧真实偏差 (丢线时保持用) */

    /* PID: set=0, ref=error → out = -Kp*error (方向与符号需实测调) */
    w = PID_calc(&g_pid_gray, error, 0.0f);

    /* 一阶低通滤波: 平滑角速度防抖 (GRAY_W_ALPHA 越小越平滑/响应越慢) */
    w = gt->w_smooth + GRAY_W_ALPHA * (w - gt->w_smooth);
    if (w >  GRAY_W_MAX) w =  GRAY_W_MAX;
    if (w < -GRAY_W_MAX) w = -GRAY_W_MAX;
    gt->w_smooth = w;                        /* 保存滤波值供下一帧用 */

    printf("%.2f\r\n",w);                    /* 打印滤波后的 w (实际发给电机) */

    v = g_tune_control_override ? g_tune_speed : GRAY_BASE_SPEED;   /* 在线调参 #vmax 生效时用车速调参值 */

    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}
