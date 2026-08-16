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

void GrayTrace_Init(GrayTrace_t *gt)
{
    if (gt == NULL) return;

    Grayscale_Init(&gt->sensor);

    const fp32 pid[3] = { GRAY_KP, GRAY_KI, GRAY_KD };
    PID_init(&gt->pid, PID_POSITION, pid, GRAY_W_MAX, 0.0f);
    gt->inited = 1U;
}

float GrayTrace_Calc_Error(GrayTrace_t *gt)
{
    static const float weight[8] = GRAY_CH_WEIGHTS;
    uint8_t dig;
    float   cluster_sum = 0.0f;
    float   best_error  = 0.0f;
    float   best_dist   = 1e6f;
    uint8_t cluster_cnt = 0U;
    uint8_t has         = 0U;
    int     i;

    if (gt == NULL) return 0.0f;
    dig = Grayscale_Get_Digital(&gt->sensor);

    /* 连续黑线 (Bit=0) 视为一段, 多段选离上一帧最近 */
    for (i = 0; i <= 8; i++) {
        uint8_t is_black = (i < 8) && ((dig & (1U << (7 - i))) == 0U);

        if (is_black) {
            cluster_sum += weight[i];
            cluster_cnt++;
        } else if (cluster_cnt > 0U) {
            float c_err = cluster_sum / (float)cluster_cnt;
            float dist = c_err - gt->pid.error[1];   /* 上一帧偏差 */
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
        /* 全白 (丢线): 维持上一帧偏差, 避免猛打方向 */
        return gt->pid.error[1];
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

    Grayscale_Update(&gt->sensor);        /* 读 8 通道 */
    error = GrayTrace_Calc_Error(gt);     /* 重心偏差 */

    /* PID: set=0, ref=error → out = -Kp*error (方向与符号需实测调) */
    w = PID_calc(&gt->pid, error, 0.0f);
    if (w >  GRAY_W_MAX) w =  GRAY_W_MAX;
    if (w < -GRAY_W_MAX) w = -GRAY_W_MAX;

    v = GRAY_BASE_SPEED;

    motor = Mecanum_Calc(v, w);
    Send_commandmotor(&motor);
}
