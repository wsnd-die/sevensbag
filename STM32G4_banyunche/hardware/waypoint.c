/**
 * @file    waypoint.c
 * @brief   路径录制/回放 — NavController 集成实现
 *
 * 单位约定：全部 mm（与 TBOP / NavController 一致）
 */

#include "Common_used.h"

/* ============================================================
 * 全局实例
 * ============================================================ */
WaypointBuffer_t g_path;
WaypointNav      g_waypoint_nav;

/* ============================================================
 * 底层: waypoint buffer 操作
 * ============================================================ */

void waypoint_init(void)
{
    memset(&g_path, 0, sizeof(g_path));
}

void waypoint_clear(void)
{
    waypoint_init();
}

void waypoint_record(float x, float y, float yaw)
{
    taskENTER_CRITICAL();

    Waypoint_t *wp = &g_path.buffer[g_path.write_idx];

    wp->x   = x;
    wp->y   = y;
    wp->yaw = yaw;

    g_path.write_idx++;

    if (g_path.write_idx >= WAYPOINT_MAX) {
        g_path.write_idx = 0;
        g_path.full = true;
    }

    if (!g_path.full) {
        g_path.count++;
    }

    taskEXIT_CRITICAL();
}

uint16_t waypoint_count(void)
{
    return g_path.full ? WAYPOINT_MAX : g_path.count;
}

bool waypoint_get(uint16_t idx, float *x, float *y, float *yaw)
{
    taskENTER_CRITICAL();

    uint16_t total = g_path.full ? WAYPOINT_MAX : g_path.count;
    if (idx >= total) {
        taskEXIT_CRITICAL();
        return false;
    }

    uint16_t real_idx;
    if (g_path.full) {
        real_idx = (g_path.write_idx + idx) % WAYPOINT_MAX;
    } else {
        real_idx = idx;
    }

    *x   = g_path.buffer[real_idx].x;
    *y   = g_path.buffer[real_idx].y;
    *yaw = g_path.buffer[real_idx].yaw;

    taskEXIT_CRITICAL();
    return true;
}

bool waypoint_get_target(uint16_t target_idx, float *x, float *y, float *yaw)
{
    return waypoint_get(target_idx, x, y, yaw);
}

/* ============================================================
 * WaypointNav 适配器实现
 * ============================================================ */

void WaypointNav_Init(WaypointNav *wn)
{
    waypoint_init();

    /* 初始化内部 NavController */
    Nav_Init(&wn->nav);
    
    wn->target_idx      = 0;
    wn->total           = 0;
    wn->mode            = WP_IDLE;
    wn->record_tick     = 0;
    wn->record_interval = 100;   /* 默认 100ms = 10Hz 录制 */
}

/* ---- 模式切换 ---- */

void WaypointNav_StartRecord(WaypointNav *wn)
{
    printf("REC START\r\n");
    if (wn->mode == WP_PLAYBACK) return;  /* 回放中不允许切换 */
    waypoint_clear();
    wn->record_tick = HAL_GetTick();
    wn->mode        = WP_RECORD;
}

void WaypointNav_StopRecord(WaypointNav *wn)
{
    printf("REC STOP, %d points\r\n", waypoint_count());
    if (wn->mode == WP_RECORD) {
        wn->mode = WP_IDLE;
    }
}

void WaypointNav_StartPlayback(WaypointNav *wn)
{
    uint16_t n = waypoint_count();
    if (n == 0) return;  /* 没有录制点，不启动 */

    if (wn->mode == WP_RECORD) {
        WaypointNav_StopRecord(wn);
    }

    /* 快照当前路径 */
    wn->total      = n;
    wn->target_idx = 0;

    /* 设置第一个目标 */
    float x, y, yaw;
    waypoint_get(0, &x, &y, &yaw);
    Nav_SetTarget(&wn->nav, yaw);

    wn->mode = WP_PLAYBACK;
    printf("PLAY START, %d points\r\n", n);
}

/* ---- 核心更新 ---- */

void WaypointNav_Update(WaypointNav *wn,
                        float cur_x, float cur_y,
                        float cur_yaw, float cur_w)
{
    switch (wn->mode) {
    case WP_RECORD: {
        /* 按间隔录制 */
        uint32_t now = HAL_GetTick();
        if (now - wn->record_tick >= wn->record_interval) {
            wn->record_tick = now;
            waypoint_record(cur_x, cur_y, cur_yaw);
        }
        /* 录制模式下不输出电机控制量 */
        wn->nav.cmd_w  = 0.0f;
        break;
    }

    case WP_PLAYBACK: {
        if (wn->target_idx >= wn->total) {
            /* 回放完成 */
            wn->mode = WP_IDLE;
            wn->nav.cmd_w  = 0.0f;
            return;
        }

        /* 喂入 NavController */
        Nav_Update(&wn->nav, cur_yaw, cur_w);

        /* 到达当前点 → 切换到下一个 */
        if (Nav_Arrived(&wn->nav)) {
            wn->target_idx++;

            if (wn->target_idx >= wn->total) {
                /* 全部完成 */
                wn->mode = WP_IDLE;
                wn->nav.cmd_w  = 0.0f;
                return;
            }

            /* 设置下一个目标 */
            float x, y, yaw;
            waypoint_get(wn->target_idx, &x, &y, &yaw);
            Nav_SetTarget(&wn->nav, yaw);
        }
        break;
    }

    case WP_IDLE:
    default:
        wn->nav.cmd_w  = 0.0f;
        break;
    }
}

bool WaypointNav_Arrived(const WaypointNav *wn)
{
    return (wn->mode == WP_IDLE && wn->total > 0);
}

/* ============================================================
 * 导出 (CSV 格式, via UART3)
 * ============================================================ */

void waypoint_export(void)
{
    uint16_t total = waypoint_count();
    printf("=== WAYPOINT START (%d points) ===\r\n", total);

    for (uint16_t i = 0; i < total; i++) {
        float x, y, yaw;
        if (waypoint_get(i, &x, &y, &yaw)) {
            printf("%d,%.4f,%.4f,%.2f\r\n", i,
                   (double)x, (double)y, (double)yaw);
        }
    }

    printf("=== WAYPOINT END ===\r\n");
}
