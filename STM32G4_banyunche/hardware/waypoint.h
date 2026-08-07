/**
 * @file    waypoint.h
 * @brief   路径录制/回放系统 — 与 navigation.c 的 AngleCtrl 集成
 *
 * ============================================================
 * 单位约定：全部使用 mm（与 TBOP / AngleCtrl 一致）
 * ============================================================
 */

#ifndef STM32G4_TEST_WAYPOINT_H
#define STM32G4_TEST_WAYPOINT_H

#include <stdint.h>
#include <stdbool.h>
#include "angle_ctrl.h"

/* ============================================================
 * 容量
 * ============================================================ */
#define WAYPOINT_MAX    500         // 最大存储路径点数

/* ============================================================
 * 工作模式
 * ============================================================ */
typedef enum {
    WP_IDLE     = 0,    // 空闲
    WP_RECORD   = 1,    // 录制模式
    WP_PLAYBACK = 2     // 回放模式
} WaypointMode;

/* ============================================================
 * 单个路径点结构体
 * ============================================================ */
typedef struct {
    float x;        // 世界坐标系 X (mm)
    float y;        // 世界坐标系 Y (mm)
    float yaw;      // 偏航角 (deg)
} Waypoint_t;

/* ============================================================
 * 路径点环形缓冲区
 * ============================================================ */
typedef struct {
    Waypoint_t buffer[WAYPOINT_MAX];
    uint16_t   write_idx;   // 下一个写入位置
    uint16_t   count;       // 当前存储的点数量
    bool       full;        // 缓冲区是否满
} WaypointBuffer_t;

/* ============================================================
 * 路径导航适配器 —— 桥接 waypoint buffer ↔ AngleCtrl
 *
 * 用法示例:
 *
 *   WaypointNav wn;
 *   WaypointAngle_Init(&wn);
 *
 *   // 录制
 *   WaypointNav_StartRecord(&wn);
 *   // 在 IMU_TASK 中:
 *   WaypointAngle_Update(&wn, cur_x, cur_y, cur_yaw, cur_w);
 *
 *   // 回放
 *   WaypointNav_StartPlayback(&wn);
 *   // 在 Send_yuyin 中:
 *   WaypointAngle_Update(&wn, cur_x, cur_y, cur_yaw, cur_w);
 *   motor = Mecanum_Calc(0.0f, wn.ac.cmd_w);  // 纯旋转
 * ============================================================ */
typedef struct {
    AngleCtrl ac;           // 复用现有导航控制器
    uint16_t      target_idx;    // 当前追踪的 waypoint 索引
    uint16_t      total;         // 回放开始时的总点数 (快照)
    WaypointMode  mode;          // 当前工作模式
    uint32_t      record_tick;   // 录制间隔计时 (ms), HAL_GetTick 基准
    uint32_t      record_interval; // 录制间隔 (ms), 默认 100
} WaypointNav;

/* ============================================================
 * 全局实例
 * ============================================================ */
extern WaypointBuffer_t g_path;
extern WaypointNav      g_waypoint_nav;

/* ============================================================
 * 底层 waypoint buffer API (legacy, 仍可用)
 * ============================================================ */

/* 初始化 / 清空路径 */
void waypoint_init(void);
void waypoint_clear(void);

/* 记录当前位置到路径末尾 (单位: mm) */
void waypoint_record(float x, float y, float yaw);

/* 获取已记录的路径点数量 */
uint16_t waypoint_count(void);

/* 获取第 idx 个路径点 (0-based) */
bool waypoint_get(uint16_t idx, float *x, float *y, float *yaw);

/* 回放时获取当前目标点 (同 waypoint_get) */
bool waypoint_get_target(uint16_t target_idx, float *x, float *y, float *yaw);

/* 通过 UART 导出路径点 (CSV 格式) */
void waypoint_export(void);

/* ============================================================
 * WaypointNav 适配器 API (推荐)
 * ============================================================ */

/* 初始化适配器 (清空 buffer, 初始化 AngleCtrl) */
void WaypointAngle_Init(WaypointNav *wn);

/* ---- 模式切换 ---- */
void WaypointNav_StartRecord(WaypointNav *wn);
void WaypointNav_StopRecord(WaypointNav *wn);
void WaypointNav_StartPlayback(WaypointNav *wn);

/* ---- 核心更新 (在任务循环中调用) ---- */
void WaypointAngle_Update(WaypointNav *wn,
                        float cur_x, float cur_y,
                        float cur_yaw, float cur_w);

/* ---- 回放完成判断 ---- */
bool WaypointAngle_Arrived(const WaypointNav *wn);

#endif //STM32G4_TEST_WAYPOINT_H
