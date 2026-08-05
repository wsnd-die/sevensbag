/**
 * @file    task.h
 * @brief   三态状态机 + 事件队列
 *
 *          IDLE ──LinFolL──► Task1 ──LinFolR/GoHome──► Task2
 *            ▲                  │                          │
 *            └──── STOP ────────┴──────── STOP ───────────┘
 *
 *  Task1 (圆柱): 循迹左 + 拾取(转盘颜色收集)
 *  Task2 (奖杯): 循迹右 + 导航 + 找圆 + 放置 + 回家
 *
 *  子事件 (PickUp/PlaceDown/Navigation/QRCode/FindCircle):
 *    不改变状态, 在 current_task 上下文中执行
 */

#ifndef __TASK_H
#define __TASK_H

/*
 * FreeRTOS.h / queue.h 已由 Common_used.h 统一提供，
 * 本文件不再重复包含。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 系统事件（驱动源 → defaultTask 调度器 → Worker 任务）
 * ============================================================ */
typedef enum {
     Event_Navigation           = 0,   /* 导航到目标点 */
     Event_LinFolL              = 1,   /* 循迹左 → 启动 Task1 */
     Event_LinFolR              = 2,   /* 循迹右 → 启动 Task2 */
     Event_STOP                 = 3,   /* 停止 → IDLE */
     Event_STEERING_ROTATE      = 4,   /* 舵机旋转 */
     Event_QRCode               = 5,   /* 识别二维码 → SetQR(idx) */
     Event_FindCircle           = 6,   /* 找圆 */
     Event_PickUp               = 7,   /* 拾取(转盘颜色收集) */
     Event_PlaceDown            = 8,   /* 放置 */
     Event_GoHome               = 9,   /* 回家 → 切到 Task2 并导航 */
} SystemMode_t;

/* ============================================================
 * 三态状态机: IDLE / Task1 / Task2
 *
 *  Task1 (圆柱任务): 导航 → 二维码 → 循迹左 → 找圆 → 拾取圆柱 → 放圆柱
 *  Task2 (奖杯任务): 导航 → 二维码 → 循迹右 → 找圆 → 拾取奖杯 → 放奖杯 → 回家
 *
 *  状态入口:
 *    Event_LinFolL  → Task1
 *    Event_LinFolR  → Task2
 *    Event_GoHome   → Task2 (并触发导航回家)
 *    Event_STOP     → IDLE
 *
 *  子事件在 current_task 上下文中执行，不改变状态。
 * ============================================================ */
 typedef enum  {
    Event_IDLE         = -1,
    Event_Task1        = 0,   /* 圆柱任务 */
    Event_Task2        = 1,   /* 奖杯任务 + 回家 */
 }Current_Task_t;

 extern volatile Current_Task_t current_task;

/* ============================================================
 * 命令结构体
 * ============================================================ */
typedef struct {
    uint8_t      k;       /* 1=有效 */
    SystemMode_t Mode;
} TaskCommand_t;

/* ============================================================
 * 全局队列
 * ============================================================ */

extern QueueHandle_t systemEventQueue ;


/* ============================================================
 * API
 * ============================================================ */
void           task_init(void);
void           task_send(TaskCommand_t *cmd);
TaskCommand_t  task_recive(void);

#ifdef __cplusplus
}
#endif

#endif /* __TASK_H */
