/**
 * @file    task.h
 * @brief   任务状态机 — 循线 / 导航 / 舵机
 *
 *          IDLE ──┬──> LINE_FOLLOW_R / LINE_FOLLOW_L ──> IDLE
 *                 │
 *                 ├──> NAVIGATION ──> STEERING ──> NAVIGATION
 *                 │
 *                 └──> ERROR ──> IDLE
 *
 *  同步原语: steeringCommandQueue（队列，容量 5）
 */

#ifndef __TASK_H
#define __TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 系统模式
 * ============================================================ */
typedef enum {
     Event_Navigation           = 0,
    Event_LinFolL           = 1,
    Event_LinFolR          = 2,
    Event_STOP            = 3,
    Event_STEERING_ROTATE = 4,
} SystemMode_t;

/* ============================================================
 * 命令结构体
 * ============================================================ */
typedef struct {
    uint8_t      k;      /* 附加参数           */
    SystemMode_t Mode;   /* 目标 / 来源模式    */
} TaskCommand_t;

/* ============================================================
 * 全局句柄
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
