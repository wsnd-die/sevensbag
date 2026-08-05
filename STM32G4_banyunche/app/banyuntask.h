/**
 * @file    task.h
 * @brief   ����״̬�� �� ѭ�� / ���� / ���
 *
 *          IDLE �����Щ���> LINE_FOLLOW_R / LINE_FOLLOW_L ����> IDLE
 *                 ��
 *                 ������> NAVIGATION ����> STEERING ����> NAVIGATION
 *                 ��
 *                 ������> ERROR ����> IDLE
 *
 *  ͬ��ԭ��: steeringCommandQueue�����У����� 5��
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
 * ϵͳģʽ
 * ============================================================ */
/* ============================================================
 * 系统事件（驱动源 → defaultTask 调度器 → Worker 任务）
 * ============================================================ */
typedef enum {
     Event_Navigation           = 0,   /* 导航        [共享] */
     Event_LinFolL              = 1,   /* 循迹左      [Task1] */
     Event_LinFolR              = 2,   /* 循迹右      [Task2] */
     Event_STOP                 = 3,   /* 停止        [全局] */
     Event_STEERING_ROTATE      = 4,   /* 舵机旋转    [共享] */
     Event_QRCode               = 5,   /* 识别二维码  [共享] */
     Event_FindCircle           = 6,   /* 找圆        [共享] */
     Event_PickUp               = 7,   /* 拾取圆柱/奖杯 [共享: 继承上下文] */
     Event_PlaceDown            = 8,   /* 放下圆柱/奖杯 [共享: 继承上下文] */
     Event_GoHome               = 9,   /* 回家        [Task2] */
} SystemMode_t;

/* ============================================================
 * 任务互斥分组
 * ============================================================
 *
 *  Task1 (圆柱任务): 导航 → 二维码 → 循迹左 → 找圆 → 拾取圆柱 → 放圆柱
 *  Task2 (奖杯任务): 导航 → 二维码 → 循迹右 → 找圆 → 拾取奖杯 → 放奖杯 → 回家
 *
 *  共享事件（导航/二维码/找圆/拾取/放下/舵机）:
 *    继承 current_task 上下文，在哪个任务中触发就属于哪个任务。
 *
 *  专属事件:
 *    循迹左  → 只能是圆柱任务 (Task1)
 *    循迹右  → 只能是奖杯任务 (Task2)
 *    回家    → 只能是奖杯任务 (Task2)
 */
 typedef enum  {
    Event_IDLE         = -1,
    Event_Task1        = 0,   /* 圆柱任务 */
    Event_Task2        = 1,   /* 奖杯任务 + 回家 */
 }Current_Task_t;

 extern volatile Current_Task_t current_task;

/* ============================================================
 * ����ṹ��
 * ============================================================ */
typedef struct {
    uint8_t      k;
    SystemMode_t Mode;
} TaskCommand_t;

/* ============================================================
 * ȫ�־��
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
