#include "Common_used.h"

QueueHandle_t systemEventQueue = NULL;


void task_init()
{
    systemEventQueue =
        xQueueCreate(5, sizeof(TaskCommand_t));
}

/*Event_Navigation
     Event_LinFolL
     Event_LinFolR
     Event_STOP
     Event_STEERING_ROTATE
cmd
*/
/**
 * @brief 发送任务命令
 * @param mode 任务模式
 */
void task_send(SystemMode_t mode)
{
    TaskCommand_t cmd;
    cmd.k = 1;
    cmd.Mode = mode;
    xQueueSend(
        systemEventQueue,
        &cmd,
        pdMS_TO_TICKS(100)
    );
}

/**
 * @brief 接收任务命令
 * @return 接收到的任务命令
 */
TaskCommand_t task_recive()
{
    TaskCommand_t receivedCmd;

    if (xQueueReceive(systemEventQueue, &receivedCmd,
                      portMAX_DELAY) == pdPASS)
    {
        return receivedCmd;
    }

    receivedCmd.k = 0;
    return receivedCmd;
}


