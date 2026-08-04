#include "FreeRTOS.h"
#include "banyuntask.h"
#include "queue.h"

QueueHandle_t systemEventQueue = NULL;


void task_init()
{
    systemEventQueue =
        xQueueCreate(5, sizeof(TaskCommand_t));
}

void task_send(TaskCommand_t *cmd)
{
    xQueueSend(
        systemEventQueue,
        cmd,
        pdMS_TO_TICKS(100)
    );
}

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
