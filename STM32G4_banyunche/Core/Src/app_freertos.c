/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "Common_used.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#ifndef LEGACY_USART1_HOST_ENABLE
#define LEGACY_USART1_HOST_ENABLE 0
#endif
/* USER CODE END PD */

/* USER CODE BEGIN PM */
SemaphoreHandle_t Sem_Act_QR;
SemaphoreHandle_t Sem_Act_FINDCIRCLE;
SemaphoreHandle_t Sem_Act_Steer;
SemaphoreHandle_t Sem_Act_FollowLineL;
SemaphoreHandle_t Sem_Act_FollowLineR;
SemaphoreHandle_t Sem_Act_Navigat;

volatile Current_Task_t current_task = Event_IDLE;
volatile TaskCommand_t   g_last_cmd;
/* USER CODE END PM */

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask", .priority = (osPriority_t) osPriorityNormal, .stack_size = 256 * 4
};
osThreadId_t ctrl_servoHandle;
const osThreadAttr_t ctrl_servo_attributes = {
  .name = "ctrl_servo", .priority = (osPriority_t) osPriorityAboveNormal6, .stack_size = 512 * 4
};
osThreadId_t NAVIGATIONHandle;
const osThreadAttr_t NAVIGATION_attributes = {
  .name = "NAVIGATION", .priority = (osPriority_t) osPriorityAboveNormal6, .stack_size = 512 * 4
};
osThreadId_t uart1_motorHandle;
const osThreadAttr_t uart1_motor_attributes = {
  .name = "uart1_motor", .priority = (osPriority_t) osPriorityAboveNormal6, .stack_size = 256 * 4
};
osThreadId_t Uart3_k230Handle;
const osThreadAttr_t Uart3_k230_attributes = {
  .name = "Uart3_k230", .priority = (osPriority_t) osPriorityAboveNormal6, .stack_size = 256 * 4
};
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED", .priority = (osPriority_t) osPriorityHigh, .stack_size = 512 * 4
};

void led() { HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13); osDelay(200); }

void StartDefaultTask(void *argument);
void ctrl_servo_task(void *argument);
void Navigation_TASK(void *argument);
void Uart1M_task(void *argument);
void Uart3K230_task(void *argument);
void OLED_TASK(void *argument);

void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  Sem_Act_QR         = xSemaphoreCreateBinary();
  Sem_Act_Steer       = xSemaphoreCreateBinary();
  Sem_Act_FollowLineL = xSemaphoreCreateBinary();
  Sem_Act_FollowLineR = xSemaphoreCreateBinary();
  Sem_Act_Navigat     = xSemaphoreCreateBinary();
  Sem_Act_FINDCIRCLE  = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */
  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  ctrl_servoHandle = osThreadNew(ctrl_servo_task, NULL, &ctrl_servo_attributes);
  NAVIGATIONHandle = osThreadNew(Navigation_TASK, NULL, &NAVIGATION_attributes);
  uart1_motorHandle = osThreadNew(Uart1M_task, NULL, &uart1_motor_attributes);
  Uart3_k230Handle = osThreadNew(Uart3K230_task, NULL, &Uart3_k230_attributes);
  OLEDHandle = osThreadNew(OLED_TASK, NULL, &OLED_attributes);

  SW_UART_Init();

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	TaskCommand_t cmd;
	task_init();
  for(;;)
  {
   cmd= task_recive();
		if(cmd.k==1)
		{
			g_last_cmd = cmd;
			switch(cmd.Mode)
			{
			case Event_LinFolL:
				if (current_task == Event_Task1) break;
				current_task = Event_Task1; xSemaphoreGive(Sem_Act_FollowLineL); break;
			case Event_LinFolR:
				if (current_task == Event_Task2) break;
				current_task = Event_Task2; xSemaphoreGive(Sem_Act_FollowLineR); break;
			case Event_Navigation:    xSemaphoreGive(Sem_Act_Navigat); break;
			case Event_GoHome:
				current_task = Event_Task2; xSemaphoreGive(Sem_Act_Navigat); break;
			case Event_PickUp: case Event_PlaceDown: case Event_STEERING_ROTATE:
				xSemaphoreGive(Sem_Act_Steer); break;
			case Event_QRCode:      xSemaphoreGive(Sem_Act_QR); break;
			case Event_FindCircle:  xSemaphoreGive(Sem_Act_FINDCIRCLE); break;
			case Event_STOP:        current_task = Event_IDLE; break;
			default: break;
			}
		}
		osDelay(30);
  }
  /* USER CODE END StartDefaultTask */
}

void ctrl_servo_task(void *argument)
{
  /* USER CODE BEGIN Send_motor */
	Servo_SetAngle(38);
  for(;;)
  {
  	xSemaphoreTake(Sem_Act_Steer, portMAX_DELAY);
  	if (g_last_cmd.Mode==Event_PickUp)
  	{
  		Servo_SetAngle(38); Color_Init();
  		BlockBasic_TurntableTo(1);
  		if (BlockBasic_TurntableTo(1)==BLOCK_OK) TT_SetColor(SLOT_1, Color_DetectDominant());
  		BlockBasic_TurntableTo(2);
  		if (BlockBasic_TurntableTo(2)==BLOCK_OK) TT_SetColor(SLOT_2, Color_DetectDominant());
  		BlockBasic_TurntableTo(3);
  		if (BlockBasic_TurntableTo(3)==BLOCK_OK) TT_SetColor(SLOT_3, Color_DetectDominant());
  		BlockBasic_TurntableTo(4);
  		if (BlockBasic_TurntableTo(4)==BLOCK_OK) TT_SetColor(SLOT_4, Color_DetectDominant());
  		BlockBasic_TurntableTo(5);
  		if (BlockBasic_TurntableTo(5)==BLOCK_OK) TT_SetColor(SLOT_5, Color_DetectDominant());
  	}
    osDelay(10);
  }
  /* USER CODE END ctrl_servo_task */
}

void Navigation_TASK(void *argument)
{
  /* USER CODE BEGIN Navigation_TASK */
  for(;;)
  {
  	if (g_last_cmd.Mode==Event_Navigation)  { xSemaphoreTake(Sem_Act_Navigat, portMAX_DELAY); }
  	if (g_last_cmd.Mode==Event_LinFolR)     { xSemaphoreTake(Sem_Act_FollowLineR, portMAX_DELAY); Trace_LineFollow(); }
  	if (g_last_cmd.Mode==Event_LinFolL)     { xSemaphoreTake(Sem_Act_FollowLineL, portMAX_DELAY); Trace_LineFollow(); }
  	if (g_last_cmd.Mode==Event_FindCircle)  { xSemaphoreTake(Sem_Act_Steer, portMAX_DELAY); Circle_Follow(); }
		osDelay(10);
  }
  /* USER CODE END Navigation_TASK */
}

void Uart1M_task(void *argument)
{
  /* USER CODE BEGIN Uart1M_task */
  for(;;) { osDelay(10); }
  /* USER CODE END Uart1M_task */
}

void Uart3K230_task(void *argument)
{
  /* USER CODE BEGIN Uart3Yuyin_task */
  for(;;) { osDelay(10); }
  /* USER CODE END Uart3K230_task */
}

/* USER CODE BEGIN Header_OLED_TASK */
void OLED_TASK(void *argument)
{
  /* USER CODE BEGIN OLED_TASK */
	HAL_UART_Transmit(&huart1, (uint8_t *)"Color Test\r\n", 13, 100);
	Color_SetLedLevel(0);//切记是先设置在初始化否则没办法写入寄存器
	HAL_Delay(50);
	Color_Init();
	HAL_Delay(50);

  for(;;)
  {
		Color_DataTypeDef d;
		HAL_StatusTypeDef rc = Color_ReadData(&d);
		Color_TypeDef c = (rc == HAL_OK) ? Color_Judge(&d) : COLOR_UNKNOWN;

		char buf[64];
		int n = snprintf(buf, sizeof(buf), "R=%d G=%d B=%d -> ", d.red, d.green, d.blue);
		if      (c == COLOR_RED)   n += snprintf(buf+n, sizeof(buf)-n, "RED\r\n");
		else if (c == COLOR_GREEN) n += snprintf(buf+n, sizeof(buf)-n, "GREEN\r\n");
		else if (c == COLOR_BLUE)  n += snprintf(buf+n, sizeof(buf)-n, "BLUE\r\n");
		else if (c == COLOR_WHITE) n += snprintf(buf+n, sizeof(buf)-n, "WHITE\r\n");
		else if (c == COLOR_BLACK) n += snprintf(buf+n, sizeof(buf)-n, "BLACK\r\n");
		else                       n += snprintf(buf+n, sizeof(buf)-n, "? (rc=%d)\r\n", rc);
		HAL_UART_Transmit(&huart1, (uint8_t *)buf, n, 100);
		osDelay(200);
  }
  /* USER CODE END OLED_TASK */
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */
