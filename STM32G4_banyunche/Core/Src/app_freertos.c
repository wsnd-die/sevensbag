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
  Color_CalibLoad();

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
  		Servo_SetAngle(38);
  		Color_SetLedLevel(5);
  		HAL_Delay(50);
  		Color_Init();
  		HAL_Delay(50);
  		for (uint8_t slot = 1; slot <= 5; slot++) {
  			if (BlockBasic_TurntableTo(slot) != BLOCK_OK) continue;
  			Color_TypeDef color = Color_DetectDominant();
  			if (color != COLOR_UNKNOWN) TT_SetColor(slot - 1, color);
  		}
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
#define COLOR_CALIB_MODE  0  /* 1=校准, 0=正常 */

void OLED_TASK(void *argument)
{
  /* USER CODE BEGIN OLED_TASK */
	Color_SetLedLevel(0);
	HAL_Delay(50);
	Color_Init();
	HAL_Delay(50);
	Servo_SetAngle(38);

#if COLOR_CALIB_MODE
	const char *steps[] = {"EMPTY","RED","GREEN","BLUE","WHITE","BLACK"};
	const Color_TypeDef colors[] = {COLOR_RED,COLOR_GREEN,COLOR_BLUE,COLOR_WHITE,COLOR_BLACK};
	char msg[64];

	HAL_UART_Transmit(&huart1, (uint8_t *)"=== EMPTY slot ===\r\n", 20, 100);
	osDelay(3000);
	Color_CalibAmbient();
	HAL_UART_Transmit(&huart1, (uint8_t *)"Ambient OK\r\n", 13, 100);

	for (int i = 0; i < 5; i++) {
		int n = snprintf(msg, sizeof(msg), "=== Slot %d: %s ===\r\n", i+1, steps[i+1]);
		HAL_UART_Transmit(&huart1, (uint8_t *)msg, n, 100);

		if (BlockBasic_TurntableTo(i+1) == BLOCK_OK) {
			osDelay(2000);
			Color_DataTypeDef d;
			if (Color_ReadData(&d) == HAL_OK) {
				n = snprintf(msg, sizeof(msg), "  R=%d G=%d B=%d\r\n", d.red, d.green, d.blue);
				HAL_UART_Transmit(&huart1, (uint8_t *)msg, n, 100);
			}
			Color_Calibrate(colors[i]);
			n = snprintf(msg, sizeof(msg), "Slot %d OK\r\n", i+1);
		} else {
			n = snprintf(msg, sizeof(msg), "Slot %d FAIL\r\n", i+1);
		}
		HAL_UART_Transmit(&huart1, (uint8_t *)msg, n, 100);
		osDelay(500);
	}
	Color_CalibSave();
	HAL_UART_Transmit(&huart1, (uint8_t *)"=== SAVED ===\r\n", 14, 100);

  for(;;) { osDelay(1000); }

#else
  for(;;)
  {
		char b[400]; int n = 0;

		/* 实时 RGB + 颜色 */
		Color_DataTypeDef d;
		if (Color_ReadData(&d) == HAL_OK) {
			Color_TypeDef c = Color_Judge(&d);
			n += snprintf(b+n, sizeof(b)-n, "RGB=%d,%d,%d -> %s | ",
				d.red, d.green, d.blue, Color_ToString(c));
		} else {
			n += snprintf(b+n, sizeof(b)-n, "RGB=? | ");
		}

		/* 校准数据摘要 */
		n += snprintf(b+n, sizeof(b)-n, "Amb(%d,%d,%d,%d) | ",
			g_color_ambient.r,g_color_ambient.g,g_color_ambient.b,g_color_ambient.enabled);
		for (int i = 0; i < COLOR_COUNT; i++) {
			Color_Calib_t *c = &g_color_calib[i];
			n += snprintf(b+n, sizeof(b)-n, "%s(%d,%d,%d,%d) ",
				Color_ToString((Color_TypeDef)i), c->r, c->g, c->b, c->enabled);
		}
		n += snprintf(b+n, sizeof(b)-n, "\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)b, n, 100);
		osDelay(500);
  }
#endif
  /* USER CODE END OLED_TASK */
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */