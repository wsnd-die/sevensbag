/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Common_used.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef LEGACY_USART1_HOST_ENABLE
#define LEGACY_USART1_HOST_ENABLE 0
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
SemaphoreHandle_t Sem_Act_QR;
SemaphoreHandle_t Sem_Act_FINDCIRCLE;
SemaphoreHandle_t Sem_Act_Steer;
SemaphoreHandle_t Sem_Act_FollowLineL;
SemaphoreHandle_t Sem_Act_FollowLineR;
SemaphoreHandle_t Sem_Act_Navigat;

volatile Current_Task_t current_task = Event_IDLE;
volatile TaskCommand_t   g_last_cmd;           /* Worker 可读取最近一次命令 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ctrl_motor */
osThreadId_t ctrl_servoHandle;
const osThreadAttr_t ctrl_servo_attributes = {
  .name = "ctrl_servo",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 512 * 4
};
/* Definitions for NAVIGATION */
osThreadId_t NAVIGATIONHandle;
const osThreadAttr_t NAVIGATION_attributes = {
  .name = "NAVIGATION",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 512 * 4
};
/* Definitions for uart1_motor */
osThreadId_t uart1_motorHandle;
const osThreadAttr_t uart1_motor_attributes = {
  .name = "uart1_motor",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 256 * 4
};
/* Definitions for Uart3_k230 */
osThreadId_t Uart3_k230Handle;
const osThreadAttr_t Uart3_k230_attributes = {
  .name = "Uart3_k230",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 256 * 4
};
/* Definitions for OLED */
osThreadId_t OLEDHandle;
const osThreadAttr_t OLED_attributes = {
  .name = "OLED",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void led()
{
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	 osDelay(200);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ctrl_servo_task(void *argument);
void Navigation_TASK(void *argument);
void Uart1M_task(void *argument);
void Uart3K230_task(void *argument);
void OLED_TASK(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  Sem_Act_QR           = xSemaphoreCreateBinary();
  Sem_Act_Steer       = xSemaphoreCreateBinary();
  Sem_Act_FollowLineL = xSemaphoreCreateBinary();
  Sem_Act_FollowLineR = xSemaphoreCreateBinary();
  Sem_Act_Navigat     = xSemaphoreCreateBinary();
  Sem_Act_FINDCIRCLE  = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ctrl_servo */
  ctrl_servoHandle = osThreadNew(ctrl_servo_task, NULL, &ctrl_servo_attributes);

  /* creation of NAVIGATION */
  NAVIGATIONHandle = osThreadNew(Navigation_TASK, NULL, &NAVIGATION_attributes);

  /* creation of uart1_motor */
  uart1_motorHandle = osThreadNew(Uart1M_task, NULL, &uart1_motor_attributes);

  /* creation of Uart3_k230 */
  Uart3_k230Handle = osThreadNew(Uart3K230_task, NULL, &Uart3_k230_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(OLED_TASK, NULL, &OLED_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	TaskCommand_t cmd;
	 task_init();
  /* Infinite loop */
  for(;;)
  {
   cmd= task_recive();

		if(cmd.k==1)
		{
			/* ============================================================
			 * 三态状态机: IDLE / Task1 / Task2
			 *
			 *  状态入口:
			 *    Event_LinFolL  → Task1 (圆柱: 循迹左 + 拾取颜色)
			 *    Event_LinFolR  → Task2 (奖杯: 循迹右 + 导航 + 放置 + 回家)
			 *
			 *  同状态重复 → 跳过
			 *  STOP          → IDLE
			 *
			 *  子事件不改变状态, 在 current_task 上下文中执行
			 * ============================================================ */
			g_last_cmd = cmd;

			switch(cmd.Mode)
			{
			/* ---- 状态入口 ---- */
			case Event_LinFolL:
				if (current_task == Event_Task1) break;
				current_task = Event_Task1;
				xSemaphoreGive(Sem_Act_FollowLineL);
				break;

			case Event_LinFolR:
				if (current_task == Event_Task2) break;
				current_task = Event_Task2;
				xSemaphoreGive(Sem_Act_FollowLineR);
				break;

			/* ---- 子事件 (不改变状态) ---- */
			case Event_Navigation:
				xSemaphoreGive(Sem_Act_Navigat);
				break;

			case Event_GoHome:
				current_task = Event_Task2;
				xSemaphoreGive(Sem_Act_Navigat);
				break;

			case Event_PickUp:
			case Event_PlaceDown:
			case Event_STEERING_ROTATE:
				xSemaphoreGive(Sem_Act_Steer);
				break;

			case Event_QRCode:
				/* TODO: K230 扫码 → SetQR(idx) */
				xSemaphoreGive(Sem_Act_QR);
				break;

			case Event_FindCircle:
				/* TODO: K230 视觉找圆 → 导航到圆 */
				xSemaphoreGive(Sem_Act_FINDCIRCLE);
				break;

			/* ---- 停止 ---- */
			case Event_STOP:
				current_task = Event_IDLE;
				break;

			default:
				break;
			}
		}
		 osDelay(30);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ctrl_servo_task */
/**
* @brief Function implementing the ctrl_servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ctrl_servo_task */
void ctrl_servo_task(void *argument)
{
  /* USER CODE BEGIN Send_motor */

  /* Infinite loop */
	// BlockBasic_LiftTo(DOWN,43);
	 Servo_SetAngle(38+88);
	// uint32_t last_k230_tick = osKernelGetTickCount();

  for(;;)
  {
		//等待舵机任务
  	xSemaphoreTake(Sem_Act_Steer, portMAX_DELAY);
  	if (g_last_cmd.Mode==Event_PickUp)
  	{
  		Servo_SetAngle(38);
  		Color_Init();

  		/* ---- 转盘 5 槽位颜色收集 ---- */
  		BlockBasic_TurntableTo(1);
  		if (BlockBasic_TurntableTo(1)==BLOCK_OK)
  		{
  			TT_SetColor(SLOT_1, Color_DetectDominant());
  		}
  		BlockBasic_TurntableTo(2);
  		if (BlockBasic_TurntableTo(2)==BLOCK_OK)
  		{
  			TT_SetColor(SLOT_2, Color_DetectDominant());
  		}
  		BlockBasic_TurntableTo(3);
  		if (BlockBasic_TurntableTo(3)==BLOCK_OK)
  		{
  			TT_SetColor(SLOT_3, Color_DetectDominant());
  		}
  		BlockBasic_TurntableTo(4);
  		if (BlockBasic_TurntableTo(4)==BLOCK_OK)
  		{
  			TT_SetColor(SLOT_4, Color_DetectDominant());
  		}
  		BlockBasic_TurntableTo(5);
  		if (BlockBasic_TurntableTo(5)==BLOCK_OK)
  		{
  			TT_SetColor(SLOT_5, Color_DetectDominant());
  		}
  		/* ---- 根据 QR 映射查目标槽位 → 取点位 → 导航 ---- */
  		// uint8_t slot = SlotByColor(target_color);
  		// float x,y,yaw; TogetPos(slot, &x,&y,&yaw);
  		// Nav_SetTarget(&nav, x, y, yaw);
  	}
  	if (g_last_cmd.Mode==Event_PlaceDown)
  	{
  	}

    osDelay(10);
  }
  vTaskDelete(NULL);  /* 安全：永远不应该走到这，但以防万一 */
  /* USER CODE END ctrl_servo_task */
}

/* USER CODE BEGIN Header_Navigation_TASK */
/**
* @brief Function implementing the NAVIGATION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Navigation_TASK */
void Navigation_TASK(void *argument)
{
  /* USER CODE BEGIN Navigation_TASK */
		// Nav_RunWaypoints();

	//Emm_V5_Pos_Control( 2,1,100,50,1500,0,0);
	//Emm_V5_Pos_Control( 4,1,100,50,1500,0,0);
	//Emm_V5_Pos_Control( 4,1,100,50,1500,0,1);
	//HAL_Delay(20);
  // Nav_GoToWorld(0.1,0.0,0);
//Mecanum_RunPath();
  /* Infinite loop */
  for(;;)
  {


  	if (g_last_cmd.Mode==Event_Navigation)
  	{
  		xSemaphoreTake(Sem_Act_Navigat, portMAX_DELAY);
  	}
  	if (g_last_cmd.Mode==Event_LinFolR)
  	{
  		xSemaphoreTake(Sem_Act_FollowLineR, portMAX_DELAY);
  		Trace_LineFollow();
  	}
  	if (g_last_cmd.Mode==Event_LinFolL)
  	{
  		xSemaphoreTake(Sem_Act_FollowLineL, portMAX_DELAY);
  		Trace_LineFollow();
  	}
  	if (g_last_cmd.Mode==Event_FindCircle)
  	{
  		xSemaphoreTake(Sem_Act_Steer, portMAX_DELAY);
  		Circle_Follow();
  	}
		osDelay(10);
  }
  /* USER CODE END Navigation_TASK */
}   //Emm_V5_Synchronous_motion(0);


/* USER CODE BEGIN Header_Uart1M_task */
/**
* @brief Function implementing the uart1_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart1M_task */
void Uart1M_task(void *argument)
{
  /* USER CODE BEGIN Uart1M_task */

  /* Infinite loop */
  for(;;)
  {
			if(Read_QrFlag())
		{
			GetQR();
		}
  	osDelay(10);
  }
  /* USER CODE END Uart1M_task */
}

/* USER CODE BEGIN Header_Uart3Yuyin_task */
/**
* @brief Function implementing the Uart3_yuyin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart3K230_task */
void Uart3K230_task(void *argument)
{
  /* USER CODE BEGIN Uart3Yuyin_task */
  /* Infinite loop */
  for(;;)
  {


    osDelay(10);
  }
  /* USER CODE END Uart3K230_task */
}

/* USER CODE BEGIN Header_OLED_TASK */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_TASK */
void OLED_TASK(void *argument)
{
  /* USER CODE BEGIN OLED_TASK */
  /* Infinite loop */
  for(;;)
  {

		osDelay(10);
  }
  /* USER CODE END OLED_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */
