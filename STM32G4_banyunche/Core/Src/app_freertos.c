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
#include "waypoint.h"
#include "tim.h"
#include "color.h"
#include "oled.h"
#include "NavigationMecanum.h"
#include "semphr.h"  
#include "banyuntask.h"
//#include "sw_uart.h"

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
SemaphoreHandle_t Sem_Act_M;
SemaphoreHandle_t Sem_Act_Steer;
SemaphoreHandle_t Sem_Act_FollowLineL;
SemaphoreHandle_t Sem_Act_FollowLineR;
SemaphoreHandle_t Sem_Act_Navigat;
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
osThreadId_t ctrl_motorHandle;
const osThreadAttr_t ctrl_motor_attributes = {
  .name = "ctrl_motor",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 256 * 4
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
/* Definitions for Uart3_yuyin */
osThreadId_t Uart3_yuyinHandle;
const osThreadAttr_t Uart3_yuyin_attributes = {
  .name = "Uart3_yuyin",
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
void Send_motor(void *argument);
void Navigation_TASK(void *argument);
void Uart1M_task(void *argument);
void Uart3Yuyin_task(void *argument);
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
  /* add semaphores, ... */
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

  /* creation of ctrl_motor */
  ctrl_motorHandle = osThreadNew(Send_motor, NULL, &ctrl_motor_attributes);

  /* creation of NAVIGATION */
  NAVIGATIONHandle = osThreadNew(Navigation_TASK, NULL, &NAVIGATION_attributes);

  /* creation of uart1_motor */
  uart1_motorHandle = osThreadNew(Uart1M_task, NULL, &uart1_motor_attributes);

  /* creation of Uart3_yuyin */
  Uart3_yuyinHandle = osThreadNew(Uart3Yuyin_task, NULL, &Uart3_yuyin_attributes);

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
				switch(cmd.Mode)
					{
					case Event_Navigation:xSemaphoreGive(Sem_Act_Navigat);
						break;
					case Event_LinFolL:xSemaphoreGive(Sem_Act_FollowLineL);
						break;
					case Event_LinFolR:xSemaphoreGive(Sem_Act_FollowLineR);
						break;
					case Event_STOP:
						break;
					case Event_STEERING_ROTATE:xSemaphoreGive(Sem_Act_Steer);
						break;
					}
			}
			 osDelay(30);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Send_motor */
/**
* @brief Function implementing the ctrl_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_motor */
void Send_motor(void *argument)
{
  /* USER CODE BEGIN Send_motor */
	
  /* Infinite loop */
  for(;;)
  {
		//等待舵机任务
		xSemaphoreTake(Sem_Act_Steer, portMAX_DELAY);
		
    osDelay(10);
  }
  /* USER CODE END Send_motor */
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
		Nav_RunWaypoints();
	
	//Emm_V5_Pos_Control( 2,1,100,50,1500,0,0);
	//Emm_V5_Pos_Control( 4,1,100,50,1500,0,0);
	//Emm_V5_Pos_Control( 4,1,100,50,1500,0,1);
	//HAL_Delay(20);
  // Nav_GoToWorld(0.1,0.0,0);
//Mecanum_RunPath();
  /* Infinite loop */
  for(;;)
  {
	 xSemaphoreTake(Sem_Act_Navigat, portMAX_DELAY);
		
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
	uint8_t *x;
  /* Infinite loop */
  for(;;)
  {

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
/* USER CODE END Header_Uart3Yuyin_task */
void Uart3Yuyin_task(void *argument)
{
  /* USER CODE BEGIN Uart3Yuyin_task */
	uint8_t i=0;
  /* Infinite loop */
  for(;;)
  {
		if(i==0)
		{
			xSemaphoreTake(Sem_Act_FollowLineL, portMAX_DELAY);
			
		}
		
		else
		{
			xSemaphoreTake(Sem_Act_FollowLineR, portMAX_DELAY);
			
			
		}
		
		
		
		
    osDelay(1);
  }
  /* USER CODE END Uart3Yuyin_task */
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
		/* ---- 路径录制 (录制模式下按间隔自动记录) ---- */


		// /* ---- OLED 显示 ---- */
		// OLED_Clear();
		// OLED_Printf(1,1,OLED_6X8,"x: %.2f",TB_position.xdata);
		// OLED_Printf(1,9,OLED_6X8,"y: %.2f",TB_position.ydata);
		// OLED_Printf(1,17,OLED_6X8,"vx:%.2f vy:%.2f",TB_speed.xdata,TB_speed.ydata);
		// OLED_// if (g_waypoint_nav.mode == WP_RECORD) {
		//		// 	WaypointNav_Update(&g_waypoint_nav,
		//		// 		TB_position.xdata, TB_position.ydata,
		//		// 		imu_yaw, imu_gz);
		//		// }
		//		// /* ---- 里程计标定状态机 ---- */
		//		// if (g_calib.state != CALIB_IDLE && g_calib.state != CALIB_DONE) {
		//		// 	Odometry_Calib_Update();
		//		// }Printf(1,27,OLED_6X8,"PWM:%.2f",(front_angle) / 180 * 2000 + 500);
		// OLED_Printf(1,37,OLED_6X8,"gz:%.2f",imu_gz);
		// OLED_Printf(1,47,OLED_6X8,"yaw:%.2f",imu_yaw);
		//
		// /* 底部状态栏: 模式 + 航点数 */
		// const char *mode_str = "IDLE";
		// if (g_calib.state == CALIB_FWD)   mode_str = "CAL_FWD";
		// if (g_calib.state == CALIB_RIGHT) mode_str = "CAL_RGT";
		// if (g_calib.state == CALIB_DONE)  mode_str = "CAL_OK";
		// if (g_waypoint_nav.mode == WP_RECORD)   mode_str = "REC";
		// if (g_waypoint_nav.mode == WP_PLAYBACK) mode_str = "PLAY";
		// OLED_Printf(1,55,OLED_6X8,"%s wp:%d", mode_str, waypoint_count());
		//
		// OLED_Update();
		osDelay(10);
  }
  /* USER CODE END OLED_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

