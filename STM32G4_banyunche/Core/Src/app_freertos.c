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
  .stack_size = 256 * 4
};
/* Definitions for uart1_motor */
#if LEGACY_USART1_HOST_ENABLE
osThreadId_t uart1_motorHandle;
const osThreadAttr_t uart1_motor_attributes = {
  .name = "uart1_motor",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 256 * 4
};
#endif
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
  .stack_size = 640 * 4
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
#if LEGACY_USART1_HOST_ENABLE
void Uart1M_task(void *argument);
#endif
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
#if LEGACY_USART1_HOST_ENABLE
  uart1_motorHandle = osThreadNew(Uart1M_task, NULL, &uart1_motor_attributes);
#endif

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
	MecanumResult motor_data;
  /* Infinite loop */
  //Servo_SetAngle(90);
	for(;;)
  {
        Servo_SetAngle(125-88);
		// Servo_SetAngle(160);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 60 / 180 * 2000 + 500);
		BlockBasic_LiftTo(10);
		osDelay(1);
		// float current_time=HAL_GetTick();
		// float last_time=0.0f;
		// if (current_time-last_time<=200)
		// {
		// motor_data=Mecanum_Calc(0.2, 0.8);
  //       Send_commandmotor(&motor_data);
		// 	last_time=current_time;
		// }
		// osDelay(10);
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
	NavController nav;
	MecanumResult motor;
	int wp = 0;

	Nav_Init(&nav);
	WaypointNav_Init(&g_waypoint_nav);

	/* 默认目标: 原点 */
	static const struct { float x, y, yaw; } pts[] = {
		{ 0, 0, 0 },
		// {213.62,677.14,90},
		// {129.04,889.82,65.71},
		// {344.56,1252.38,39.48},
		// {753.96,1519.38,3.29},
		// {1187.62,1537.93,-19.79},
		// {1581.93,1342.86,-44.30},
		// {787.90,787.96,0},
		// {556.89,568.52,0},
		// {795.93,-24.56,0},
		// {553.58,-134.45,0},
		// {589.58,-621.70,0},
	};
	const int n = sizeof(pts) / sizeof(pts[0]);
	Nav_SetTarget(&nav, pts[wp].x, pts[wp].y, pts[wp].yaw);

  /* Infinite loop */
  for(;;)
  {
		/* ---- 标定中: 不输出导航电机 ---- */
		if (g_calib.state != CALIB_IDLE && g_calib.state != CALIB_DONE) {
			osDelay(10);
			continue;
		}
		/* ---- 录制模式: 不输出电机 (手动遥控) ---- */
		else if (g_waypoint_nav.mode == WP_RECORD) {
			/* 录制由 OLED_TASK 驱动, 此处不干涉电机 */
		}
		/* ---- 空闲模式: 独立 NavController 定点悬停 ---- */
		else {
			Nav_Update(&nav,
				TB_position.xdata, TB_position.ydata,
				imu_yaw, imu_gz);
			motor = Mecanum_Calc_Full(nav.cmd_vx, nav.cmd_vy, nav.cmd_w);
			Send_commandmotor(&motor);

			if (Nav_Arrived(&nav)) {
				wp = (wp + 1) % n;
				Nav_SetTarget(&nav, pts[wp].x, pts[wp].y, pts[wp].yaw);
			}
		}

		osDelay(10);
  }
  /* USER CODE END Navigation_TASK */
}

/* USER CODE BEGIN Header_Uart1M_task */
/**
* @brief Function implementing the uart1_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart1M_task */
#if LEGACY_USART1_HOST_ENABLE
void Uart1M_task(void *argument)
{
  /* USER CODE BEGIN Uart1M_task */
	uint8_t *x;
  /* Infinite loop */
  for(;;)
  {
		#if ni_he_mode
		
		#else
		shell_print3(x);
		#endif
		osDelay(10);
  }
  /* USER CODE END Uart1M_task */
}
#endif

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

  /* Infinite loop */
  for(;;)
  {
		/* 录制模式下通过串口发送位置数据给上位机 */
		// if (g_waypoint_nav.mode == WP_RECORD) {
		// 	printf("%.2f,%.2f,%.2f\n",
		// 	       imu_yaw, TB_position.xdata, TB_position.ydata);
		// }
  	printf("%.2f,%.2f,%.2f\n",
	   imu_yaw, TB_position.xdata, TB_position.ydata);
		osDelay(50);
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
		if (g_waypoint_nav.mode == WP_RECORD) {
			WaypointNav_Update(&g_waypoint_nav,
				TB_position.xdata, TB_position.ydata,
				imu_yaw, imu_gz);
		}
		/* ---- 里程计标定状态机 ---- */
		if (g_calib.state != CALIB_IDLE && g_calib.state != CALIB_DONE) {
			Odometry_Calib_Update();
		}

		/* ---- OLED 显示 ---- */
		OLED_Clear();
		OLED_Printf(1,1,OLED_6X8,"x: %.2f",TB_position.xdata);
		OLED_Printf(1,9,OLED_6X8,"y: %.2f",TB_position.ydata);
		OLED_Printf(1,17,OLED_6X8,"vx:%.2f vy:%.2f",TB_speed.xdata,TB_speed.ydata);
		OLED_Printf(1,27,OLED_6X8,"PWM:%.2f",(front_angle) / 180 * 2000 + 500);
		OLED_Printf(1,37,OLED_6X8,"gz:%.2f",imu_gz);
		OLED_Printf(1,47,OLED_6X8,"yaw:%.2f",imu_yaw);

		/* 底部状态栏: 模式 + 航点数 */
		const char *mode_str = "IDLE";
		if (g_calib.state == CALIB_FWD)   mode_str = "CAL_FWD";
		if (g_calib.state == CALIB_RIGHT) mode_str = "CAL_RGT";
		if (g_calib.state == CALIB_DONE)  mode_str = "CAL_OK";
		if (g_waypoint_nav.mode == WP_RECORD)   mode_str = "REC";
		if (g_waypoint_nav.mode == WP_PLAYBACK) mode_str = "PLAY";
		OLED_Printf(1,55,OLED_6X8,"%s wp:%d", mode_str, waypoint_count());

		OLED_Update();
		osDelay(10);
  }
  /* USER CODE END OLED_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

