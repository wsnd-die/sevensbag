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
#include "sw_uart.h"
#include "k230.h"
#include "Trace_base.h"
#include "Circle_base.h"

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
  .stack_size = 512 * 6
};
/* Definitions for ctrl_servo */
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

  /* creation of ctrl_servo */
  ctrl_servoHandle = osThreadNew(ctrl_servo_task, NULL, &ctrl_servo_attributes);

  /* creation of NAVIGATION */
	NAVIGATIONHandle = osThreadNew(Navigation_TASK, NULL, &NAVIGATION_attributes);

  /* creation of uart1_motor */
  //  uart1_motorHandle = osThreadNew(Uart1M_task, NULL, &uart1_motor_attributes);

  /* creation of Uart3_k230 */
  //  Uart3_k230Handle = osThreadNew(Uart3K230_task, NULL, &Uart3_k230_attributes);

  /* creation of OLED */
  //  OLEDHandle = osThreadNew(OLED_TASK, NULL, &OLED_attributes);

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

  /* ---- 初始化颜色传感器 ---- */
  //(void)Color_Init();

  /* 默认启动循迹模式 */

  K230_RequestMode(K230_MODE_CIRCLE);

  SW_UART_Printf("STM32 boot, K230 mode=LINE\r\n");

  uint32_t last_k230_tick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
      /* ---- K230 模式切换（标志驱动）---- */
      K230_ApplyMode();

      /* ---- K230 数据读取 (角度/位置同包, 方向独立) ---- */
      float angle, posx;
      if (K230_GetLineAngle(&angle)) {
          last_k230_tick = osKernelGetTickCount();
      }
      if (K230_GetPosition(&posx, NULL)) {
          last_k230_tick = osKernelGetTickCount();
      }
      static char s_mode = 0;  /* 0=循迹, 非0=找圆 */
      char dir;
      if (K230_GetCircleDir(&dir)) {
          s_mode = dir;         /* 收到方向数据 → 找圆模式 */
          last_k230_tick = osKernelGetTickCount();
      }

      {
          static uint32_t dbg_cnt = 0;
          if (++dbg_cnt % 2 == 0) {
              if (s_mode != 0) {
                  SW_UART_Printf("D=%c vx=%.3f vy=%.3f\r\n",
                                 s_mode, g_circle_vx, g_circle_vy);
              } else {
                  SW_UART_Printf("A=%.1f X=%.1f T=%.1f v=%.3f w=%.3f\r\n",
                                 g_trace_angle, g_trace_posx, g_trace_target,
                                 g_trace_v, g_trace_w);
              }
          }
      }

      /*
       * 若超过 2 秒未收到 K230 数据，强制重发 'f' 命令。
       * 处理 K230 Python 启动慢（约 2-3 秒）导致初始命令丢失的问题。
       */
      if ((osKernelGetTickCount() - last_k230_tick) > 2000U) {
          K230_SetMode(K230_MODE_CIRCLE);
          last_k230_tick = osKernelGetTickCount();

          uint32_t rx_bytes, rx_ok, rx_err, rx_unk;
          K230_GetDiag(&rx_bytes, &rx_ok, &rx_err, &rx_unk);
          SW_UART_Printf("K230 retry rx:byte=%lu ok=%lu err=%lu unk=%lu\r\n",
                         rx_bytes, rx_ok, rx_err, rx_unk);
      }

      osDelay(10);
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
  /* USER CODE BEGIN ctrl_servo_task */
  /* Infinite loop */
	// BlockBasic_LiftTo(DOWN,43);
	 Servo_SetAngle(38);
	// uint32_t last_k230_tick = osKernelGetTickCount();

  for(;;)
  {
    /* 循迹 — 持续运行 */
  	Circle_Follow();
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
	// NavController nav;
	// MecanumResult motor;
	// int wp = 0;
	// MecanumResult motor_data={0};
	// motor_data=Mecanum_Calc(0.2,0.3);


	// Nav_Init(&nav);
	// WaypointNav_Init(&g_waypoint_nav);

	/* 默认目标: 原点 */

	/*static const struct { float x, y, yaw; } pts[] = {
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
*/
	// Nav_RunWaypoints();
  /* Infinite loop */
  for(;;)
  {
  	osDelay(100);
  	// Send_commandmotor(&motor_data);

		/* ---- 标定中: 不输出导航电机 ---- */
/*		if (g_calib.state != CALIB_IDLE && g_calib.state != CALIB_DONE) {
			osDelay(10);
			continue;
		}
		//= ---- 录制模式: 不输出电机 (手动遥控) ---- =/
		else if (g_waypoint_nav.mode == WP_RECORD) {
			//= 录制由 OLED_TASK 驱动, 此处不干涉电机 */
		}
		// ---- 空闲模式: 独立 NavController 定点悬停 ---- */
		// else {
		// 	Nav_Update(&nav,
		// 		TB_position.xdata, TB_position.ydata,
		// 		imu_yaw, imu_gz);
		// 	motor = Mecanum_Calc_Full(nav.cmd_vx, nav.cmd_vy, nav.cmd_w);
		// 	Send_commandmotor(&motor);
		//
		// 	if (Nav_Arrived(&nav)) {
		// 		wp = (wp + 1) % n;
		// 		Nav_SetTarget(&nav, pts[wp].x, pts[wp].y, pts[wp].yaw);
// 		// 	}
// 		}
		osDelay(10);
	osDelay(10);
  /* USER CODE END Navigation_TASK */
}

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
  	Circle_Follow();
  }
  /* USER CODE END Uart1M_task */
}

/* USER CODE BEGIN Header_Uart3K230_task */
/**
* @brief Function implementing the Uart3_k230 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart3K230_task */
void Uart3K230_task(void *argument)
{
  /* USER CODE BEGIN Uart3K230_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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

