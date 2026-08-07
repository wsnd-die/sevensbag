/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
volatile TaskCommand_t   g_last_cmd;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for NLFKION */
osThreadId_t NLFKIONHandle;
const osThreadAttr_t NLFKION_attributes = {
  .name = "NLFKION",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 512 * 4
};
/* Definitions for ColorFunion */
osThreadId_t ColorFunionHandle;
const osThreadAttr_t ColorFunion_attributes = {
  .name = "ColorFunion",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 512 * 4
};
/* Definitions for BsRtFunion */
osThreadId_t BsRtFunionHandle;
const osThreadAttr_t BsRtFunion_attributes = {
  .name = "BsRtFunion",
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
/* Definitions for FCFUION */
osThreadId_t FCFUIONHandle;
const osThreadAttr_t FCFUION_attributes = {
  .name = "FCFUION",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for QRFUNION */
osThreadId_t QRFUNIONHandle;
const osThreadAttr_t QRFUNION_attributes = {
  .name = "QRFUNION",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for IMU_TASK */
osThreadId_t IMU_TASKHandle;
const osThreadAttr_t IMU_TASK_attributes = {
  .name = "IMU_TASK",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void NLF_TASK(void *argument);
void Color_task(void *argument);
void BsRt_task(void *argument);
void OLED_TASK(void *argument);
void FC_TASK(void *argument);
void QR_TASK(void *argument);
void IMU_FUCTION(void *argument);

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

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of NLFKION */
  NLFKIONHandle = osThreadNew(NLF_TASK, NULL, &NLFKION_attributes);

  /* creation of ColorFunion */
  ColorFunionHandle = osThreadNew(Color_task, NULL, &ColorFunion_attributes);

  /* creation of BsRtFunion */
  BsRtFunionHandle = osThreadNew(BsRt_task, NULL, &BsRtFunion_attributes);

  /* creation of OLED */
  OLEDHandle = osThreadNew(OLED_TASK, NULL, &OLED_attributes);

  /* creation of FCFUION */
  FCFUIONHandle = osThreadNew(FC_TASK, NULL, &FCFUION_attributes);

  /* creation of QRFUNION */
  QRFUNIONHandle = osThreadNew(QR_TASK, NULL, &QRFUNION_attributes);

  /* creation of IMU_TASK */
  IMU_TASKHandle = osThreadNew(IMU_FUCTION, NULL, &IMU_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */
  SW_UART_Init();
  Color_CalibLoad();
  /* USER CODE BEGIN RTOS_EVENTS */
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

/* USER CODE BEGIN Header_NLF_TASK */
/**
* @brief Function implementing the NLFKION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NLF_TASK */
void NLF_TASK(void *argument)
{
  /* USER CODE BEGIN NLF_TASK */
	/*
	 *导航循线任务
	 */
	K230_SetMode(K230_MODE_CIRCLE);
	K230_ApplyMode();


  /* Infinite loop */
  for(;;)
  {
  	Circle_Follow();
  	osDelay(10);
    osDelay(1);
  }
  /* USER CODE END NLF_TASK */
}

/* USER CODE BEGIN Header_Color_task */
/**
* @brief Function implementing the ColorFunion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Color_task */
void Color_task(void *argument)
{
  /* USER CODE BEGIN Color_task */
#define COLOR_CALIB 0
#if COLOR_CALIB
#define COLOR_CALIB_MODE 0
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
		n += snprintf(b+n, sizeof(b)-n, "Amb(%d,%d,%d,%d) ",
			g_color_ambient.r,g_color_ambient.g,g_color_ambient.b,g_color_ambient.enabled);
		for (int i = 0; i < COLOR_COUNT; i++)
			n += snprintf(b+n, sizeof(b)-n, "%c:%d ", "URGWB"[i], g_color_calib[i].enabled);
		n += snprintf(b+n, sizeof(b)-n, "\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t *)b, n, 100);
		osDelay(50);
  }
#endif
#endif
	for (;;)
	{
		printf("TRACE: ang=%.1f posX=%.1f v=%.2f w=%.2f | CIRCLE: dir=%c vx=%.2f vy=%.2f\r\n",
		       g_trace_angle, g_trace_posx, g_trace_v, g_trace_w,
		       g_circle_dir, g_circle_vx, g_circle_vy);


		osDelay(100);

	}

  /* USER CODE END Color_task */
}

/* USER CODE BEGIN Header_BsRt_task */
/**
* @brief Function implementing the BsRtFunion thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BsRt_task */
void BsRt_task(void *argument)
{
	/* USER CODE BEGIN BsRt_task */
	/*
	 *舵机转盘任务
	 */
	Servo_SetAngle(38);
	BlockBasic_TurntableTo(1);
	HAL_Delay(1000);
	for(;;)
	{
		xSemaphoreTake(Sem_Act_Steer, portMAX_DELAY);

		if (g_last_cmd.Mode==Event_PickUp)
		{
				Servo_SetAngle(38);
			Color_SetLedLevel(0);
			HAL_Delay(50);
			Color_Init();
			HAL_Delay(50);
			/* 逐槽位检测：RGB跳变→有物块→旋转；无跳变→环境光→停止 */
			for (uint8_t slot = 1; slot <= 5; slot++) {
				Color_DataTypeDef d;
				if (Color_ReadData(&d) != HAL_OK) {slot--;continue;}
				/* 计算和环境光基线的 RGB 差值（绝对值之和） */
				int dr = abs((int)d.red   - g_color_ambient.r);
				int dg = abs((int)d.green - g_color_ambient.g);
				int db = abs((int)d.blue  - g_color_ambient.b);
				if (dr < 30 && dg < 30 && db < 30) {
					/* 跳变太小 → 环境光/空槽 → 停止 */
					slot--;
					continue;
				}
				/* 跳变明显 → 有物块 → 判断颜色 → 旋转 */
				Color_TypeDef c = Color_Judge(&d);
				TT_SetColor(slot - 1, c);
				if (slot < 5) {
					if (BlockBasic_TurntableTo(slot + 1) != BLOCK_OK) break;
					HAL_Delay(500);  /* 等待舵机转到位 */
				}
			}
		}
		osDelay(10);
		/* USER CODE END BsRt_task */
	}
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

  for(;;)
  {

		osDelay(500);
  }

  /* USER CODE END OLED_TASK */
}

/* USER CODE BEGIN Header_FC_TASK */
/**
* @brief Function implementing the FCFUION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FC_TASK */
void FC_TASK(void *argument)
{
  /* USER CODE BEGIN FC_TASK */
	/*
	 *找圆任务
	 *
	 */

  /* Infinite loop */
  for(;;)
  {
  	if (g_last_cmd.Mode==Event_FindCircle)  { xSemaphoreTake(Sem_Act_Steer, portMAX_DELAY); Circle_Follow(); }

    osDelay(50);
  }
  /* USER CODE END FC_TASK */
}

/* USER CODE BEGIN Header_QR_TASK */
/**
* @brief Function implementing the QRFUNION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_QR_TASK */
void QR_TASK(void *argument)
{
  /* USER CODE BEGIN QR_TASK */
	uint8_t QR_result=0;

  /* Infinite loop */
  for(;;)
  {

  	if (g_last_cmd.Mode==Event_QRCode)  { xSemaphoreTake(Sem_Act_QR, portMAX_DELAY);  QR_result=Qr_Get(); }

    osDelay(50);
  }
  /* USER CODE END QR_TASK */
}

/* USER CODE BEGIN Header_IMU_FUCTION */
/**
* @brief Function implementing the IMU_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_FUCTION */
void IMU_FUCTION(void *argument)
{
  /* USER CODE BEGIN IMU_FUCTION */
  for(;;)
  {
    // if (Flag_TBOFdata) {
    //   Flag_TBOFdata = 0;
    //   printf("X=%.2f Y=%.2f Yaw=%.2f Gz=%.2f\r\n",
    //     TB_position.xdata, TB_position.ydata, imu_yaw, imu_gz);
    // }
    osDelay(10);
  }
  /* USER CODE END IMU_FUCTION */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

