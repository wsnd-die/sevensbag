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
#include "HWT101_iic.h"
#include "stdlib.h"
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
volatile uint8_t         g_color_collect_done = 0;  /* 0=未完成, 1=5个槽已收集完 */
volatile uint8_t         g_trophy_done = 0;         /* 0=未完成, 1=3个奖杯槽已收集完 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityHigh,
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
  .priority = (osPriority_t) osPriorityAboveNormal7,
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
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 256 * 4
};
/* Definitions for QRFUNION */
osThreadId_t QRFUNIONHandle;
const osThreadAttr_t QRFUNION_attributes = {
  .name = "QRFUNION",
  .priority = (osPriority_t) osPriorityAboveNormal6,
  .stack_size = 256 * 4
};
/* Definitions for IMU_TASK */
osThreadId_t IMU_TASKHandle;
const osThreadAttr_t IMU_TASK_attributes = {
  .name = "IMU_TASK",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 8
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

		// if(cmd.k==1)
		// {
		 	g_last_cmd = cmd;
		// 	switch(cmd.Mode)
		// 	{
		// 	case Event_LinFolL:
		// 		if (current_task == Event_Task1) break;
		// 		current_task = Event_Task1; break;
		// 	case Event_LinFolR:
		// 		if (current_task == Event_Task2) break;
		// 		current_task = Event_Task2;  break;
		// 	case Event_Navigation:    break;
		// 	case Event_GoHome:
		// 		current_task = Event_Task2;  break;
		// 	case Event_PickUp: case Event_PlaceDown: case Event_STEERING_ROTATE:
		// 		break;
		// 	case Event_QRCode: break;
		// 	case Event_FindCircle:  break;
		// 	case Event_STOP:  current_task = Event_IDLE; break;
		// 	default: break;
		// 	}
		// }
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
	// SystemMode_t Navafter_mode[6]={Event_QRCode,Event_LinFolR,Event_PlaceDown,Event_LinFolL,Event_QRCode,Event_FindCircle};
	// uint8_t NavafterNum[6]={1,3,1,1,1,5};
	SystemMode_t Navafter_mode[1]={Event_FindCircle};
	uint8_t NavafterNum[1]={4};
	uint8_t i=0;
	bool flag_finish=false;
	uint8_t P_Nava=0;
	uint8_t rank[3]={second_place,champion,third_place};

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)(89.5f / 180.0f * 2000.0f + 500.0f)); // 后级
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(81.0f / 180.0f * 2000.0f + 500.0f)); //前级

	K230_SetMode(K230_MODE_LINE);
	K230_ApplyMode();
	task_send(Event_LinFolR);
	BlockBasic_TurntableTo(1);
	osDelay(500);

  /* Infinite loop */
  for(;;)
  {
  	if (g_last_cmd.Mode==Event_Navigation)
  	{
  		Nav_FeDuanPoint();
  		printf("[TASK] Navigation done, P_Nava=%d\r\n", P_Nava);
  		//Nav_MoveForward(0.5);
  		//Nav_MoveLeft(-0.5);
  		if (P_Nava<1)
			{
				task_send(Navafter_mode[P_Nava]);
				NavafterNum[P_Nava]--;
	  			if (NavafterNum[P_Nava]==0) {
	  				P_Nava++;
	  			}
			}
  		else
			{
  			    task_send(Event_GoHome);
				Mecanum_StopAll();  /* 全部任务跑完, 停车 */
			}

	  	}
else if (g_last_cmd.Mode==Event_LinFolL)
  	{
  		Trace_LineFollow();
  		if (g_color_collect_done==1)
  		{
  			task_send(Event_GoHome);
  			Mecanum_StopAll();
  			printf("[TASK] LinFolL done\r\n");
  		}
	osDelay(10);
  	}
  	else if (g_last_cmd.Mode==Event_LinFolR)
  	{
  		Trace_LineFollow();
        if (g_trophy_done==1)
        {
	        task_send(Event_GoHome);
        	Mecanum_StopAll();
        	printf("[TASK] LinFolR done\r\n");
        }
  	}
	else if (g_last_cmd.Mode==Event_FindCircle)
		{
			//加入物料放置转盘逻辑
			if (flag_finish==false)
				{
					flag_finish = true;   /* 先锁住, 防止 osDelay 期间重复进入 */
					TT_RotateByQR();
				}
				Circle_Follow();
			if (g_circle_dir=='O')
			{
				Place('O');
				printf("[TASK] FindCircle done");
				flag_finish=false;
				task_send(Event_Navigation);
			}
		}
else if (g_last_cmd.Mode==Event_PlaceDown)
	  	{
	  		if (flag_finish==false)
	  		{
	  			BlockBasic_LiftTo(UP, 20);  /* 先升丝杆, flag_finish 由 switch 内管理 */
	  		}
	  		switch (rank[i])
  			{
  			case champion:
  				{
  					//加入奖杯转盘放置逻辑
  					if (flag_finish==false)
  					{
  						BlockBasic_TurntableTo(Slop_dirjang(champion));
  						// osDelay(500);
  						flag_finish=true;
  					}
  					Circle_Follow();
  					if (g_circle_dir=='O')
  					{
  						Place('O');
  						printf("[TASK] PlaceDown champion done\r\n");
  						i++;
  						BlockBasic_LiftTo(UP,20);
  						flag_finish=false;
  						task_send(Event_Navigation);
  						break;
  					}

  				}

  			case  second_place:
  				{
  					//丝干在走进亚军时要先升起，在寻线完进行了升起

  					//加入奖杯转盘放置逻辑
  					if (flag_finish==false)
  					{
  						BlockBasic_TurntableTo(Slop_dirjang(second_place));
  						// osDelay(500);
  						flag_finish=true;
  					}
  					Circle_Follow();
  					if (g_circle_dir=='O')
  					{
  						Place('O');
  						printf("[TASK] PlaceDown second done\r\n");
  						i++;
  						flag_finish=false;
  						task_send(Event_Navigation);
  						break;
  					}
  					//在走到冠军前要先升起来
  				}
  			case third_place:
  				{
  					if (flag_finish==false)
  					{
  						BlockBasic_LiftTo(DOWN,20);
  						BlockBasic_TurntableTo(Slop_dirjang(third_place));
  						osDelay(500);
  						//加入奖杯转盘放置逻辑（已加）
  						flag_finish=true;
  					}
  					Circle_Follow();
  					if (g_circle_dir=='O')
  					{
  						Place('O');
  						printf("[TASK] PlaceDown third done\r\n");
  						task_send(Event_Navigation);
  						break;
  					}
  				}
  			}

  		}

  	osDelay(10);

  		// Circle_Follow();
  		// if (g_circle_dir=='O')
  		// {
  		// 	Place('O');
  		// 	flag_finish=false;
  		// }
  		// task_send(Event_Navigation);
  	}

  	osDelay(10);
}
  /* USER CODE END NLF_TASK */

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
	Color_Init();
#define COLOR_CALIB 1
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

	HAL_UART_Transmit(&huart2, (uint8_t *)"=== EMPTY slot ===\r\n", 20, 100);
	osDelay(1000);
	Color_CalibAmbient();
	HAL_UART_Transmit(&huart2, (uint8_t *)"Ambient OK\r\n", 13, 100);

	for (int i = 0; i < 5; i++) {
		int n = snprintf(msg, sizeof(msg), "=== Slot %d: %s ===\r\n", i+1, steps[i+1]);
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, n, 100);

		if (BlockBasic_TurntableTo(i+1) == BLOCK_OK) {
			// osDelay(2000);
			Color_DataTypeDef d;
			if (Color_ReadData(&d) == HAL_OK) {
				n = snprintf(msg, sizeof(msg), "  L=%d A=%d B=%d\r\n", d.l, d.a, d.b);
				HAL_UART_Transmit(&huart2, (uint8_t *)msg, n, 100);
			}
			Color_Calibrate(colors[i]);
			n = snprintf(msg, sizeof(msg), "Slot %d OK\r\n", i+1);
		} else {
			n = snprintf(msg, sizeof(msg), "Slot %d FAIL\r\n", i+1);
		}
		HAL_UART_Transmit(&huart2, (uint8_t *)msg, n, 100);
		osDelay(500);
	}
	Color_CalibSave();
	HAL_UART_Transmit(&huart2, (uint8_t *)"=== SAVED ===\r\n", 14, 100);

  for(;;) { osDelay(1000); }

#else
  for(;;)
  {
		char b[400]; int n = 0;

		/* 实时 Lab + 颜色 */
		Color_DataTypeDef d;
		if (Color_ReadData(&d) == HAL_OK) {
			Color_TypeDef c = Color_Judge(&d);
			n += snprintf(b+n, sizeof(b)-n, "Blk=%d L=%d A=%d B=%d -> %s | ",
				d.l, d.red, d.a, d.b, Color_ToString(c));
		} else {
			n += snprintf(b+n, sizeof(b)-n, "Lab=? | ");
		}

		n += snprintf(b+n, sizeof(b)-n, "\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)b, n, 100);
		osDelay(50);
  }
#endif
#endif
	for (;;)
	{
		// printf("TRACE: ang=%.1f posX=%.1f v=%.2f w=%.2f | CIRCLE: dir=%c vx=%.2f vy=%.2f\r\n",
		//        imu_yaw, g_trace_posx, g_trace_v, g_trace_w,
		//        g_circle_dir, g_circle_vx, g_circle_vy);
		{
			Color_DataTypeDef data;
			Color_TypeDef c;
			// if (Color_ReadData(&data) == HAL_OK) {
			// 	c = Color_Judge(&data);
			// 	printf("[OpenMV] raw=%d | color=%d (%s)\r\n",
			// 	       data.sensor_color, (int)c, Color_ToString(c));
			// }
		}


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
	uint8_t K = 1;//0为先走物块任务，1为先走奖杯任务
	Servo_SetAngle(38);
	// BlockBasic_TurntableTo(1);
	// BlockBasic_LiftTo(UP,20);
	// HAL_Delay(1000);
	// BlockBasic_LiftTo(UP,40);
	for(;;)
	{
		osDelay(10);

		if (g_last_cmd.Mode==Event_LinFolL||g_last_cmd.Mode==Event_LinFolR)
		{

			Color_SetLedLevel(0);
			BlockBasic_TurntableTo(1);
			osDelay(1000);
			Color_Init();
			HAL_Delay(50);
#if USE_OPENMV_COLOR == 1  /* ---- GY-33 ---- */
			if (K==0) {
				/* 逐槽位检测：RGB跳变→有物块→旋转；无跳变→环境光→停止 */
				for (uint8_t slot = 1; slot <= 5; slot++) {
					Color_DataTypeDef d;
					if (Color_ReadData(&d) != HAL_OK) {
						slot--;
						osDelay(5);   /* 等下一帧到达再重试 */
						continue;
					}
					/* 计算和环境光基线的 RGB 差值（绝对值之和） */
					int dr = abs((int)d.red   - g_color_ambient.r);
					int dg = abs((int)d.green - g_color_ambient.g);
					int db = abs((int)d.blue  - g_color_ambient.b);
					if (dr < 30 && dg < 30 && db < 30) {
						/* 跳变太小 → 环境光/空槽 → 停止 */
						slot--;
						osDelay(50);
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
				K=1;
				g_color_collect_done = 1;
			}
			else {
				/* 逐槽位检测：RGB跳变→有物块→旋转；无跳变→环境光→停止 */
				for (uint8_t slot = 1; slot <= 3; slot++) {
					Color_DataTypeDef d;
					if (Color_ReadData(&d) != HAL_OK) {
						slot--;
						osDelay(5);   /* 等下一帧到达再重试 */
						continue;
					}
					/* 计算和环境光基线的 RGB 差值（绝对值之和） */
					int dr = abs((int)d.red   - g_color_ambient.r);
					int dg = abs((int)d.green - g_color_ambient.g);
					int db = abs((int)d.blue  - g_color_ambient.b);
					if (dr < 30 && dg < 30 && db < 30) {
						/* 跳变太小 → 环境光/空槽 → 停止 */
						slot--;
						osDelay(50);
						continue;
					}
					/* 跳变明显 → 有物块 → 判断颜色 → 旋转 */
					Color_TypeDef c = Color_Judge(&d);
					//TT_SetColor(slot - 1, c);
					if (slot < 3) {
						if (BlockBasic_TurntableTo(slot + 1) != BLOCK_OK) break;
						HAL_Delay(500);  /* 等待舵机转到位 */
					}
				}
				K=0;
				g_trophy_done = 1;
			}
#else  /* ---- OpenMV ---- */
				{
					#define JUMP_AB   20
					#define JUMP_LBLK 60
					#define JUMP_LMEAN 16
					static uint8_t p_lblk, p_lmean, p_a, p_b, p_init;
					if (!p_init) { Color_DataTypeDef id; if (Color_ReadData(&id) == HAL_OK) { p_lblk=id.l; p_lmean=id.red; p_a=id.a; p_b=id.b; p_init=1; } }
					if (K==0 && !g_color_collect_done&&g_last_cmd.Mode==Event_LinFolL) {
						for (uint8_t slot = 1; slot <= 5; slot++) {
							for (;;) {
								Color_DataTypeDef d;
								if (Color_ReadData(&d) != HAL_OK) { osDelay(5); continue; }
								int da = abs((int)d.a - p_a), db = abs((int)d.b - p_b);
								int dlblk = abs((int)d.l - p_lblk), dlmean = abs((int)d.red - p_lmean);
								p_lblk=d.l; p_lmean=d.red; p_a=d.a; p_b=d.b;
								int has_jump = (da+db>=JUMP_AB) || (dlblk>=JUMP_LBLK) || (dlmean>=JUMP_LMEAN);
								if (has_jump) {
									Color_TypeDef c = Color_Judge(&d);
									int only_lmean = (dlmean>=JUMP_LMEAN) && (da+db<JUMP_AB) && (dlblk<JUMP_LBLK);
									if (only_lmean ? (c==COLOR_WHITE) : (c!=COLOR_UNKNOWN))
										{ TT_SetColor(slot-1,c); break; }
								}
								osDelay(5);
							}
							if (slot < 5) {
								if (BlockBasic_TurntableTo(slot+1) != BLOCK_OK) break;
								osDelay(900);
							}
						}
						Servo_Angle(333.0f);
						K=1; g_color_collect_done=1;
					}
					else if (!g_trophy_done && g_last_cmd.Mode==Event_LinFolR) {
						for (uint8_t slot = 1; slot <= 3; slot++) {
							for (;;) {
								Color_DataTypeDef d;
								if (Color_ReadData(&d) != HAL_OK) { osDelay(5); continue; }
								int dlblk = (int)p_lblk - d.l;       /* 黑占比降→白 */
								int dlmean = (int)d.red - p_lmean;  /* 亮度升→白 */
								p_lblk=d.l; p_lmean=d.red;
								int has_jump = (dlblk>=JUMP_LBLK) || (dlmean>=JUMP_LMEAN);
								if (has_jump) {
									Color_TypeDef c = Color_Judge(&d);
									if (c == COLOR_WHITE) break;
								}
								osDelay(5);
							}
							if (slot < 3) {
								if (BlockBasic_TurntableTo(slot+1) != BLOCK_OK) break;
								osDelay(900);
							}
						}
							BlockBasic_LiftTo(UP, 20);
						Servo_Angle(180.0f);
						K=0; g_trophy_done=1;
					}
				}
				#endif /* USE_OPENMV_COLOR */

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
  	// printf("yaw:%.1f\n",siyuan_yaw*RAD_TO_DEG);

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

  	// if (g_last_cmd.Mode==Event_FindCircle)  {  Circle_Follow(); }
  	// printf("yaw=%.2f pitch=%.2f roll=%.2f\r\n",
			//  siyuan_yaw * RAD_TO_DEG,
			//  siyuan_pitch * RAD_TO_DEG,
			//  siyuan_roll * RAD_TO_DEG);
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
	uint8_t QR_result=0,i=0;
	bool clr_rank_flag=false;
	// SystemMode_t  QR_after[2]={Event_LinFolR,Event_Navigation};
  /* Infinite loop */
  for(;;)
  {

  	if (g_last_cmd.Mode==Event_QRCode)
  	{
  		if (clr_rank_flag==false)
  		{
  			QR_result=Qr_Get();
  			clr_rank_flag=true;
  		}
  		else
  		{
  			QR_result=Qr_Get();
  			SetQR(QR_result);
  		}
  		task_send(Event_Navigation);
  	}

    osDelay(10);
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
  char hwt101_msg[48];

  for(;;)
  {
    if (g_hwt101_data_ready) {
      int len = snprintf(hwt101_msg, sizeof(hwt101_msg),
                         "HWT101 yaw=%.2f deg\r\n",
                         (double)HWT101_GetZeroYaw());
      if (len > 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)hwt101_msg,
                          (uint16_t)len, 20U);
      }
    }
    osDelay(100);
  }
  /* USER CODE END IMU_FUCTION */
}
