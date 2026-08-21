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
/* 灰度循迹开关: 1=左循迹用灰度传感器(PA4 CLK / PA5 DAT), 0=用 K230 摄像头 */
#ifndef GRAY_TRACE_ENABLE
#define GRAY_TRACE_ENABLE 0
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

volatile TaskCommand_t   g_last_cmd;
volatile uint8_t         g_color_collect_done = 0;  /* 0=未完成, 1=5个槽已收集完 */
volatile uint8_t         g_trophy_done = 0;         /* 0=未完成, 1=3个奖杯槽已收集完 */

/* --- 颜色读取握手: BsRt_task 检测到物块到位 → 置 g_color_req 请求读色,
 *    Color_task 读到有效色写槽位 → 清 g_color_req 应答 --- */
volatile uint8_t g_color_req      = 0;   /* 1=有读色请求待处理 */
volatile uint8_t g_color_req_slot = 0;   /* 待读槽索引 (0~3), 配合 g_color_req */
static uint8_t   s_seen[COLOR_COUNT];   /* 已读颜色去重标记 (收集阶段共享) */

/* --- 角度控制任务 (FC_TASK) --- */
volatile uint8_t g_angle_ctrl_enable = 0;    /* 1=使能角度控制, 0=停止 */
volatile float
g_angle_target_yaw   = 90.0f; /* 目标角度 (deg), 固定值可改 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
GrayTrace_t g_gray_trace;   /* 灰度循迹控制器 (PA4 CLK / PA5 DAT) */

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
  .priority = (osPriority_t) osPriorityAboveNormal7,
  .stack_size = 512 * 6
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
		g_last_cmd = cmd;
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
	SystemMode_t Navafter_mode[6]={Event_QRCode,Event_LinFolL,Event_FindCircle,Event_QRCode,Event_LinFolR,Event_PlaceDown};
	  // SystemMode_t Navafter_mode[6]={Event_QRCode,Event_LinFolR,Event_PlaceDown};
	// uint8_t NavafterNum[6]={1,1,3};
	 uint8_t NavafterNum[6]={1,1,5,1,1,3};
	//SystemMode_t Navafter_mode[4]={Event_PlaceDown,Event_LinFolL,Event_QRCode,Event_FindCircle};
	// uint8_t NavafterNum[4]={2,1,1,5};
	// SystemMode_t Navafter_mode[1]={Event_PlaceDown};
	// uint8_t NavafterNum[1]={2};
	uint8_t i=0;
	bool flag_finish=false;
	uint8_t P_Nava=0;
	uint8_t rank[3]={second_place,champion,third_place};

	K230_RequestMode(K230_MODE_LINEL);
	K230_ApplyMode();
	task_send(Event_Navigation);
	// BlockBasic_LiftTo(UP,44);
	// Nav_MoveForward(0.5);
	// osDelay(2000);
	// Nav_MoveForward(-0.5);
	BlockBasic_TurntableTo(1);
	// Emm_V5_Vel_Control(1, 0, 100, 250, 0);
	// Emm_V5_Vel_Control(2, 0, 100, 250, 0);
	// Emm_V5_Vel_Control(3, 0, 100, 250, 0);
	// Emm_V5_Vel_Control(4, 0, 100, 250, 0);
	osDelay(100);
  /* Infinite loop */
  for(;;)
  {
	  if (g_last_cmd.Mode==Event_Navigation)
	  {
	  	Nav_FeDuanPoint();
	  	printf("[TASK] Navigation done, P_Nava=%d\r\n", P_Nava);
	  	//Nav_MoveForward(0.5);
	  	//Nav_MoveLeft(-0.5);
	  	if (P_Nava<6)
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
	g_angle_ctrl_enable = 0;   /* 循迹不用角度闭环, 关闭 */
#if GRAY_TRACE_ENABLE
	GrayTrace_Update(&g_gray_trace);   /* 灰度循迹 (PA4 CLK / PA5 DAT) */
	/* 串口打印 digital (每 100ms), 便于调灰度读取/极性/方向 */
	{
	// 	static uint32_t s_gray_print = 0U;
	// 	uint32_t now = HAL_GetTick();
	// 	if (now - s_gray_print >= 100U) {
	// 		uint8_t d = Grayscale_Get_Digital(&g_gray_trace.sensor);
	// 		s_gray_print = now;
	// 		printf("GRAY d=0x%02X [%c%c%c%c%c%c%c%c]\r\n",
	// 		       d,
	// 		       (d & 0x80) ? '0' : '_', (d & 0x40) ? '0' : '_',
	// 		       (d & 0x20) ? '0' : '_', (d & 0x10) ? '0' : '_',
	// 		       (d & 0x08) ? '0' : '_', (d & 0x04) ? '0' : '_',
	// 		       (d & 0x02) ? '0' : '_', (d & 0x01) ? '0' : '_');
	// 	}
	}
#else
	Trace_SetSide(0);          /* 左循迹: 角度环用 ANGLE_KP/KI/KD */
	K230_RequestMode(K230_MODE_LINEL);
	K230_ApplyMode();
  		Trace_LineFollow(Event_LinFolL);
#endif
  		if (g_color_collect_done==1)
  		{
  			task_send(Event_Navigation);
  			Mecanum_StopAll();
  			{
  			    World_Dir_t p = World_position_get();
  			    printf("[POS] LinFolL  x=%.3f y=%.3f yaw=%.2f\r\n",
  			           (double)p.x, (double)p.y, (double)(p.yaw * RAD_TO_DEG));
  			}
  			Nav_CalibrateAfterTrace(false);   /* 物料循迹 → 校准a点 */
  			K230_RequestMode(K230_MODE_CIRCLE);
  			K230_ApplyMode();
  			g_color_collect_done=0;
  		}
	osDelay(10);
  	}
  	else if (g_last_cmd.Mode==Event_LinFolR)
  	{
  		g_angle_ctrl_enable = 0;   /* 循迹不用角度闭环, 关闭 */
  		Trace_SetSide(1);          /* 右循迹: 角度环用 ANGLE_KP_R/KI_R/KD_R */
  		K230_RequestMode(K230_MODE_LINER);
  		K230_ApplyMode();
  		Trace_LineFollow(K230_MODE_LINER);
        if (g_trophy_done==1)
        {
	        task_send(Event_Navigation);
        	Mecanum_StopAll();
        	{
        	    World_Dir_t p = World_position_get();
        	    printf("[POS] LinFolR  x=%.3f y=%.3f yaw=%.2f\r\n",
        	           (double)p.x, (double)p.y, (double)(p.yaw * RAD_TO_DEG));
        	}
        	Nav_CalibrateAfterTrace(true);    /* 奖杯循迹 → 校准亚军点 */
        	K230_RequestMode(K230_MODE_CIRCLE);
        	K230_ApplyMode();
        	g_color_collect_done=0;
        }
			osDelay(10);
  	}
	else if (g_last_cmd.Mode==Event_FindCircle)
			{
		K230_RequestMode(K230_MODE_CIRCLE);
		K230_ApplyMode();
				g_circle_speed = 1.0f;  /* 左侧快 */

		if (g_circle_dir!='O')
		{
			Circle_Follow();
		}
			if (g_circle_dir=='O')
			{
				TT_RotateByQR();
				Place('O', g_circle_avg_x, g_circle_avg_y, 0);
				g_circle_dir=' ';
				printf("[TASK] FindCircle placed");
				flag_finish = false;
				if (TT_IsDone()) {
					printf("[TASK] LIFT servo");
					// Servo_SetAngle(125);
				}
				World_Reset();                    /* 放置完物块: 清零里程计, 奖杯段从0重新记 */
				task_send(Event_Navigation);
			}
			}
else if (g_last_cmd.Mode==Event_PlaceDown)
	  	{
	K230_RequestMode(K230_MODE_CIRCLE);
	K230_ApplyMode();
	g_circle_speed=1.0f;
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
  					if (g_circle_dir!='O')
  					{
  						Circle_Follow();
  					}
  					if (g_circle_dir=='O')
  					{
						Place('O', g_circle_avg_x, g_circle_avg_y, 33);
  						printf("[TASK] PlaceDown champion done\r\n");
  						i++;
  						flag_finish=false;
  						g_circle_dir=' ';
  						task_send(Event_Navigation);
  						break;
  					}
  				}
	  			break;
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
  					if (g_circle_dir!='O')
  					{
  						Circle_Follow();
  					}
  					if (g_circle_dir=='O')
  					{
						Place('O', g_circle_avg_x, g_circle_avg_y, 28);
  						printf("[TASK] PlaceDown second done\r\n");
  						i++;
  						flag_finish=false;
  						BlockBasic_LiftTo(UP,48);
  						task_send(Event_Navigation);
  						g_circle_dir=' ';
  						osDelay(800);
  						break;
  					}
  					//在走到冠军前要先升起来
  				}
	  			break;
  			case third_place:
  				{
  					if (flag_finish==false)
  					{
  						BlockBasic_LiftTo(DOWN,14);
  						BlockBasic_TurntableTo(Slop_dirjang(third_place));
  						//加入奖杯转盘放置逻辑（已加）
  						flag_finish=true;
  					}
  					if (g_circle_dir!='O')
  					{
  						Circle_Follow();
  					}
  					if (g_circle_dir=='O')
  					{
  						printf("[TASK] PlaceDown third done\r\n");
  						Place('O', g_circle_avg_x, g_circle_avg_y, 21);
  						i++;
  						flag_finish=false;
  						g_circle_dir=' ';
  						task_send(Event_Navigation);
  						break;
  					}
  				}break;
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
	Color_SetLedLevel(5);
	Color_Init();

	IR_Init();

	/* 简单校准开关: 1=执行一次校准存 Flash, 0=实时打印 RGB+颜色 */
#define COLOR_SIMPLE_CALIB 0
#if COLOR_SIMPLE_CALIB
	{
		static const Color_TypeDef seq[5] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK};
		static const char *name[5] = {"RED", "GREEN", "BLUE", "WHITE", "BLACK"};

		printf("[CALIB] Remove block, sample ambient in 3s\r\n");
		osDelay(1000);
		Color_CalibAmbient();
		printf("[CALIB] Ambient saved\r\n");

		for (int i = 0; i < 5; i++) {
			printf("[CALIB] >>> Put %s in front, sample in 3s\r\n", name[i]);
			osDelay(1000);
			Color_DataTypeDef d;
			if (Color_ReadData(&d) == HAL_OK) {
				Color_Calibrate(seq[i]);
				printf("[CALIB] %s: R=%3d G=%3d B=%3d saved\r\n",
				       name[i], d.red, d.green, d.blue);
			} else {
				printf("[CALIB] %s: no data!\r\n", name[i]);
			}
		}
		Color_CalibSave();
		printf("[CALIB] Saved to Flash\r\n");
		for (;;) { osDelay(1000); }
	}
#else
	{
		// uint8_t raw_dumped = 0;
		// for (;;) {
		// 	Color_DataTypeDef d;
		// 	if (Color_ReadData(&d) == HAL_OK) {
		// 		printf("R=%3d G=%3d B=%3d -> %s (cb=%lu, rxState=%d, IR=%d)\r\n",
		// 			   d.red, d.green, d.blue, Color_ToString(Color_Judge(&d)),
		// 			   (unsigned long)dbg_rx_cb, (int)huart2.RxState,
		// 			   IR_ObjectPresent());
		// 	} else {
		// 		printf("R= ? G= ? B= ? (no data, cb=%lu, rxState=%d, IR=%d)\r\n",
		// 			   (unsigned long)dbg_rx_cb, (int)huart2.RxState,
		// 			   IR_ObjectPresent());
		// 	}
		// 	if (!raw_dumped && dbg_rx_cb > 0) {
		// 		raw_dumped = 1;
		// 		printf("raw: ");
		// 		for (uint8_t i = 0; i < DMA_RX_BUF_SIZE; i++) {
		// 			printf("%02X ", dma_rx_buf[i]);
		// 		}
		// 		printf("\r\n");
		// 	}
		// 	// /* 调试: 打印5个槽的颜色, 验证槽位映射是否正确 */
		// 	printf("[COLLECT] slot1=%s slot2=%s slot3=%s slot4=%s slot5=%s\r\n",
		// 		   Color_ToString(ColorAtSlot(0)),
		// 		   Color_ToString(ColorAtSlot(1)),
		// 		   Color_ToString(ColorAtSlot(2)),
		// 		   Color_ToString(ColorAtSlot(3)),
		// 		   Color_ToString(ColorAtSlot(4)));
		// 	osDelay(1000);
		// }
		for (;;) {
			if (g_color_req) {
				uint8_t slot = g_color_req_slot;   /* 先取槽, 防握手期被改 */
				Color_TypeDef c = COLOR_UNKNOWN;

				/* 读取槽slot颜色(转盘静止): 带重试上限(约5s), 读不出不阻塞 */
				for (uint16_t tries = 0; tries < 50; tries++) {
					c = Collect_ReadColor();
					if (c != COLOR_UNKNOWN && c < COLOR_COUNT && !s_seen[c])
						break;
					osDelay(10);
				}
				if (c != COLOR_UNKNOWN && c < COLOR_COUNT && !s_seen[c]) {
					s_seen[c] = 1;
					TT_SetColor(slot, c);      /* 槽索引 0~3 直接写 */
				}
				g_color_req = 0;               /* 读色完成应答 */
			}
			osDelay(5);
		}

	}
#endif

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
	uint8_t K = 0;//0为先走物块任务，1为先走奖杯任务
	Servo_SetAngle(41);
	BlockBasic_TurntableTo(1);
	IR_Init();
	GrayTrace_Init(&g_gray_trace);   /* 灰度循迹控制器初始化 (PA4 CLK / PA5 DAT) */
	// BlockBasic_TurntableTo(1);
	// BlockBasic_LiftTo(UP,20);
	// HAL_Delay(1000);
	// BlockBasic_LiftTo(UP,40);
	for(;;)
	{
		osDelay(10);

		if (g_last_cmd.Mode==Event_LinFolL||g_last_cmd.Mode==Event_LinFolR)
		{

#if USE_OPENMV_COLOR == 0  /* ---- GY-33 ---- */
			/* 收集: 红外对射检测物体进入 → 读颜色 → 旋转转盘 */
			if (K==0) {
				g_color_collect_done = 0;
				for (uint8_t i = 0; i < COLOR_COUNT; i++) s_seen[i] = 0;   /* 清已读颜色标记 */
				Color_TypeDef c;
				uint8_t slot;

				g_color_req = 0;				/* 清残余请求 */
				Collect_WaitEnter();			/* 物块1完全进入槽1 (IR 脉冲0→1) */
				BlockBasic_TurntableTo(2);		/* 槽1 → 传感器下 (已完全进入, 无需落稳延时) */


				for (slot = 1; slot <= 4; slot++) {
					while (!IR_ObjectEntered()) osDelay(5);   /* 等物块完全进入 (IR 脉冲0→1) */
					g_color_req_slot = slot - 1;
					g_color_req = 1;
					// while (g_color_req) osDelay(3);   /* 等 Color_task 读色完成 */

					if (slot < 4) {
						BlockBasic_TurntableTo(slot + 2);	/* 槽slot+1 → 传感器下 (直接转, 无需落稳延时) */
					}
				}

				osDelay(200);
				Servo_Angle(333.0f);
				for (c = COLOR_RED; c < COLOR_COUNT; c++) {
					if (!s_seen[c]) { TT_SetColor(4, c); break; }
				}

				K=1;
				g_color_collect_done = 1;
				osDelay(200);
			}
			else {
				g_trophy_done = 0;
				BlockBasic_TurntableTo(1);
				for (uint8_t slot = 1; slot <= 3; slot++) {
					Collect_WaitEnter();
					if (slot < 3) {
						BlockBasic_TurntableTo(slot + 1);
					}
				}
				K=0;
				Servo_Angle(180.0f);
				BlockBasic_LiftTo(UP, 48);
				g_trophy_done = 1;
			}
#else  /* ---- OpenMV ---- */
				{
					#define JUMP_AB   25
					#define JUMP_LBLK 50
					#define JUMP_LMEAN 20
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
								osDelay(20);
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
						for (uint8_t slot = 1; slot <=3; slot++) {
							for (;;) {
								Color_DataTypeDef d;
								if (Color_ReadData(&d) != HAL_OK) { osDelay(5); continue; }
								int dlblk = (int)p_lblk - d.l;       /* 黑占比降→白 */
								int dlmean = (int)d.red - p_lmean;  /* 亮度升→白 */
								p_lblk=d.l; p_lmean=d.red;
								int has_jump = (dlmean>=JUMP_LMEAN) || (dlblk>=JUMP_LBLK);
								if (has_jump) {
									Color_TypeDef c = Color_Judge(&d);
									if (c == COLOR_WHITE) break;
								}
								osDelay(20);
							}
							if (slot < 3) {
								if (BlockBasic_TurntableTo(slot+1) != BLOCK_OK) break;
								osDelay(200);
							}
						}
						BlockBasic_LiftTo(UP, 43);
						Servo_Angle(180.0f);
						K=0; g_trophy_done=1; p_init=0; /* p_init重置，避免白值污染物块采集baseline */
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
	World_Dir_t position;
  /* USER CODE BEGIN OLED_TASK */

  for(;;)
  {
 //  	if (can_error_count) printf("CAN err cnt=%lu code=0x%lX\r\n",
	// (unsigned long)can_error_count, (unsigned long)can_error_code);
  	uint8_t raw_dumped = 0;
  	position=World_position_get();
  	// printf("xyyawixiy:%2f,%2f,%2f,%.2f,%.2f\r\n",position.x,position.y,position.yaw,g_ins.x,g_ins.y);
  	// printf("yaw:%.1f\n",siyuan_yaw*RAD_TO_DEG);
  		// printf("[COLLECT] slot1=%s slot2=%s slot3=%s slot4=%s slot5=%s\r\n",
  		// 	   Color_ToString(ColorAtSlot(0)),
  		// 	   Color_ToString(ColorAtSlot(1)),
  		// 	   Color_ToString(ColorAtSlot(2)),
  		// 	   Color_ToString(ColorAtSlot(3)),
  		// 	   Color_ToString(ColorAtSlot(4)));

  		// Color_DataTypeDef d;
  		// if (Color_ReadData(&d) == HAL_OK) {
  		// 	printf("R=%3d G=%3d B=%3d -> %s (cb=%lu, rxState=%d, IR=%d)\r\n",
  		// 		   d.red, d.green, d.blue, Color_ToString(Color_Judge(&d)),
  		// 		   (unsigned long)dbg_rx_cb, (int)huart2.RxState,
  		// 		   IR_ObjectPresent());
  		// } else {
  		// 	printf("R= ? G= ? B= ? (no data, cb=%lu, rxState=%d, IR=%d)\r\n",
  		// 		   (unsigned long)dbg_rx_cb, (int)huart2.RxState,
  		// 		   IR_ObjectPresent());
  		// }
  		// if (!raw_dumped && dbg_rx_cb > 0) {
  		// 	raw_dumped = 1;
  		// 	printf("raw: ");
  		// 	for (uint8_t i = 0; i < DMA_RX_BUF_SIZE; i++) {
  		// 		printf("%02X ", dma_rx_buf[i]);
  		// 	}
  		// 	printf("\r\n");
  		// }
  		// osDelay(100);
		osDelay(20);
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
	 *角度控制任务
	 *  g_angle_ctrl_enable = 1 时使能:
	 *    AngleCtrl 串级PID (角度环→角速度环), 持续维持 g_angle_target_yaw 角度 (deg)
	 *    用 siyuan_yaw 做角度反馈, imu660ra_gyro_x 做角速度反馈
	 *  g_angle_ctrl_enable = 0 时停止输出
	*/
	AngleCtrl ac;
	Angle_Init(&ac);
	bool running = false;

  /* Infinite loop */
  for(;;)
  {
		if (g_angle_ctrl_enable) {
			if (!running) {
				/* 首次使能: 设置目标并启动 */
				Angle_SetTarget(&ac, g_angle_target_yaw);
				running = true;
			}
			/* 持续维持: 每次刷新目标(不重置PID), 保持 MOVING 状态, 抗扰动 */
			Angle_UpdateTarget(&ac, g_angle_target_yaw);
			Angle_Update(&ac,
			             siyuan_yaw * RAD_TO_DEG,
			             siyuan_gyro_z_rate);   /* 用已 LPF + 减零偏的偏航率, 替代原始陀螺 */
			MecanumResult motor = Mecanum_Calc(0.0f, -ac.cmd_w);
			Send_commandmotor(&motor);
		} else {
			if (running) {
				Mecanum_StopAll();
				running = false;
			}
		}
		osDelay(10);   /* 与 AngleCtrl DT=0.01 匹配 */
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
	uint32_t s_last_print=0;
	// SystemMode_t  QR_after[2]={Event_LinFolR,Event_Navigation};
  /* Infinite loop */
  for(;;)
  {

  	if (g_last_cmd.Mode==Event_QRCode)
  	{
  		/* 每次都 SetQR: 否则 cnt 保持 0, 找圆时 TT_RotateByQR 直接 return false */
  		QR_result=Qr_Get();
  		printf("[QR] got %d\r\n", QR_result);   /* 调试: 确认扫码是否真的收到 */
  		SetQR(QR_result-1);
  		task_send(Event_Navigation);
  	}
  		// printf("[COLLECT] slot1=%s slot2=%s slot3=%s slot4=%s slot5=%s\r\n",
  		// 	   Color_ToString(ColorAtSlot(0)),
  		// 	   Color_ToString(ColorAtSlot(1)),
  		// 	   Color_ToString(ColorAtSlot(2)),
  		// 	   Color_ToString(ColorAtSlot(3)),
  		// 	   Color_ToString(ColorAtSlot(4)));
	  // {
  	// 	uint32_t now = HAL_GetTick();
  	// 	if (now - s_last_print >= 100U) {
  	// 		//Grayscale_Update(&g_gray_trace.sensor);   /* 必须触发串行读取 */
  	// 		uint8_t d = Grayscale_Get_Digital(&g_gray_trace.sensor);
  	// 		s_last_print = now;
  	// 		printf("GRAY d=0x%02X [%c%c%c%c%c%c%c%c] \r\n",
			// 		 d,
			// 		 (d & 0x80) ? '0' : '_', (d & 0x40) ? '0' : '_',
			// 		 (d & 0x20) ? '0' : '_', (d & 0x10) ? '0' : '_',
			// 		 (d & 0x08) ? '0' : '_', (d & 0x04) ? '0' : '_',
			// 		 (d & 0x02) ? '0' : '_', (d & 0x01) ? '0' : '_');
	  // }
	  // }

    osDelay(500);
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
	// EulerAngle e ;
	// calibrate_gyro();
	// Ins_Init();                    /* 惯导初始化 (清零状态, 记录起始 tick) */

  for(;;)
  {
    // if (Flag_TBOFdata) {
    //   Flag_TBOFdata = 0;
    //   printf("X=%.2f Y=%.2f Yaw=%.2f Gz=%.2f\r\n",
    //     TB_position.xdata, TB_position.ydata, imu_yaw, imu_gz);
    // }
  	// MahonyAHRS_Update(0.01f);                      // dt = 5ms
  	siyuan_imu_task();
  	// Ins_Update();
  	// e = MahonyAHRS_GetEuler_deg();
    osDelay(5);
  }
  /* USER CODE END IMU_FUCTION */
}
