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
#include "HWT101_iic.h"
#include "block_basic.h"
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

// volatile Current_Task_t current_task = Event_IDLE;   /* banyuntask.h 已删除 Current_Task_t, 仅注释代码在用 */
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
#define FIND_CIRCLE_ONLY_TASK 0
#if FIND_CIRCLE_ONLY_TASK
	K230_RequestMode(K230_MODE_LINE);
	K230_ApplyMode();
	printf("[TASK] Find circle only\r\n");

	Color_Init();
	Color_SetLedLevel(5);
	for (;;) {
		Color_DataTypeDef d;
		if (Color_ReadData(&d) == HAL_OK) {
			printf("R=%3d G=%3d B=%3d -> %s\r\n",
				   d.red, d.green, d.blue, Color_ToString(Color_Judge(&d)));
		}
		/* 一次性 dump 原始接收字节, 确认帧格式 */
		osDelay(100);
	}

		// for (uint8_t slot = 1; slot <= 3; slot++) {
		// 	/* 等红外对射检测到物体进入 */
		// 	while (!IR_ObjectEntered()) {
		// 		osDelay(10);
		// 	}
		// 	/* ---- 读颜色过程 (已注释) ---- */
		// 	// Color_DataTypeDef d;
		// 	// if (Color_ReadData(&d) != HAL_OK) { slot--; osDelay(5); continue; }
		// 	// int dr = abs((int)d.red   - g_color_ambient.r);
		// 	// int dg = abs((int)d.green - g_color_ambient.g);
		// 	// int db = abs((int)d.blue  - g_color_ambient.b);
		// 	// if (dr < 30 && dg < 30 && db < 30) { slot--; osDelay(50); continue; }
		// 	if (slot < 4) {
		// 		osDelay(150);
		// 		BlockBasic_TurntableTo(slot + 1);
		//
		// 	}
		// }
		g_trophy_done = 1;
		//Trace_LineFollow();
		// MecanumResult motor;
		// motor = Mecanum_Calc_Full(0.0,0.0,0.5);
		// Send_commandmotor(&motor);
		osDelay(10);
#else
	/*
	 *导航循线任务
	 */
	 //SystemMode_t Navafter_mode[6]={Event_QRCode,Event_LinFolR,Event_PlaceDown,Event_LinFolL,Event_QRCode,Event_FindCircle};
	SystemMode_t Navafter_mode[7]={Event_QRCode,Event_LinFolR,Event_Navigation,Event_PlaceDown,Event_QRCode,Event_LinFolL,Event_FindCircle};
	uint8_t NavafterNum[7]={1,1,1,3,1,1,5};
	//SystemMode_t Navafter_mode[4]={Event_PlaceDown,Event_LinFolL,Event_QRCode,Event_FindCircle};
	// uint8_t NavafterNum[4]={2,1,1,5};
	// SystemMode_t Navafter_mode[1]={Event_PlaceDown};
	// uint8_t NavafterNum[1]={2};
	uint8_t i=0;
	bool flag_finish=false;
	uint8_t P_Nava=0;
	uint8_t rank[3]={second_place,champion,third_place};

	K230_RequestMode(K230_MODE_CIRCLE);
	K230_ApplyMode();
	task_send(Event_Navigation);
	// BlockBasic_LiftTo(UP,43);
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
	  	if (P_Nava<7)
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
	K230_RequestMode(K230_MODE_LINE);
	K230_ApplyMode();
  		Trace_LineFollow();
	osDelay(150);
  		if (g_color_collect_done==1)
  		{
  			task_send(Event_Navigation);
  			Mecanum_StopAll();
  			printf("[TASK] LinFolL done\r\n");
  			K230_RequestMode(K230_MODE_CIRCLE);
  			K230_ApplyMode();
  		}
	osDelay(10);
  	}
  	else if (g_last_cmd.Mode==Event_LinFolR) // 循线右
  	{
  		K230_RequestMode(K230_MODE_LINE);
  		K230_ApplyMode();

  		Trace_LineFollow();
        if (g_trophy_done==1)
        {
	        task_send(Event_Navigation);
        	Mecanum_StopAll();
					BlockBasic_DualArmSetPos(7);
        	printf("[TASK] LinFolR done\r\n");
        	K230_RequestMode(K230_MODE_CIRCLE);
        	K230_ApplyMode();
        }
			osDelay(10);
  	}
	else if (g_last_cmd.Mode==Event_FindCircle) // 找圆
			{
		K230_RequestMode(K230_MODE_CIRCLE);
		K230_ApplyMode();
				g_circle_speed = 1.0f;  /* 左侧快 */
				//加入物料放置转盘逻辑
				if (flag_finish==false)
				{
					flag_finish = true;
				}
				if (flag_finish)
				{

					Circle_Follow();
					if (g_circle_dir=='O')
					{
						TT_RotateByQR();
						osDelay(100);
						Place('O', 8);
						printf("[TASK] FindCircle placed");
						flag_finish = false;
						if (TT_IsDone()) {//全部转完
							printf("[TASK] LIFT servo");
							// Servo_SetAngle(125);
              //BlockBasic_DualArmSetPos(5);
						}
						task_send(Event_Navigation);
					}
				}
			}
else if (g_last_cmd.Mode==Event_PlaceDown)
	  	{
	K230_RequestMode(K230_MODE_CIRCLE);
	K230_ApplyMode();
	g_circle_speed=1.0f;
	  		switch (rank[i])
  			{
  			case champion:// 冠军
  				{
  					//加入奖杯转盘放置逻辑
					if (flag_finish==false)
					{
						/* HEIGHT_CHANGE: podium champion pre-place arm height. */
						BlockBasic_DualArmSetPos(4);
  						
  						// osDelay(500);
  						flag_finish=true;
  					}
					  BlockBasic_TurntableTo(Slop_dirjang(champion));
  					Circle_Follow();
  					if (g_circle_dir=='O')
  					{
  						/* HEIGHT_CHANGE: podium champion place arm height via Place(height=3). */
  						Place('O', 5);
  						BlockBasic_DualArmSetPos(4);
  						printf("[TASK] PlaceDown champion done\r\n");
  						i++;
  						flag_finish=false;
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
						/* HEIGHT_CHANGE: podium second-place pre-place arm height. */
						BlockBasic_DualArmSetPos(7);
  						BlockBasic_TurntableTo(Slop_dirjang(second_place));
  						// osDelay(500);
  						flag_finish=true;
  					}
  					Circle_Follow();
  					if (g_circle_dir=='O')
  					{
  						osDelay(1000);
  						/* HEIGHT_CHANGE: podium second-place place arm height via Place(height=5). */
  						Place('O', 3);
  						printf("[TASK] PlaceDown second done\r\n");
  						i++;
  						flag_finish=false;
							BlockBasic_DualArmSetPos(4);
  						task_send(Event_Navigation);
							
  						/* HEIGHT_CHANGE: podium second-place post-place CH1 arm angle. */
  						// Servo_SetAngle(42);
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
						/* HEIGHT_CHANGE: podium third-place pre-place arm height. */
						BlockBasic_DualArmSetPos(6);
  						BlockBasic_TurntableTo(Slop_dirjang(third_place));
  						osDelay(500);
  						//加入奖杯转盘放置逻辑（已加）
  						flag_finish=true;
  					}
  					Circle_Follow();
  					if (g_circle_dir=='O')
  					{
  						/* HEIGHT_CHANGE: podium third-place place arm height via Place(height=2). */
  						Place('O', 2);
  						printf("[TASK] PlaceDown third done\r\n");
  						i++;
  						flag_finish=false;
  						BlockBasic_DualArmSetPos(8);
  						//BlockBasic_TurntableTo(1);   /* 放完第三个奖杯后转回槽位1，后续收集5个物料也从槽位1开始 */
  						task_send(Event_Navigation);
  						K230_RequestMode(K230_MODE_LINE);
  						K230_ApplyMode();
  						break;
  					}
  				}break;
  			}

  		}

  	osDelay(10);

  		// Circle_Follow();
  		// if (g_circle_dir=='O')
  		// {
  		// 	Place('O', 2);
  		// 	flag_finish=false;
  		// }
  		// task_send(Event_Navigation);
  	}

  	osDelay(10);
#endif
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
	Color_SetLedLevel(5);

	/* 简单校准开关: 1=执行一次校准存 Flash, 0=实时打印 RGB+颜色 */
#define COLOR_SIMPLE_CALIB 0
#if COLOR_SIMPLE_CALIB
	{
		static const Color_TypeDef seq[5] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK};
		static const char *name[5] = {"RED", "GREEN", "BLUE", "WHITE", "BLACK"};

		printf("[CALIB] Remove block, sample ambient in 3s\r\n");
		osDelay(3000);
		Color_CalibAmbient();
		printf("[CALIB] Ambient saved\r\n");

		for (int i = 0; i < 5; i++) {
			printf("[CALIB] >>> Put %s in front, sample in 3s\r\n", name[i]);
			osDelay(3000);
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
		for (;;) {
			Color_DataTypeDef d;
			if (Color_ReadData(&d) == HAL_OK) {
				printf("R=%3d G=%3d B=%3d -> %s\r\n",
				       d.red, d.green, d.blue, Color_ToString(Color_Judge(&d)));
			} else {
				printf("R= ? G= ? B= ? (no data)\r\n");
			}
			/* 一次性 dump 原始接收字节, 确认帧格式 */
			osDelay(100);
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
	uint8_t K = 1;//0为先走物块任务，1为先走奖杯任务
	IR_Init();   /* 红外对射初始化 */
	BlockBasic_DualArmSetPos(8);
	osDelay(1000);
	BlockBasic_TurntableTo(1);
	// BlockBasic_TurntableTo(1);
	// BlockBasic_LiftTo(UP,20);
	// HAL_Delay(1000);
	// BlockBasic_LiftTo(UP,40);
	for(;;)
	{
		osDelay(10);

		if (g_last_cmd.Mode==Event_LinFolL || g_last_cmd.Mode==Event_LinFolR)
		{

			Color_SetLedLevel(0);
			//Color_Init();
			osDelay(50);
#if USE_OPENMV_COLOR == 1  /* ---- GY-33 ---- */
			/* 收集: 红外对射检测物体进入 → 识别记录当前槽位颜色 → 转下一槽位 */
			if (K==0) {
				for (uint8_t slot = 1; slot <= 5; slot++) {
					/* 等红外对射检测到物体进入 */
					while (!IR_ObjectEntered()) {
						osDelay(10);
					}
					if (slot < 6) {
						osDelay(165);
						BlockBasic_TurntableTo(slot+1);
					}
					if (slot <= 4) {
						/* ---- 读颜色并记录到当前槽位 ---- */
						Color_DataTypeDef d;
						if (Color_ReadData(&d) != HAL_OK) { slot--; osDelay(10); continue; }
						int dr = abs((int)d.red   - g_color_ambient.r);
						int dg = abs((int)d.green - g_color_ambient.g);
						int db = abs((int)d.blue  - g_color_ambient.b);
						if (dr < 30 && dg < 30 && db < 30) { slot--; osDelay(50); continue; }
						Color_TypeDef c = Color_Judge(&d);
						TT_SetColor(slot, c);
						printf("[COLLECT] slot=%d, color=%s\r\n", slot, Color_ToString(c));
					}
					else {
						/* 识别完4个槽位后, 剩余最后一个颜色自动记录到槽位5 */
						Color_TypeDef missing = COLOR_UNKNOWN;
						for (Color_TypeDef c = COLOR_RED; c < COLOR_COUNT; c++) {
							uint8_t s;
							for (s = 0; s < 4; s++) {
								if (g_tt.color[s] == c) break;
							}
							if (s == 4) { missing = c; break; }
						}
						if (missing != COLOR_UNKNOWN) {
							TT_SetColor(4, missing);
							printf("[COLLECT] slot=5, color=%s\r\n", Color_ToString(missing));
						} else {
							printf("[COLLECT] slot=5, color=UNKNOWN (no missing found)\r\n");
						}
					}

				}
				osDelay(150);
				K=1;
				g_color_collect_done = 1;
				BlockBasic_TurntableRotate(45.0f);   /* 收集完5个物料后, 转盘再旋转45度 */
			}
			else {
				for (uint8_t slot = 1; slot <= 3; slot++) {
					/* 等红外对射检测到物体进入 */
					while (!IR_ObjectEntered()) {
						osDelay(10);
					}
					/* ---- 读颜色过程 (已注释) ---- */
					// Color_DataTypeDef d;
					// if (Color_ReadData(&d) != HAL_OK) { slot--; osDelay(5); continue; }
					// int dr = abs((int)d.red   - g_color_ambient.r);
					// int dg = abs((int)d.green - g_color_ambient.g);
					// int db = abs((int)d.blue  - g_color_ambient.b);
					// if (dr < 30 && dg < 30 && db < 30) { slot--; osDelay(50); continue; }
					if (slot < 4) {
						osDelay(220);
						BlockBasic_TurntableTo(slot+1);
					}
				}
				K=0;
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
  			printf("QR_result=%d\n",QR_result);
  			//Mecanum_MoveWithEncoder(&g_mecanum_config, 0.1f, 0.0f, 0.0f, 1.0f, 80U, 5000);
  			clr_rank_flag=true;
  		}
  		else
  		{
  			QR_result=Qr_Get();
  			SetQR(QR_result - 1);   /* QR号从1起, T1下标从0起 */
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
	EulerAngle e ;
	AngleCtrl angle_ctrl;
	Angle_Init(&angle_ctrl);


	for(;;)
	{
		// if (Flag_TBOFdata) {
		//   Flag_TBOFdata = 0;
		//   printf("X=%.2f Y=%.2f Yaw=%.2f Gz=%.2f\r\n",
		//     TB_position.xdata, TB_position.ydata, imu_yaw, imu_gz);
		// }
		// MahonyAHRS_Update(0.01f);                      // dt = 5ms
		// siyuan_imu_task();

		/* HWT101零点校准 */
		//printf("Yaw=%.2f\r\n",HWT101_GetZeroYaw());
		// Angle_UpdateTarget(&angle_ctrl, 0.0f);
		// AngleLoop_Update(&angle_ctrl, HWT101_GetZeroYaw(), g_hwt101_gyro_z);
		// Angle_Update(&angle_ctrl, HWT101_GetZeroYaw(), g_hwt101_gyro_z);
		// MecanumResult motor = Mecanum_Calc(0.0f, angle_ctrl.cmd_w);
		// Send_commandmotor(&motor);


		// e = MahonyAHRS_GetEuler_deg();
		osDelay(10);
	}
	/* USER CODE END IMU_FUCTION */
}
