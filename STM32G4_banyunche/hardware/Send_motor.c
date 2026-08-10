#include "Common_used.h"
uint16_t left_vel,right_vel;
uint8_t left_acc,left_dir;
uint8_t right_acc,right_dir;
float motor_v,motor_w;  

//舵机拟合改这个变量front_angle，完成后把为它赋拟合后的常数。
volatile float front_angle=94.35285048f;
extern TIM_HandleTypeDef htim3;
// AckermanResult motor_data;
extern MecanumResult motor_data;

//uint8_t flag_ok;

void Send_commandmotor(MecanumResult *data)
{
	/* 电机编号与物理位置对应（与 Mecanum_Move.h 的 MECANUM_ADDR_* 一致）：
	 *   1=前左(FL)  2=后左(RL)  3=后右(RR)  4=前右(FR) */
	Emm_V5_Vel_Control(1, data->fl_dir, data->fl_speed, 250, 1);  /* 1号=前左 FL */
	HAL_Delay(1);
	Emm_V5_Vel_Control(2, data->rl_dir, data->rl_speed, 250, 1);   /* 2号=后左 RL */
	HAL_Delay(1);
	Emm_V5_Vel_Control(3, data->rr_dir, data->rr_speed, 250, 1);  /* 3号=后右 RR */
	HAL_Delay(1);
	Emm_V5_Vel_Control(4, data->fr_dir, data->fr_speed, 250, 1);  /* 4号=前右 FR */
	HAL_Delay(1);
	Emm_V5_Synchronous_motion(0);
}




