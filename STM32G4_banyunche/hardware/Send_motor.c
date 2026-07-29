#include "stm32g4xx.h" // Device header
#include <stdbool.h>

#include "cmsis_os2.h"
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
	Emm_V5_Vel_Control(1, !data->fr_dir, data->fr_speed, 250, 0);  /* 1号=前右 */
	Emm_V5_Vel_Control(2, data->rl_dir, data->rl_speed, 250, 0);   /* 2号=后左 */
	Emm_V5_Vel_Control(3, !data->fl_dir, data->fl_speed, 250, 0);  /* 3号=前左 */
	Emm_V5_Vel_Control(4, data->rr_dir, data->rr_speed, 250, 0);  /* 4号=后右 */
	osDelay(5);
	Emm_V5_Synchronous_motion(0);
}

void Servo_SetAngle(float Angle)
{
	//ÏÞ·ù

#if use_xing_che

	if(Angle>160){Angle=160;}
	if(Angle<0){Angle=0;}

#else

	if(Angle>160){Angle=160;}
	if(Angle<0){Angle=0;}

#endif
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Angle / 180 * 2000 + 500);

}
#if   use_nanof	

void commands_detect(void)
{
	
	
		left_vel=(
													((uint16_t)Data_uart1[1] << 0)		|
													((uint16_t)Data_uart1[2] << 8)
								 );//
				left_acc=Data_uart1[3];
				left_dir=Data_uart1[4];

				right_vel=(
													((uint16_t)Data_uart1[5] << 0)		|
													((uint16_t)Data_uart1[6] << 8)
								 );//
				right_acc=Data_uart1[7];
				right_dir=Data_uart1[8];
			
				uint8_t *float_ptr = (uint8_t *)&front_angle;
            float_ptr[0] = Data_uart1[9];
            float_ptr[1] = Data_uart1[10];
            float_ptr[2] = Data_uart1[11];
            float_ptr[3] = Data_uart1[12];
	
	
	
}

void shell_print(uint8_t *x)
{
 if(FlagOFMotor == 1)
	{
		commands_detect();
		FlagOFMotor = 0;
		//flag_ok=1;
	}
}

#else

void comamd_detect()
{
	uint8_t * p_motorv=(uint8_t *)&motor_v;
	p_motorv[0]=Data_uart1[1];
	p_motorv[1]=Data_uart1[2];
	p_motorv[2]=Data_uart1[3];
	p_motorv[3]=Data_uart1[4];

	uint8_t * p_motorw=(uint8_t *)&motor_w;
	p_motorw[0]=Data_uart1[5];
	p_motorw[1]=Data_uart1[6];
	p_motorw[2]=Data_uart1[7];
	p_motorw[3]=Data_uart1[8];
}

void shell_print(uint8_t *x)
{
	
	if(FlagOFMotor==1)
	{
		comamd_detect();
	// motor_data=Mecanum_Calc(motor_v,motor_w);
	// #if use_xing_che
	//
	// left_vel=data->left_speed;
	// right_vel=data->right_speed;
	//
	// #else
	// left_vel=data->left_speed;
 //    right_vel=data->right_speed;
	// #endif
	// left_dir=data->left_dir;
	// right_dir=data->right_dir;
	// front_angle=data->servo_pwm;
	FlagOFMotor=0;
	}
	
	
}

#endif  


