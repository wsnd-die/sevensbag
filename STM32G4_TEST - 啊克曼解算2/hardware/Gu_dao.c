#include "stm32g4xx.h"                  // Device header
#include "math.h"   
#include "Common_used.h"   

#define DEG_TO_RAD  0.017453292519943295f
#define GRAVITY     9.80665f
#define PI          3.14159265358979323846f

float V_x = 0.0f;
float V_y = 0.0f;

float acc_bias_x = 0.0f;
float acc_bias_y = 0.0f;

void Guan_dao_Reset(void)
{
    V_x = 0.0f;
    V_y = 0.0f;
}

void Guan_dao(float DT)
{
    float yaw_rad;
    float ax;
    float ay;
    float acc_world_x;
    float acc_world_y;

    if (DT <= 0.0f || DT > 0.05f)
    {
        return;
    }

    yaw_rad = imu_data.yaw * DEG_TO_RAD;

    /*
       去掉静止零偏，再从 g 转成 m/s^2
    */
    ax = (imu_data.ax - acc_bias_x) * GRAVITY;
    ay = (imu_data.ay - acc_bias_y) * GRAVITY;

    /*
       车体坐标转世界坐标
    */
    acc_world_x = -ay * cosf(yaw_rad) + ax * sinf(yaw_rad);
    acc_world_y = -ay * sinf(yaw_rad) - ax * cosf(yaw_rad);

    /*
       死区，静止附近不要积分
    */
    if (fabsf(acc_world_x) < 0.15f)
    {
        acc_world_x = 0.0f;
    }

    if (fabsf(acc_world_y) < 0.15f)
    {
        acc_world_y = 0.0f;
    }

    V_x += acc_world_x * DT;
    V_y += acc_world_y * DT;

    printf("DT=%.4f ax=%.4f ay=%.4f awx=%.4f awy=%.4f Vx=%.3f Vy=%.3f\r\n",
           (double)DT,
           (double)imu_data.ax,
           (double)imu_data.ay,
           (double)acc_world_x,
           (double)acc_world_y,
           (double)V_x,
           (double)V_y);
}




void guan_init()
{
	IMU660RC_Init();
	
	
	
}










