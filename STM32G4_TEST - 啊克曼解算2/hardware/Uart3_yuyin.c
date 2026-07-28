#include "stm32g4xx.h"                  // Device header
#include "Common_used.h"
#include "waypoint.h"

uint8_t FlagOFYuyin;
uint8_t pPack_uart3;
uint8_t Data_uart3[20];
uint32_t Data_buffer;

char Path_Buffer [10]  = {"/00001"};
char Path_Buffer1[10]  = {"/00002"};
char Path_Buffer2[10]  = {"/00003"};
char Path_Buffer3[10]  = {"/00004"};
char Path_Buffer4[10]  = {"/00005"};
char Path_Buffer5[10]  = {"/00006"};
char Path_Buffer6[10]  = {"/00007"};
char Path_Buffer7[10]  = {"/00008"};
char Path_Buffer8[10]  = {"/00009"};
char Path_Buffer9[10]  = {"/00010"};
char Path_Buffer10[10] = {"/00011"};
char Path_Buffer11[10] = {"/00012"};
char Path_Buffer12[10] = {"/00013"};
char Path_Buffer13[10] = {"/00014"};
char Path_Buffer14[10] = {"/00015"};
char Path_Buffer15[10] = {"/00016"};
char Path_Buffer16[10] = {"/00017"};
char Path_Buffer17[10] = {"/00018"};
char Path_Buffer18[10] = {"/00019"};
char Path_Buffer19[10] = {"/00020"};
char Path_Buffer20[10] = {"/00021"};
char Path_Buffer21[10] = {"/00022"};
char Path_Buffer22[10] = {"/00023"};
char *buffer[23] = {Path_Buffer,Path_Buffer1,Path_Buffer2,Path_Buffer3,Path_Buffer4,
					Path_Buffer5,Path_Buffer6,Path_Buffer7,Path_Buffer8,Path_Buffer9,
					Path_Buffer10,Path_Buffer11,Path_Buffer12,Path_Buffer13,Path_Buffer14,
					Path_Buffer15,Path_Buffer16,Path_Buffer17,Path_Buffer18,Path_Buffer19,Path_Buffer20,Path_Buffer21,Path_Buffer22};


/*
 * UART3 字节接收状态机
 *
 * 支持两种格式:
 *   1) 帧格式:  [0x5F] [cmd] [0xFB]     → Data_uart3[0]=0x5F, [1]=cmd
 *   2) 裸字节:  直接发 'r' / 's' 等       → 自动包装为帧
 */
void Uart3_deel()
{
	static uint8_t state;

	if (state == 0)
	{
		if (rx3 == 0x5F)
		{
			/* 帧头: 进入帧接收模式 */
			Data_uart3[0] = rx3;
			if (FlagOFYuyin == 0)
			{
				state = 1;
				pPack_uart3 = 0;
			}
		}
		else
		{
			/* 裸字节: 直接当作命令, 自动包装 */
			Data_uart3[0] = 0x5F;
			Data_uart3[1] = rx3;
			FlagOFYuyin = 1;
		}
	}
	else if (state == 1)
	{
		if (rx3 == 0xFB)
		{
			/* 帧尾: 帧接收完成 */
			FlagOFYuyin = 1;
			state = 0;
		}
		else
		{
			/* 数据字节 */
			pPack_uart3++;
			Data_uart3[pPack_uart3] = rx3;
			if (pPack_uart3 > 5)
			{
				state = 0;
			}
		}
	}
}


/*
 * 命令解析 (由 shell_print3 调用, 或 task 直接 switch Data_uart3[1])
 */
uint8_t buffer_flag, buf;

void commands_detect3(void)
{
	buffer_flag = 1;

	switch (Data_uart3[1])
	{
		/* ---- 路径录制控制 ---- */
		case 'r':
			WaypointNav_StartRecord(&g_waypoint_nav);
			break;
		case 's':
			WaypointNav_StopRecord(&g_waypoint_nav);
			break;
		default: break;
	}

	Data_buffer = 0;
}


void shell_print3(uint8_t *x)
{
	if (FlagOFYuyin == 1)
	{
		commands_detect3();
		FlagOFYuyin = 0;
	}
}