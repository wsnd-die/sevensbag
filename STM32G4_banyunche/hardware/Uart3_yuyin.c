#include "stm32g4xx.h"                  // Device header
#include "Common_used.h"
#include "waypoint.h"

uint8_t FlagOFYuyin,rx3;
uint8_t pPack_uart3;
uint8_t Data_uart3[20];
uint32_t Data_buffer;

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