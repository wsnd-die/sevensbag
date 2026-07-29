#include "stm32g4xx.h"                  // Device header
#include "Common_used.h"
#include "waypoint.h"
#include "mecanum.h"

uint8_t rx1,rx3;
uint8_t FlagOFMotor;
uint8_t pPack_uart1;
//==DMA接收数据==//
uint8_t dma_write_index;
uint8_t soft_read_index;
uint8_t parse_uart1[RX_BUF_SIZE];
uint8_t Data_uart1[25];


#if ni_he_mode


void Uart1_deel()
{
	static uint8_t state;
	if(state==0)
		{
			if(rx1==0x55)
			{
				pPack_uart1=5;
				Data_uart1[pPack_uart1]=rx1;
				state=1;



			}
			else if(rx1==0xFA)
			{
				state=0;


			}

		}
	else if(state==1)
		{

			if(rx1==0xFA)
			{

				state=0;
				memcpy((uint8_t *)&front_angle,&Data_uart1[1],4);
			}
			else
		{
			pPack_uart1--;
			Data_uart1[pPack_uart1]=rx1;
			if(pPack_uart1<1)
			{
				state=0;
				pPack_uart1=5;
			}

		}

		}
}




#else

void Uart1_deel()
{
	static uint8_t state;
	if(state==0)
		{
			if(rx1==0x55)
			{
				Data_uart1[pPack_uart1]=rx1;
				if(FlagOFMotor==0)
				{
					pPack_uart1=0;
					state=1;
				}
			}
			else if(rx1==0xFA)
			{
				state=0;
				FlagOFMotor=1;

			}

		}
	else if(state==1)
		{

			if(rx1==0xFA)
			{
				FlagOFMotor=1;
				state=0;

			}
			else
		{
			pPack_uart1++;
			Data_uart1[pPack_uart1]=rx1;
			if(pPack_uart1>10)
			{
				state=0;
				pPack_uart1=0;
			}

		}

		}
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Uart1_deel();
    HAL_UART_Receive_IT(&huart1, &rx1, 1);
  }
  if (huart->Instance == USART3)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  /* DEBUG: 收到UART3字节翻转LED */
    Uart3_deel();
    HAL_UART_Receive_IT(&huart3, &rx3, 1);
  }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_FLAG(&huart1,
            UART_CLEAR_OREF |
            UART_CLEAR_FEF |
            UART_CLEAR_NEF);
        HAL_UART_Receive_IT(&huart1, &rx1, 1);
    }
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_FLAG(&huart3,
            UART_CLEAR_OREF |
            UART_CLEAR_FEF |
            UART_CLEAR_NEF);
        HAL_UART_Receive_IT(&huart3, &rx3, 1);
    }
}