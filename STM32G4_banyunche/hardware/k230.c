#include "k230.h"
#include "usart.h"
#include <string.h>

static uint8_t k230_dma_rx;
static uint8_t k230_rx_buf[K230_RX_BUF_SIZE];
static volatile uint16_t k230_rx_len;

#ifndef LEGACY_USART2_ODOM_ENABLE
#define LEGACY_USART2_ODOM_ENABLE 0
#endif

typedef enum{
	trace_data,
	color_data,
	verify,
	over,

}K230;


void K230_Start(void)
{
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart2, &k230_dma_rx, K230_RX_BUF_SIZE);
}

void K230_Clear(void)
{
    k230_rx_len = 0;
    memset(k230_rx_buf, 0, sizeof(k230_rx_buf));
}

const uint8_t *K230_GetBuffer(uint16_t *len)
{
    if (len != NULL) {
        *len = k230_rx_len;
    }
    return k230_rx_buf;
}

HAL_StatusTypeDef K230_Send(const uint8_t *data, uint16_t len, uint32_t timeout)
{
    return HAL_UART_Transmit(&huart2, (uint8_t *)data, len, timeout);
}


void K230_xDeel()
{
	static K230 StateSho;
	static uint8_t k230_DataP;
	if(StateSho==over)
	{
		if(k230_dma_rx==0xA3)
		{
			k230_DataP=0;
			StateSho=verify;
			k230_rx_buf[k230_DataP++]=k230_dma_rx;
		}
	}
	else if(StateSho==verify)
	{
		if(k230_dma_rx==0xB3)
		{
			StateSho=trace_data;
		}
			
	}
	
	 if(StateSho==trace_data)
	{
		
		k230_rx_buf[k230_DataP++]=k230_dma_rx;
		if(k230_dma_rx==0xff)
		{
			StateSho=over;
			
		}
	}
	
	
	
}


#if !LEGACY_USART2_ODOM_ENABLE
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
       
			
			
			
			HAL_UART_Receive_IT(&huart2, &k230_dma_rx, K230_RX_BUF_SIZE);
    }

    
}
#endif
