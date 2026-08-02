#include "k230.h"
#include "usart.h"
#include <string.h>

static uint8_t k230_dma_rx,Trace_dataFlag=0;
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
    Trace_dataFlag = 0;
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

uint8_t Read_TraceFlag()
{
	return Trace_dataFlag;
}

/**
 * @brief  从 K230 接收缓冲区读取循迹偏移数据
 * @param  data: 输出缓冲区 (至少 2 字节)，[0]=int16低字节, [1]=int16高字节
 * @note   数据包格式: 0xA3 0xB3 [int16_lo] [int16_hi] 0xFF
 *         调用前应先通过 Read_TraceFlag() 确认有新数据
 */
void Read_Tracedata(uint8_t * data)
{
    if (data == NULL || k230_rx_len < 5) {
        return;
    }
    /* 验证包头 0xA3 0xB3 */
    if (k230_rx_buf[0] != 0xA3 || k230_rx_buf[1] != 0xB3) {
        return;
    }
    /* 提取 int16 数据（小端序）：buf[2]=低字节, buf[3]=高字节 */
    data[0] = k230_rx_buf[2];
    data[1] = k230_rx_buf[3];
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
			k230_rx_len = k230_DataP;    /* 记录完整包长度 */
			Trace_dataFlag=1;
		}
	}
	
	
	
}


#if !LEGACY_USART2_ODOM_ENABLE
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
       
			K230_xDeel();
			
			
			HAL_UART_Receive_IT(&huart2, &k230_dma_rx, 1);
    }

    
}
#endif
