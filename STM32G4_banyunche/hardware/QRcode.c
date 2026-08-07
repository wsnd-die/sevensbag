#include "Common_used.h"

static uint8_t qrcode_rx_byte;
static uint8_t qrcode_rx_buf[QRCODE_RX_BUF_SIZE];
static volatile uint16_t qrcode_rx_len=0;
uint8_t QR_Flag;
Qr_Yantitl Qr_YanBiao[16] = {
    /* 数字 1-3: 完整 5 色 */
    [0]  = { .Color_xu = { COLOR_BLACK, COLOR_WHITE, COLOR_RED,   COLOR_GREEN, COLOR_BLUE  } },
    [1]  = { .Color_xu = { COLOR_WHITE, COLOR_BLACK, COLOR_RED,   COLOR_GREEN, COLOR_BLUE  } },
    [2]  = { .Color_xu = { COLOR_WHITE, COLOR_BLACK, COLOR_GREEN, COLOR_RED,   COLOR_BLUE  } },
    /* 数字 4-16: 4 色 + 第 5 位补充 */
    [3]  = { .Color_xu = { COLOR_BLUE,  COLOR_WHITE, COLOR_BLACK, COLOR_RED,    COLOR_GREEN } },
    [4]  = { .Color_xu = { COLOR_WHITE, COLOR_RED,   COLOR_BLUE,  COLOR_BLACK,  COLOR_GREEN } },
    [5]  = { .Color_xu = { COLOR_BLACK, COLOR_RED,   COLOR_BLUE,  COLOR_WHITE,  COLOR_GREEN } },
    [6]  = { .Color_xu = { COLOR_BLUE,  COLOR_GREEN, COLOR_BLACK, COLOR_WHITE,  COLOR_RED   } },
    [7]  = { .Color_xu = { COLOR_GREEN, COLOR_WHITE, COLOR_BLUE,  COLOR_BLACK,  COLOR_RED   } },
    [8]  = { .Color_xu = { COLOR_WHITE, COLOR_GREEN, COLOR_BLACK, COLOR_BLUE,   COLOR_RED   } },
    [9]  = { .Color_xu = { COLOR_BLACK, COLOR_RED,   COLOR_BLUE,  COLOR_GREEN,  COLOR_WHITE } },
    [10] = { .Color_xu = { COLOR_RED,   COLOR_BLUE,  COLOR_GREEN, COLOR_BLACK,  COLOR_WHITE } },
    [11] = { .Color_xu = { COLOR_GREEN, COLOR_RED,   COLOR_BLACK, COLOR_BLUE,   COLOR_WHITE } },
    [12] = { .Color_xu = { COLOR_WHITE, COLOR_RED,   COLOR_BLUE,  COLOR_GREEN,  COLOR_BLACK } },
    [13] = { .Color_xu = { COLOR_RED,   COLOR_GREEN, COLOR_WHITE, COLOR_BLUE,   COLOR_BLACK } },
    [14] = { .Color_xu = { COLOR_BLUE,  COLOR_WHITE, COLOR_GREEN, COLOR_RED,    COLOR_BLACK } },
    [15] = { .Color_xu = { COLOR_GREEN, COLOR_BLUE,  COLOR_RED,   COLOR_WHITE,  COLOR_BLACK } },
};


Qr_Jantitl Qr_JanBiao[6] = {
    [0] = { .Jang = { champion,     second_place, third_place  } },  /* A B C */
    [1] = { .Jang = { champion,     third_place,  second_place } },  /* A C B */
    [2] = { .Jang = { second_place, champion,     third_place  } },  /* B A C */
    [3] = { .Jang = { second_place, third_place,  champion     } },  /* B C A */
    [4] = { .Jang = { third_place,  champion,     second_place } },  /* C A B */
    [5] = { .Jang = { third_place,  second_place, champion     } },  /* C B A */
};
void QRcode_Start(void)
{
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);
}

void QRcode_Clear(void)
{
    qrcode_rx_len = 0;
    memset(qrcode_rx_buf, 0, sizeof(qrcode_rx_buf));
}

const uint8_t *QRcode_GetBuffer(uint16_t *len)
{
    if (len != NULL) {
        *len = qrcode_rx_len;
    }
    return qrcode_rx_buf;
}



HAL_StatusTypeDef QRcode_Send(const uint8_t *data, uint16_t len, uint32_t timeout)
{
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, len, timeout);
}




void Qr_Get() {

    while (Read_QrFlag()==0) {
        osDelay(50);
    }

     QR_deel();
}


void  QR_deel()
{
    uint8_t P;
    static uint8_t 	result=0;

    for(i=0;i<3;i++)
    {
        P=qrcode_rx_buf[i];
        if(P==0x0D)
        {
            break;
        }
        result=result*10+(P-0x30);
    }
    HAL_UART_Transmit_IT(&huart1,&result ,1 );
    result=0;


}
uint8_t Read_QrFlag()
{
    uint8_t i;
    i=QR_Flag;
    QR_Flag=0;
    return i;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART1) {
        if (qrcode_rx_len >= QRCODE_RX_BUF_SIZE) {
            qrcode_rx_len = 0;
        }

        qrcode_rx_buf[qrcode_rx_len++] = qrcode_rx_byte;
        if(qrcode_rx_byte==0x0d)
        {
            QR_Flag=1;
            qrcode_rx_len=0;
        }

        HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);
    }
    else if (huart->Instance == USART3) {
        K230_RxProcessByte();
    }

}
void QRcode_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        K230_RxRestart();
        return;
    }
    if (huart->Instance != USART1) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    HAL_UART_Receive_IT(&huart1, &qrcode_rx_byte, 1);
}
