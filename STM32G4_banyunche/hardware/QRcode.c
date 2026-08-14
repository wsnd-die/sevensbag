#include "Common_used.h"
#include "HWT101_iic.h"

static uint8_t qrcode_rx_buf[QRCODE_RX_BUF_SIZE];
static volatile uint16_t qrcode_rx_len=0;
uint8_t QR_Flag;
uint8_t Jang_Num,Yan_Num;

void QRcode_Start(void)
{
    /* USART1 已改为 HWT101 专用。 */
}

void QRcode_Clear(void)
{
    qrcode_rx_len = 0;
    memset(qrcode_rx_buf, 0, sizeof(qrcode_rx_buf));
}

/* 从串口直接写入二维码数据 (USART2 扫码模块), 置 QR_Flag */
void QRcode_SetData(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0) return;
    if (len > QRCODE_RX_BUF_SIZE) len = QRCODE_RX_BUF_SIZE;
    memset(qrcode_rx_buf, 0, sizeof(qrcode_rx_buf));
    memcpy(qrcode_rx_buf, buf, len);
    qrcode_rx_len = len;
    QR_Flag = 1;
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
    (void)data;
    (void)len;
    (void)timeout;
    return HAL_ERROR;
}




uint8_t Qr_Get(void) {

    /* 阻塞等扫码; 收到十六进制数字串(如 31 0D → 1, 30 0D → 0)后识别 */
    while (Read_QrFlag()==0) {
        osDelay(50);
    }
    return QR_deel();
}


uint8_t  QR_deel(void)
{
    uint8_t P;
    static uint8_t 	result=0,k=0;

    uint8_t cnt = qrcode_rx_len;
    if (cnt > 3) cnt = 3;
    for(uint8_t i=0;i<cnt;i++)
    {
        P=qrcode_rx_buf[i];
        if(P==0x0D || P<'0' || P>'9')
        {
            break;
        }
        result=result*10+(P-0x30);
    }
    if (k==0)
    {
        Jang_Num=result-1;
    }
    else
    {
        Yan_Num=result-1;
    }

    uint8_t ret = result;
    result = 0;
    return ret;
}
uint8_t Read_QrFlag()
{
    uint8_t i;
    i=QR_Flag;
    QR_Flag=0;
    return i;
}

uint8_t Slop_dirjang(Jang_type jang)
{
    uint8_t i;
    for (i=0;i<3;i++)
    {
        if (T2[Jang_Num][i]==jang)
        {
            return 3-i;
        }
    }
    return 0;  /* 未匹配 */
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART1) {
        HWT101_UART_RxCpltCallback(huart);
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

    HWT101_UART_ErrorCallback(huart);
}
