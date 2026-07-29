#include "stm32g4xx.h"                  // Device header
#include <string.h>
#include "Common_used.h"
/************************************************************************
����������ģ��IO��ʼ������
��ڲ����� ��
�� �� ֵ�� none
����˵���� ������õ��߿���JQ8x00_Workmode��ֵ��Ϊ1
**************************************************************************/
void JQ8x00_Init()
{
//	GPIO_InitTypeDef GPIO_InitStructure;			//�������æ���͵��߹��ܶ�û�п�������������ʾһ������δʹ�õľ��棬���Խ����д������λ��߲��ù�
	#if JQ8x00_BusyCheck
	RCC_APB2PeriphClockCmd(JQ8x00_BUSY_RCC,ENABLE);	 //ʹ��ʱ��
	
	GPIO_InitStructure.GPIO_Pin  = JQ8x00_BUSY_GPIO_Pin;		//JQ8X00æ���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //��������
 	GPIO_Init(JQ8x00_BUSY_GPIO, &GPIO_InitStructure);
	#endif
	
	#if JQ8x00_Workmode         
    RCC_APB2PeriphClockCmd(JQ8x00_VPP_RCC,ENABLE);
    
	GPIO_InitStructure.GPIO_Pin  = JQ8x00_VPP_GPIO_Pin;						//JQ8x00���߿���IO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(JQ8x00_VPP_GPIO, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.1
	GPIO_SetBits(JQ8x00_VPP_GPIO,JQ8x00_VPP_GPIO_Pin);						 	//PA1 �����
	#endif
}

/************************��������**************************************/
#if JQ8x00_Workmode						//���Ʒ�ʽѡ��

/******************************���߿���********************************/

/************************************************************************
����������	���߿��Ƶ��ֽڷ��ͺ���
��ڲ����� 	DATA����������
�� �� ֵ�� none
����˵���� ����Ҫ���͵�������Ϊ�βδ���
**************************************************************************/
void OneLine_SendData(uint8_t DATA)
{
	uint8_t i;
	
	JQ8x00_VPP_Set();		//��ʼ�ź�
	JQ8x00_us(100);
	JQ8x00_VPP_Clr();
	JQ8x00_ms(3);
	
	for(i=0;i<8;i++)
	{
		JQ8x00_VPP_Set();
		if(DATA & 0x01)
		{
			JQ8x00_us(1500);
			JQ8x00_VPP_Clr();
			JQ8x00_us(500);
		}
		else
		{
			JQ8x00_us(500);
			JQ8x00_VPP_Clr();
			JQ8x00_us(1500);
		}
		DATA >>= 1;
	} 
	
	JQ8x00_VPP_Set();
}

/************************************************************************
����������	�����ֽڿ���
��ڲ����� 	Mode������
�� �� ֵ�� none
����˵���� ����Ҫ�Ĺ�����Ϊ�βδ���
**************************************************************************/
void OneLine_ByteControl(LineByteSelect Mode)
{
	#if JQ8x00_BusyCheck
	while(JQ8x00_BUSY_Read);				//æ���
	#endif
	OneLine_SendData(Mode);
}

/************************************************************************
����������	���߿�����Ϸ��ͺ���
��ڲ����� 	Nume�����֣�Mode������
�� �� ֵ�� none
����˵���� ����Ҫ���͵����ݺ���Ҫ�Ĺ�����Ϊ�βδ���
**************************************************************************/
void OneLine_ZHControl(uint8_t Nume,LineModeSelect Mode)
{
	#if JQ8x00_BusyCheck
	while(JQ8x00_BUSY_Read);				//æ���
	#endif
	OneLine_SendData(0x0a);
	if(Nume < 10)
	{
		OneLine_SendData(Nume);
	}
	else
	{
		OneLine_SendData(Nume/10);
		OneLine_SendData(Nume%10);
	}
	OneLine_SendData(Mode);
}

#else

/**************************���ڿ���************************************/

/************************************************************************
������������ϲ�������
��ڲ����� 	*DAT���ַ���ָ��,Len�ַ�������
�� �� ֵ�� none
����˵���� ����Ҫ�������ļ���������������Ϊ�βμ���
**************************************************************************/




void  JQ8x00_ZuHeBoFang(uint8_t *DATA,uint8_t Len)
{
	uint16_t CRC_data=0,i = 3;
	uint8_t Buffer[ZH_MAX] ={0xaa,0x1b};
	Buffer[2] = Len*2;			//���㳤��
	CRC_data = CRC_data + 0xaa + 0x1b + Buffer[2];
	while(Len--)
	{
		Buffer[i] = *DATA/10+0x30;			//ȡ��ʮλ
		CRC_data = CRC_data + Buffer[i];
		i++;
		Buffer[i] = *DATA%10+0x30;			//ȡ����λ
		CRC_data = CRC_data + Buffer[i];
		i++;
		DATA++;
	}
	Buffer[i] = CRC_data;
	i++;
	#if JQ8x00_BusyCheck
	while(JQ8x00_BUSY_Read);				//æ���
	#endif
	JQ8x00_UART(Buffer,i);
}

/************************************************************************
������������ʼ��-ָ������-���ݳ���-У���
��ڲ����� 	MODE��ģʽ
�� �� ֵ�� none
����˵���� �������ݴ���
**************************************************************************/
void  JQ8x00_Command(UartCommand Command)
{
	uint8_t Buffer[4] ={0xaa};

    Buffer[1] = Command;            //ָ������			
    Buffer[2] = 0x00;           //���ݳ���
    Buffer[3] = Buffer[0] +  Buffer[1] +  Buffer[2];            //У���

	#if JQ8x00_BusyCheck
	while(JQ8x00_BUSY_Read);				//æ���
	#endif
	JQ8x00_UART(Buffer,4);
}

/************************************************************************
������������ʼ��-ָ������-���ݳ���-����-У���
��ڲ����� 	*DAT���ַ���ָ��,Len�ַ�������
�� �� ֵ�� none
����˵���� 
**************************************************************************/
void  JQ8x00_Command_Data(UartCommandData Command,uint8_t DATA)
{
	uint8_t Buffer[6] ={0xaa};
    uint8_t DataLen = 0;
    Buffer[1] = Command;       //ָ������
    if((Command != AppointTrack) && (Command != SetCycleCount) && (Command != SelectTrackNoPlay) && (Command != AppointTimeBack) && (Command != AppointTimeFast))        //ֻ��һ������ָ��    
    {
        Buffer[2] = 1;			//���ݳ���
        Buffer[3] = DATA;       //����
        Buffer[4] = Buffer[0] +  Buffer[1] +  Buffer[2] + Buffer[3];            //У���
        DataLen = 5;
    }
    else                                                                                        //����������ָ�� 
    {
        Buffer[2] = 2;			//���ݳ���
        Buffer[3] = DATA/256;       //����
        Buffer[4] = DATA%256;       //����
        Buffer[5] = Buffer[0] +  Buffer[1] +  Buffer[2] + Buffer[3] + Buffer[4];  
        DataLen = 6;
    }
    
	#if JQ8x00_BusyCheck
	while(JQ8x00_BUSY_Read);				//æ���
	#endif
	JQ8x00_UART(Buffer,DataLen);
}

/************************************************************************
������������������·���µĵ���Ƶ�ļ�
��ڲ�����JQ8X00_Symbol:ϵͳ�̷���*DATA:��Ҫ���ŵ���Ƶ�ļ�·��
�� �� ֵ�� none
����˵��������˵���� �� ��/���/С���ֻ�.mp3,���԰����¸�ʽ
        /���* /С��*???,�����*������ǰ������Ϊ�������ļ��С���*Ϊͨ���
        ע���ʽ����һ��Ŀ¼����ǰҪ��*����/����1* /����2* /00002*???
        JQ_8x00_RandomPathPlay(JQ8X00_FLASH,"���* /С��")
        ����FLASH��Ŀ¼���ļ���Ϊ00001.mp3����Ƶ����JQ_8x00_RandomPathPlay(JQ8X00_FLASH,"/00001");
**************************************************************************/
void JQ8x00_RandomPathPlay(JQ8X00_Symbol symbol,char *DATA)
{
    uint8_t Buffer[52] ={0xaa,0x08};
    uint8_t i,j;
    Buffer[2] = 1 + strlen(DATA) + 4;       //����,1Ϊ�̷���4Ϊ*������
    Buffer[3] = symbol;        //�̷�
    i = 4;
    while(*DATA)
    {
       Buffer[i++] =  *DATA;
        DATA++;
    }
    Buffer[i++] = '*';
    Buffer[i++] = '?';
    Buffer[i++] = '?';
    Buffer[i++] = '?';
    for(j=0;j<i;j++)
    {
        Buffer[i] = Buffer[i] + Buffer[j];      //����У���
    }
    i++;
    JQ8x00_UART(Buffer,i);
}


void Send_commendyu()
{
	if(buffer_flag)
		{
			uint8_t buf_temp = buf;
			buffer_flag = 0;
			
			taskENTER_CRITICAL();
			#if use_xing_che
			
			JQ8x00_RandomPathPlay(JQ8X00_SD,(buffer[buf_temp]));//�³�������Ƶ
			#else
			
			JQ8x00_RandomPathPlay(JQ8X00_FLASH,(buffer[buf_temp]));//�ϳ�������Ƶ
			#endif
			
			
			taskEXIT_CRITICAL();//�˳��ٽ���
	
		}
}



#endif




















