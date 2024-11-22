#include "stdio.h"
#include "usart.h"	
#include "string.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "message_buffer.h"

#define UART1_RX_BUF_LEN 512
#define UART1_TX_BUF_LEN 512
char Uart1_Rx_Buf[UART1_RX_BUF_LEN];
char Uart1_Tx_Buf[UART1_TX_BUF_LEN];

//����ʹ�ð�����ģʽ    
__asm (".global __use_no_semihosting\n\t");   
void _sys_exit(int x) 
{ 
	x = x; 
} 

void _ttywrch(int ch)
{
    ch = ch;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}
/* FILE �� stdio.h���涨��. */
FILE __stdout;

#if defined ( __GNUC__ ) && !defined (__clang__) 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 

//�ض���fputc���� 
PUTCHAR_PROTOTYPE
{
	while((USART1->SR&0X40)==0);//�ȴ���һ֡���ݷ������  
	USART1->DR = (uint8_t)ch;
	return ch;
}


/**********************************����1*************************************/
//bound:������
void UART1_Config(uint32_t bound){
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART1);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1 �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;	//GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//����
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;										//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);										//��ʼ������1

	//USART1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10;		//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;			//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);								//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	//��������ж�

	USART_Cmd(USART1, ENABLE);						//ʹ�ܴ���1
}

void DMA_UART1_TX_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	//����DMAʱ�� 
    DMA_DeInit(DMA2_Stream7);
    while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}		//�ȴ�stream�����ã���DMAy_SxCR.EN��Ϊ0
    //����Stream
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//��8��channel��ѡ��һ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;       //�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart1_Tx_Buf;			//�洢��0��ַ��˫����ģʽ��Ҫʹ��M1AR
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;            		//�洢��������ģʽ RX TX��ͬ������赽��ַ���ߵ�ַ������
    DMA_InitStructure.DMA_BufferSize = UART1_TX_BUF_LEN;                	//���ݴ�������������������Ϊ��λ 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ���ֲ���
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�洢����ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������λ��:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�洢������λ��:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                         	//��ͨģʽ(��ѭ��ģʽ��Ӧ)
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //��ֹFIFOģʽ         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	//�������ж�(1)
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//���DMA2_Steam7������ɱ�־(2)
		
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	//����DMA��ʽ����
	DMA_Cmd(DMA2_Stream7, DISABLE);					//����DMA���� 
}

extern SemaphoreHandle_t uart1SendSemaphore;

void DMA2_Stream7_IRQHandler(void)
{
	//�����־
	if(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET)//�ȴ�DMA2_Steam7�������
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
		xSemaphoreGiveFromISR(uart1SendSemaphore, NULL);
	}
}

void DMA_USART1_Send(char *data, uint16_t size)
{
	xSemaphoreTake(uart1SendSemaphore, portMAX_DELAY);
	memcpy(Uart1_Tx_Buf, data, size);					//�������ݵ�DMA���ͻ�����
	
	DMA_Cmd(DMA2_Stream7, DISABLE);						//��DMA����������ʼ����
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	//ȷ��DMA���Ա�����
	DMA_SetCurrDataCounter(DMA2_Stream7, size);			//�������ݴ��䳤��
	DMA_Cmd(DMA2_Stream7, ENABLE);						//��DMA����������ʼ����
}

void DMA_UART1_RX_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	//����DMAʱ�� 
    DMA_DeInit(DMA2_Stream5);
    while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){}		//�ȴ�stream�����ã���DMAy_SxCR.EN��Ϊ0
    //����Stream
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//��8��channel��ѡ��һ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;       //�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart1_Rx_Buf;			//�洢��0��ַ��˫����ģʽ��Ҫʹ��M1AR
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;            		//�洢��������ģʽ RX TX��ͬ������赽��ַ���ߵ�ַ������
    DMA_InitStructure.DMA_BufferSize = UART1_RX_BUF_LEN;                	//���ݴ�������������������Ϊ��λ
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //�����ַ���ֲ���
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //�洢����ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //��������λ��:8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //�洢������λ��:8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //��ͨģʽ(��ѭ��ģʽ��Ӧ)
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //��ֹFIFOģʽ
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //���δ���
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //���δ���
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//����DMA��ʽ����
	DMA_Cmd(DMA2_Stream5, ENABLE);					//����DMA����(3)
}


extern MessageBufferHandle_t usart1MessageBuffer;
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //�����ж�
	{
		USART1->SR;
        USART1->DR;
		
		xMessageBufferSendFromISR(usart1MessageBuffer, Uart1_Rx_Buf, UART1_RX_BUF_LEN - DMA2_Stream5->NDTR, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		
		memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);
		DMA_Cmd(DMA2_Stream5, DISABLE);
		while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
		DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
		DMA_Cmd(DMA2_Stream5, ENABLE);
	} 
}

void USART1_Init(void) {
	UART1_Config(115200);
	DMA_UART1_TX_Config();
	DMA_UART1_RX_Config();
}


void USART_SendString(USART_TypeDef* USARTx, char *DataString){
	USART_ClearFlag(USARTx,USART_FLAG_TC);									//�����ַ�ǰ��ձ�־λ������ȱʧ�ַ����ĵ�һ���ַ���
	while(*DataString != '\0')												//�ַ���������
	{
		USART_SendData(USARTx,*DataString);									//ÿ�η����ַ�����һ���ַ�
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC) == 0);				//�ȴ����ݷ��ͳɹ�
		USART_ClearFlag(USARTx,USART_FLAG_TC);								//�����ַ�����ձ�־λ
		DataString++;
	}
}