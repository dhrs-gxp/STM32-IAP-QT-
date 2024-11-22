#include "usart.h"	
#include "string.h"
#include "stdlib.h"
#include "flash.h"
#include "boot.h"

#define UART1_RX_BUF_LEN 1024
#define UART1_TX_BUF_LEN 512
char Uart1_Rx_Buf[UART1_RX_BUF_LEN];
char Uart1_Tx_Buf[UART1_TX_BUF_LEN];

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;		//��ռ���ȼ�1
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	//�������ж�(1)
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//���DMA2_Steam7������ɱ�־(2)
		
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	//����DMA��ʽ����
	DMA_Cmd(DMA2_Stream7, DISABLE);					//����DMA���� 
}

volatile int DMA2_Stream7_Send_Over = 1;

void DMA2_Stream7_IRQHandler(void)
{
	//�����־
	if(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET)//�ȴ�DMA2_Steam7�������
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
		DMA2_Stream7_Send_Over = 1;
	}
}

void DMA_USART1_Send(char *data, uint16_t size)
{
	while(DMA2_Stream7_Send_Over == 0);
	DMA2_Stream7_Send_Over = 0;
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


volatile uint8_t UART1_IDLE_IRQ = 0;
volatile uint8_t BOOT_MODE = 0;
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	uint8_t data = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //���տ����ж�
	{
		data = USART1->SR;
        data = USART1->DR;
		
		if(BOOT_MODE == 1) UART1_IDLE_IRQ = 1;			//�����boot�Ĺ̻��������У�ÿ�ο����жϽ�UART1_IDLE_IRQ��־λ��1�����������д���
		else {											//�����û�н���boot�Ĺ̻������ܣ�ÿ�ν��յ���Ϣ������Ϣ�ӻ�����ɾ��
			memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);
			DMA_Cmd(DMA2_Stream5, DISABLE);
			DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
		}
	} 
} 

enum State {commandRecv, sizeRecv, appDownload, executeApp};	//BOOT������״̬���ֱ��ǽ���ָ������ļ���С�������ļ����ݣ�����app����
enum State uartState = commandRecv;

struct AppRecvInfo {				//����app����ʱ��һЩ����
	uint32_t writeAddress;			//������flash�еĻ���ַ
	uint32_t totalDataSize;			//�������ݵ��ܴ�С
	uint32_t receivedtotalSize;		//�Ѿ�д�뵽flash�����ݴ�С
	uint32_t receivedDataSize;		//����д��ʱ���յ������ݴ�С
};

struct AppRecvInfo appRecvInfo = {APP_BASE_ADDR, 0, 0, 0};

//�̻���˳�����Ƚ���Downloadָ��ٽ��ճ����С���ٽ��ճ������ݣ����������г��������Execute
//��boot�Ĺ̻�����������ʱ�Ĵ��ڷֳ��ĸ�״̬����ʵҲ����д��һ��
void UART1_Process(void) {
	
	switch (uartState) {								
		case commandRecv:											//�������״̬		
			if(strcmp(Uart1_Rx_Buf, "Download") == 0) { 			//Uart1_Rx_Buf�Ǵ��ڽ��ջ�����
				uartState = sizeRecv; 								//�л�״̬
				DMA_USART1_Send("Download", 8); 				//����λ������ָ��
			}
			else if(strcmp(Uart1_Rx_Buf, "Execute") == 0) { 		
				uartState = executeApp; 							//�л�״̬��executeApp��ʵûʲô��
				DMA_USART1_Send("Execute", 9); 						//����λ������ָ�ΪʲôExecute��7���ֽڶ�����9����
				while(DMA2_Stream7_Send_Over == 0);					//������෢�������ֽڣ����ڳ������ת��������ֽڻᷢ�ʹ���
				jump_to_app(APP_BASE_ADDR); 						//�˳�bootloader������app����
			}
			else {
				DMA_USART1_Send("Error", 5);						//����λ�����ش���״̬
			}
			
			memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);				//���ľ�����մ��ڽ��ջ������õ�
			DMA_Cmd(DMA2_Stream5, DISABLE);
			DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
			
			break;
		case sizeRecv:												//�����С����״̬
			appRecvInfo.totalDataSize = 0;
			for(int i = 0; Uart1_Rx_Buf[i] != '\0'; i++)			//���ַ���ת����int���͵����ݣ��൱��atoi����
				appRecvInfo.totalDataSize = appRecvInfo.totalDataSize*10 + (Uart1_Rx_Buf[i] - '0');

			if(appRecvInfo.totalDataSize > 0 && appRecvInfo.totalDataSize < 512*1024) {  	//�Գ����С��һ������
				appRecvInfo.writeAddress = APP_BASE_ADDR;									//��ʼ��flash��д���׵�ַ
				appRecvInfo.receivedDataSize = 0;											//�����յ��ĳ������ݴ�С����Ϊ0
				appRecvInfo.receivedtotalSize = 0;											//�����յ��ĳ��������ܴ�С����Ϊ0
				flashErase(APP_BASE_ADDR, APP_BASE_ADDR + appRecvInfo.totalDataSize);		//������Ҫд���flash����
				uartState = appDownload;													//�л�״̬
				
				DMA_USART1_Send(Uart1_Rx_Buf, strlen(Uart1_Rx_Buf));						//����λ����������
			}
			else {
				DMA_USART1_Send("sizeError", 9);											//���յ�����ĳ����С
				uartState = commandRecv;													//�л�״̬
			}
			
			memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);										//���ľ�����մ��ڽ��ջ������õ�
			DMA_Cmd(DMA2_Stream5, DISABLE);
			DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
			
			break;
		case appDownload:															//���س������ݣ���д�뵽flash
			//������յ�����������С����Ϊ�������������һ����1024�ֽڣ�����ôDMA�ᴥ�����жϣ�DMA2_Stream5->NDTR������ΪUART1_RX_BUF_LEN
			appRecvInfo.receivedDataSize = (DMA2_Stream5->NDTR == UART1_RX_BUF_LEN) ? UART1_RX_BUF_LEN : (UART1_RX_BUF_LEN - DMA2_Stream5->NDTR);
			appRecvInfo.receivedtotalSize += appRecvInfo.receivedDataSize;			//���յ�����������
			
			//�鿴�Ƿ���յ���һ����1024�ֽڣ�����ɿ�UART1_RX_BUF_LEN���������ݣ����������һ�����ݣ�����ǣ���д��
			if(appRecvInfo.receivedDataSize == UART1_RX_BUF_LEN || appRecvInfo.receivedtotalSize == appRecvInfo.totalDataSize) {
				
				//��һ����д��flash
				FLASH_Unlock();
				FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
								FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
				for(int i = 0; i < appRecvInfo.receivedDataSize; i += 1) {
					FLASH_ProgramByte(appRecvInfo.writeAddress, (uint8_t)Uart1_Rx_Buf[i]);
					appRecvInfo.writeAddress += 1;
				}
				FLASH_Lock();
				
				//�ж��Ƿ�������
				if(appRecvInfo.receivedtotalSize == appRecvInfo.totalDataSize) {
					DMA_USART1_Send("download over", 13);
					uartState = commandRecv;
				}
				else {
					DMA_USART1_Send("OK", 2);		//ÿ���յ�һ�����ݷ���һ��OK
				}
				
				memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);
				DMA_Cmd(DMA2_Stream5, DISABLE);
				DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
				DMA_Cmd(DMA2_Stream5, ENABLE);
			}
			break;
		case executeApp:
//			jump_to_app(appRecvInfo.baseAddress);
			break;
		default: break;
	}
}


void USART1_Init(void) {
	UART1_Config(115200);
	DMA_UART1_TX_Config();
	DMA_UART1_RX_Config();
}
