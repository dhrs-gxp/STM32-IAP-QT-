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

//避免使用半主机模式    
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
/* FILE 在 stdio.h里面定义. */
FILE __stdout;

#if defined ( __GNUC__ ) && !defined (__clang__) 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 

//重定义fputc函数 
PUTCHAR_PROTOTYPE
{
	while((USART1->SR&0X40)==0);//等待上一帧数据发送完毕  
	USART1->DR = (uint8_t)ch;
	return ch;
}


/**********************************串口1*************************************/
//bound:波特率
void UART1_Config(uint32_t bound){
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART1);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1 端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;	//GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;										//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式
	USART_Init(USART1, &USART_InitStructure);										//初始化串口1

	//USART1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10;		//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;			//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);								//根据指定的参数初始化NVIC寄存器
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);	//开启相关中断

	USART_Cmd(USART1, ENABLE);						//使能串口1
}

void DMA_UART1_TX_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	//开启DMA时钟 
    DMA_DeInit(DMA2_Stream7);
    while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}		//等待stream可配置，即DMAy_SxCR.EN变为0
    //配置Stream
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//从8个channel中选择一个
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;       //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart1_Tx_Buf;			//存储器0地址，双缓存模式还要使用M1AR
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;            		//存储器到外设模式 RX TX不同这里，外设到地址或者地址到外设
    DMA_InitStructure.DMA_BufferSize = UART1_TX_BUF_LEN;                	//数据传输量，以外设数据项为单位 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址保持不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //存储器地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据位宽:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据位宽:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                         	//普通模式(与循环模式对应)
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //禁止FIFO模式         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel                   = DMA2_Stream7_IRQn;           
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	//采用满中断(1)
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//清除DMA2_Steam7传输完成标志(2)
		
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	//采用DMA方式接收
	DMA_Cmd(DMA2_Stream7, DISABLE);					//开启DMA传输 
}

extern SemaphoreHandle_t uart1SendSemaphore;

void DMA2_Stream7_IRQHandler(void)
{
	//清除标志
	if(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET)//等待DMA2_Steam7传输完成
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
		xSemaphoreGiveFromISR(uart1SendSemaphore, NULL);
	}
}

void DMA_USART1_Send(char *data, uint16_t size)
{
	xSemaphoreTake(uart1SendSemaphore, portMAX_DELAY);
	memcpy(Uart1_Tx_Buf, data, size);					//复制数据到DMA发送缓存区
	
	DMA_Cmd(DMA2_Stream7, DISABLE);						//打开DMA数据流，开始发送
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);	//确保DMA可以被设置
	DMA_SetCurrDataCounter(DMA2_Stream7, size);			//设置数据传输长度
	DMA_Cmd(DMA2_Stream7, ENABLE);						//打开DMA数据流，开始发送
}

void DMA_UART1_RX_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);	//开启DMA时钟 
    DMA_DeInit(DMA2_Stream5);
    while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){}		//等待stream可配置，即DMAy_SxCR.EN变为0
    //配置Stream
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;							//从8个channel中选择一个
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;       //外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Uart1_Rx_Buf;			//存储器0地址，双缓存模式还要使用M1AR
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;            		//存储器到外设模式 RX TX不同这里，外设到地址或者地址到外设
    DMA_InitStructure.DMA_BufferSize = UART1_RX_BUF_LEN;                	//数据传输量，以外设数据项为单位
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址保持不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 //存储器地址递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据位宽:8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //存储器数据位宽:8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         //普通模式(与循环模式对应)
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                   //中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //禁止FIFO模式
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             //单次传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     //单次传输
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//采用DMA方式接收
	DMA_Cmd(DMA2_Stream5, ENABLE);					//开启DMA传输(3)
}


extern MessageBufferHandle_t usart1MessageBuffer;
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //接收中断
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
	USART_ClearFlag(USARTx,USART_FLAG_TC);									//发送字符前清空标志位（否则缺失字符串的第一个字符）
	while(*DataString != '\0')												//字符串结束符
	{
		USART_SendData(USARTx,*DataString);									//每次发送字符串的一个字符
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC) == 0);				//等待数据发送成功
		USART_ClearFlag(USARTx,USART_FLAG_TC);								//发送字符后清空标志位
		DataString++;
	}
}