#include "usart.h"	
#include "string.h"
#include "stdlib.h"
#include "flash.h"
#include "boot.h"

#define UART1_RX_BUF_LEN 1024
#define UART1_TX_BUF_LEN 512
char Uart1_Rx_Buf[UART1_RX_BUF_LEN];
char Uart1_Tx_Buf[UART1_TX_BUF_LEN];

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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;		//抢占优先级1
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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;          
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);	//采用满中断(1)
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//清除DMA2_Steam7传输完成标志(2)
		
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	//采用DMA方式接收
	DMA_Cmd(DMA2_Stream7, DISABLE);					//开启DMA传输 
}

volatile int DMA2_Stream7_Send_Over = 1;

void DMA2_Stream7_IRQHandler(void)
{
	//清除标志
	if(DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) != RESET)//等待DMA2_Steam7传输完成
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
		DMA2_Stream7_Send_Over = 1;
	}
}

void DMA_USART1_Send(char *data, uint16_t size)
{
	while(DMA2_Stream7_Send_Over == 0);
	DMA2_Stream7_Send_Over = 0;
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


volatile uint8_t UART1_IDLE_IRQ = 0;
volatile uint8_t BOOT_MODE = 0;
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	uint8_t data = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //接收空闲中断
	{
		data = USART1->SR;
        data = USART1->DR;
		
		if(BOOT_MODE == 1) UART1_IDLE_IRQ = 1;			//如果在boot的固化程序功能中，每次空闲中断将UART1_IDLE_IRQ标志位置1，在主函数中处理
		else {											//如果还没有进入boot的固化程序功能，每次接收到信息，将信息从缓冲区删除
			memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);
			DMA_Cmd(DMA2_Stream5, DISABLE);
			DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
		}
	} 
} 

enum State {commandRecv, sizeRecv, appDownload, executeApp};	//BOOT的四种状态，分别是接收指令，接收文件大小，接收文件数据，运行app程序
enum State uartState = commandRecv;

struct AppRecvInfo {				//接收app程序时的一些参数
	uint32_t writeAddress;			//程序在flash中的基地址
	uint32_t totalDataSize;			//程序数据的总大小
	uint32_t receivedtotalSize;		//已经写入到flash的数据大小
	uint32_t receivedDataSize;		//本次写入时接收到的数据大小
};

struct AppRecvInfo appRecvInfo = {APP_BASE_ADDR, 0, 0, 0};

//固化的顺序：首先接收Download指令，再接收程序大小，再接收程序数据，最后接收运行程序的命令Execute
//将boot的固化程序功能运行时的串口分成四个状态，其实也可以写到一起
void UART1_Process(void) {
	
	switch (uartState) {								
		case commandRecv:											//命令接收状态		
			if(strcmp(Uart1_Rx_Buf, "Download") == 0) { 			//Uart1_Rx_Buf是串口接收缓冲区
				uartState = sizeRecv; 								//切换状态
				DMA_USART1_Send("Download", 8); 				//向上位机返回指令
			}
			else if(strcmp(Uart1_Rx_Buf, "Execute") == 0) { 		
				uartState = executeApp; 							//切换状态，executeApp其实没什么用
				DMA_USART1_Send("Execute", 9); 						//向上位机返回指令，为什么Execute是7个字节而发送9个？
				while(DMA2_Stream7_Send_Over == 0);					//如果不多发送两个字节，由于程序的跳转最后两个字节会发送错误
				jump_to_app(APP_BASE_ADDR); 						//退出bootloader，运行app程序
			}
			else {
				DMA_USART1_Send("Error", 5);						//向上位机返回错误状态
			}
			
			memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);				//这四句是清空串口接收缓冲区用的
			DMA_Cmd(DMA2_Stream5, DISABLE);
			DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
			
			break;
		case sizeRecv:												//程序大小接收状态
			appRecvInfo.totalDataSize = 0;
			for(int i = 0; Uart1_Rx_Buf[i] != '\0'; i++)			//将字符串转换成int类型的数据，相当于atoi函数
				appRecvInfo.totalDataSize = appRecvInfo.totalDataSize*10 + (Uart1_Rx_Buf[i] - '0');

			if(appRecvInfo.totalDataSize > 0 && appRecvInfo.totalDataSize < 512*1024) {  	//对程序大小做一个限制
				appRecvInfo.writeAddress = APP_BASE_ADDR;									//初始化flash的写入首地址
				appRecvInfo.receivedDataSize = 0;											//将接收到的程序数据大小设置为0
				appRecvInfo.receivedtotalSize = 0;											//将接收到的程序数据总大小设置为0
				flashErase(APP_BASE_ADDR, APP_BASE_ADDR + appRecvInfo.totalDataSize);		//擦除将要写入的flash扇区
				uartState = appDownload;													//切换状态
				
				DMA_USART1_Send(Uart1_Rx_Buf, strlen(Uart1_Rx_Buf));						//向上位机反馈数据
			}
			else {
				DMA_USART1_Send("sizeError", 9);											//接收到错误的程序大小
				uartState = commandRecv;													//切换状态
			}
			
			memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);										//这四句是清空串口接收缓冲区用的
			DMA_Cmd(DMA2_Stream5, DISABLE);
			DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
			
			break;
		case appDownload:															//下载程序数据，并写入到flash
			//计算接收到的数据量大小，因为如果接受了满满一包（1024字节），那么DMA会触发满中断，DMA2_Stream5->NDTR会重置为UART1_RX_BUF_LEN
			appRecvInfo.receivedDataSize = (DMA2_Stream5->NDTR == UART1_RX_BUF_LEN) ? UART1_RX_BUF_LEN : (UART1_RX_BUF_LEN - DMA2_Stream5->NDTR);
			appRecvInfo.receivedtotalSize += appRecvInfo.receivedDataSize;			//接收到的总数据量
			
			//查看是否接收到了一包（1024字节，具体可看UART1_RX_BUF_LEN变量）数据，或者是最后一包数据，如果是，则写入
			if(appRecvInfo.receivedDataSize == UART1_RX_BUF_LEN || appRecvInfo.receivedtotalSize == appRecvInfo.totalDataSize) {
				
				//这一段是写入flash
				FLASH_Unlock();
				FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
								FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
				for(int i = 0; i < appRecvInfo.receivedDataSize; i += 1) {
					FLASH_ProgramByte(appRecvInfo.writeAddress, (uint8_t)Uart1_Rx_Buf[i]);
					appRecvInfo.writeAddress += 1;
				}
				FLASH_Lock();
				
				//判断是否接收完成
				if(appRecvInfo.receivedtotalSize == appRecvInfo.totalDataSize) {
					DMA_USART1_Send("download over", 13);
					uartState = commandRecv;
				}
				else {
					DMA_USART1_Send("OK", 2);		//每接收到一包数据发送一个OK
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
