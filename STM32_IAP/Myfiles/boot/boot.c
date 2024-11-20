#include "boot.h"	

typedef void(*appFun)(void);

void jump_to_app(uint32_t startAddr)
{
	__disable_irq();				
	
	uint32_t appAddr = *(__IO uint32_t*)(startAddr + 4);	//程序初始地址加上4个字节，是中断向量表的起始位置，存储的是Reset Handler
	appFun appStart = (appFun)appAddr;						//使用函数指针的形式，运行这个reset函数，从而引导到app程序中运行
	__set_MSP( *(__IO  uint32_t*) (startAddr));				//这个是设置堆栈指针，将堆栈指针指向用户程序的堆栈
	
	appStart();												//运行一个程序是从运行Reset Handler开始的，开始运行用户程序
}	