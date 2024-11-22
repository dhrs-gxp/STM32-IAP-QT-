#include "boot.h"	

typedef void(*appFun)(void);

void jump_to_app(uint32_t startAddr)
{
	__disable_irq();
	
	uint32_t appAddr = *(__IO uint32_t*)(startAddr + 4);
	appFun appStart = (appFun)appAddr;
	__set_MSP( *(__IO  uint32_t*) (startAddr));
	
	appStart();
}