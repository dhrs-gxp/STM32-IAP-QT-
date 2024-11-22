#ifndef __FLASH_H
#define __FLASH_H
#include "stm32f4xx.h"

void flashErase(uint32_t startAddr, uint32_t endAddr);

#endif
