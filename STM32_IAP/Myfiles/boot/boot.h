#ifndef __BOOT_H
#define __BOOT_H
#include "stm32f4xx.h"

#define APP_BASE_ADDR 0x08004000

void jump_to_app(uint32_t addr);

#endif
