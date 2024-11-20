#ifndef __USART_H
#define __USART_H
#include "stm32f4xx.h"
#include "stdint.h"

void UART1_Config(uint32_t bound);
void DMA_UART1_TX_Config(void);
void DMA_UART1_RX_Config(void);

void DMA_USART1_Send(char *data, uint16_t size);
int isDMASendOver(void);

void USART1_Init(void);
void UART1_Process(void);

#endif
