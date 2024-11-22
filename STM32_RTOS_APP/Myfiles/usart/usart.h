#ifndef __USART_H
#define __USART_H
#include "stm32f4xx.h"
#include "stdint.h"

void USART1_Init(void);

void DMA_USART1_Send(char *data, uint16_t size);

void USART_SendString(USART_TypeDef* USARTx, char *DataString);

#endif
