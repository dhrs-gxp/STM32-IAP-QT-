#ifndef __USART_H
#define __USART_H
#include "stm32f4xx.h"
#include "stdint.h"

#define UART1_RX_BUF_LEN 512
#define UART1_TX_BUF_LEN 512

void DMA_USART1_Send(char *data, uint16_t size);
int isDMASendOver(void);

void USART1_Init(void);

#endif
