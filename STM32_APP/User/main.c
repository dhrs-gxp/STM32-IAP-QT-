#include "stm32f4xx.h"
#include "string.h"
#include "usart.h"
#include "delay.h"
#include "led.h"

extern char Uart1_Rx_Buf[UART1_RX_BUF_LEN];

extern volatile uint8_t msgRecvFromUART1;

int main(void)
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
	__enable_irq();
	
	delay_init(168);
	USART1_Init();
	LED_Init();
	
	DMA_USART1_Send("Hello from app!\r\n", 17);
	
	while(1) {
		
		if(msgRecvFromUART1 == 1) {
			DMA_USART1_Send(Uart1_Rx_Buf, strlen(Uart1_Rx_Buf));
			memset(Uart1_Rx_Buf, 0, UART1_RX_BUF_LEN);
			DMA_Cmd(DMA2_Stream5, DISABLE);
			DMA2_Stream5->NDTR = UART1_RX_BUF_LEN;
			DMA_Cmd(DMA2_Stream5, ENABLE);
			msgRecvFromUART1 = 0;
		}
		
		GPIO_ToggleBits(GPIOF, GPIO_Pin_9);

		delay_ms(100);
	}
	
	return 0;
}

