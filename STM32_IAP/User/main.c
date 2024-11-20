#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "key.h"
#include "boot.h"

extern volatile uint8_t UART1_IDLE_IRQ;	//串口1接收到数据触发的空闲中断

extern volatile uint8_t BOOT_MODE;		//启动模式，直接启动还是进入固化模式

#define TICKVAL_500ms (168/8*1000*500)		//168MHz的时钟，SysTick_CLKSource_HCLK_Div8设置为8分频，因此500ms需要168/8*1000*500个周期

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	KEY_Init();
	USART1_Init();
	
	//在这里用systick设置6次500ms的计数
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	
	DMA_USART1_Send("press key0 to enter bootloader", 32);
	
	uint32_t temp = 0;
	for(int i = 0; i < 6; i++) {
		while(!isDMASendOver());
		if(i == 0) DMA_USART1_Send("3", 3);
		else if(i == 2) DMA_USART1_Send("2", 3);
		else if(i == 4) DMA_USART1_Send("1", 3);
		SysTick->LOAD=(uint32_t)TICKVAL_500ms;
		SysTick->VAL=0x00;
		SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
		
		do{
			temp=SysTick->CTRL;
			if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 0){
				BOOT_MODE = 1;		//如果在开机三秒内按下GPIOB，GPIO_Pin_8按键，那么进入固化程序的功能
				break;				//如果三秒到了或者进入了固化的功能，则退出循环
			}
		} while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达
		
		if(BOOT_MODE == 1) break;
	}
	

	if(BOOT_MODE == 0) jump_to_app(APP_BASE_ADDR);		//如果没有进入固化的功能，则运行app程序
	
	while(!isDMASendOver());
	DMA_USART1_Send("bootloader", 10);
	
	while(1) {											//执行固化程序的功能
		if(UART1_IDLE_IRQ == 1) {						//串口每接收一次数据，发生空闲中断就运行一次UART1_Process函数处理一次
			UART1_Process();
			UART1_IDLE_IRQ = 0;
		}
	}
	
	return 0;
}

