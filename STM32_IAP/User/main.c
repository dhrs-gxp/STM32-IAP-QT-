#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "key.h"
#include "boot.h"

extern volatile uint8_t UART1_IDLE_IRQ;	//����1���յ����ݴ����Ŀ����ж�

extern volatile uint8_t BOOT_MODE;		//����ģʽ��ֱ���������ǽ���̻�ģʽ

#define TICKVAL_500ms (168/8*1000*500)		//168MHz��ʱ�ӣ�SysTick_CLKSource_HCLK_Div8����Ϊ8��Ƶ�����500ms��Ҫ168/8*1000*500������

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	KEY_Init();
	USART1_Init();
	
	//��������systick����6��500ms�ļ���
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
				BOOT_MODE = 1;		//����ڿ��������ڰ���GPIOB��GPIO_Pin_8��������ô����̻�����Ĺ���
				break;				//������뵽�˻��߽����˹̻��Ĺ��ܣ����˳�ѭ��
			}
		} while((temp&0x01)&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��
		
		if(BOOT_MODE == 1) break;
	}
	

	if(BOOT_MODE == 0) jump_to_app(APP_BASE_ADDR);		//���û�н���̻��Ĺ��ܣ�������app����
	
	while(!isDMASendOver());
	DMA_USART1_Send("bootloader", 10);
	
	while(1) {											//ִ�й̻�����Ĺ���
		if(UART1_IDLE_IRQ == 1) {						//����ÿ����һ�����ݣ����������жϾ�����һ��UART1_Process��������һ��
			UART1_Process();
			UART1_IDLE_IRQ = 0;
		}
	}
	
	return 0;
}

