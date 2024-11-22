#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "message_buffer.h"

#define START_TASK_STACK_SIZE	256
#define LED_TASK_STACK_SIZE		256
#define UART1_TASK_STACK_SIZE	1024*3

#define START_TASK_Priority		1
#define LED_TASK_Priority		4
#define UART1_TASK_Priority		3

TaskHandle_t startTask_Handler = NULL;
TaskHandle_t ledTask_Handler = NULL;
TaskHandle_t uart1Task_Handler = NULL;

void start_task( void * arg );
void led_task( void * arg );
void uart1_task( void * arg );

int main(void)
{	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);
	__enable_irq();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	LED_Init();
	USART1_Init();
	
	printf("Hello from FreeRTOS!\r\n");
	
	BaseType_t taskCreateReturn;

    taskCreateReturn = xTaskCreate(	(TaskFunction_t )start_task				,
									(const char*    )"start_task"			,
									(uint16_t       )START_TASK_STACK_SIZE	,
									(void*          )NULL					,
									(UBaseType_t    )START_TASK_Priority	,
									(TaskHandle_t*  )&startTask_Handler		);

    if(taskCreateReturn == pdPASS) 
	{
		printf("task %s create successfully!\r\n", pcTaskGetName(startTask_Handler));
		vTaskStartScheduler();
	}

	return 0;
}

void start_task( void * arg )
{
	taskENTER_CRITICAL();
	
	BaseType_t taskCreateReturn;
	taskCreateReturn = xTaskCreate(	(TaskFunction_t )led_task				,
									(const char*    )"led_task"				,
									(uint16_t       )LED_TASK_STACK_SIZE	,
									(void*          )NULL					,
									(UBaseType_t    )LED_TASK_Priority		,
									(TaskHandle_t*  )&ledTask_Handler		);
	if(taskCreateReturn == pdPASS) printf("task %s create successfully!\r\n", pcTaskGetName(ledTask_Handler));
	else printf("task create error!\r\n");
									
	taskCreateReturn = xTaskCreate(	(TaskFunction_t )uart1_task				,
									(const char*    )"uart1_task"			,
									(uint16_t       )UART1_TASK_STACK_SIZE	,
									(void*          )NULL					,
									(UBaseType_t    )UART1_TASK_Priority	,
									(TaskHandle_t*  )&uart1Task_Handler		);
	if(taskCreateReturn == pdPASS) printf("task %s create successfully!\r\n", pcTaskGetName(uart1Task_Handler));
	else printf("task create error!\r\n");
	
	vTaskDelete(startTask_Handler);
									
	taskEXIT_CRITICAL();
}

void led_task( void * arg )
{
	for( ;; )
    {
		GPIO_ToggleBits(GPIOF, GPIO_Pin_9);
		vTaskDelay(500);
    }
}

SemaphoreHandle_t uart1SendSemaphore;

MessageBufferHandle_t usart1MessageBuffer;
#define usart1MessageBufferSize		320
#define uasrt1MaxMessageLength		160

void uart1_task( void * arg )
{
	uart1SendSemaphore = xSemaphoreCreateBinary();
	if(uart1SendSemaphore != NULL) printf("uart1SendSemaphore create successfully!\r\n");
	else printf("uart1SendSemaphore create error!\r\n");
	xSemaphoreGive(uart1SendSemaphore);
	
	usart1MessageBuffer = xMessageBufferCreate(usart1MessageBufferSize);  
	if(usart1MessageBuffer != NULL) printf("usart1MessageBuffer create successfully!\r\n");
	else printf("usart1MessageBuffer create error!\r\n");
	
	size_t messageLength = 0;
	char buffer[uasrt1MaxMessageLength] = {0};
	
    for( ;; )
    {
		messageLength = xMessageBufferReceive(usart1MessageBuffer, buffer, uasrt1MaxMessageLength, portMAX_DELAY);
		if(messageLength > 0) DMA_USART1_Send(buffer, messageLength);

    }
}
