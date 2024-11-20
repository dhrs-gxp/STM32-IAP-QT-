#include "boot.h"	

typedef void(*appFun)(void);

void jump_to_app(uint32_t startAddr)
{
	__disable_irq();				
	
	uint32_t appAddr = *(__IO uint32_t*)(startAddr + 4);	//�����ʼ��ַ����4���ֽڣ����ж����������ʼλ�ã��洢����Reset Handler
	appFun appStart = (appFun)appAddr;						//ʹ�ú���ָ�����ʽ���������reset�������Ӷ�������app����������
	__set_MSP( *(__IO  uint32_t*) (startAddr));				//��������ö�ջָ�룬����ջָ��ָ���û�����Ķ�ջ
	
	appStart();												//����һ�������Ǵ�����Reset Handler��ʼ�ģ���ʼ�����û�����
}	