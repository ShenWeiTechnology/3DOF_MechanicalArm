#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_RxData;		//定义串口接收的数据变量
uint8_t Serial_RxFlag;		//定义串口接收的标志位变量

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial2_Init(void)
{
	/*开启时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	// 注意：USART3 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	// GPIOB 时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				// PB10 TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				// PB11 RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					
	USART_InitStructure.USART_BaudRate = 9600;              
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;     
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);               // 配置 USART3
	
	/*中断输出配置*/
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);			
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;       // 中断号改为 USART3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;         
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      
	NVIC_Init(&NVIC_InitStructure);                         
	
	/*USART使能*/
	USART_Cmd(USART3, ENABLE);								
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	//等待发送完成
}

/**
  * 函    数：串口发送一个数组
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

/**
  * 函    数：串口发送一个字符串
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

/**
  * 函    数：次方函数
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * 函    数：printf重定向
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

/**
  * 函    数：封装printf
  */
void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

/**
  * 函    数：获取接收标志位
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

/**
  * 函    数：获取接收数据
  */
uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;
}

/**
  * 函    数：USART3中断函数
  */
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		Serial_RxData = USART_ReceiveData(USART3);
		Serial_RxFlag = 1;
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}
