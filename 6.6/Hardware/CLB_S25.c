#include "CLB_S25.h"
#include "Delay.h"

#define PWM_MIN  500
#define PWM_MAX  2500
#define ANGLE_RNG 180

static void USART2_SendData(uint8_t *dat, uint8_t len)
{
	uint8_t i;
	for(i=0;i<len;i++)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
		USART_SendData(USART2,dat[i]);
	}
}

// 初始化 USART2，同时初始化 PA2 + PA3 （支持总线转接板）
void CLB_S25_USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	// 开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	// ========== 初始化 PA2 (TX) 复用推挽 ==========
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	// ========== 初始化 PA3 (RX) 上拉输入 ==========
	// 总线转接板必须初始化RX，否则通信不稳！
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	// 串口配置
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // 收发都开
	USART_Init(USART2,&USART_InitStruct);
	
	USART_Cmd(USART2,ENABLE);
}

void CLB_S25_SetServoAngle(uint8_t servo_id, int16_t angle)
{
	uint8_t buf[9];
	uint16_t pwm_val;
	if(angle<0) angle=0;
	if(angle>180) angle=180;
	
	pwm_val = PWM_MIN + (angle*(PWM_MAX-PWM_MIN))/ANGLE_RNG;
	
	buf[0] = 0x55;
	buf[1] = 0x55;
	buf[2] = servo_id;
	buf[3] = 0x05;
	buf[4] = 0x01;
	buf[5] = pwm_val&0xff;
	buf[6] = (pwm_val>>8)&0xff;
	buf[7] = 0x00;
	buf[8] = 0x00;
	USART2_SendData(buf,9);
}