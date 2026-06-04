#include "Serial.h"

// 存储3个接收数据
uint16_t OpenMV_Data1 = 0;
uint16_t OpenMV_Data2 = 0;
uint16_t OpenMV_Data3 = 0;

uint8_t buf[8];  // 接收缓冲区
uint8_t cnt = 0; // 接收计数

// 初始化 USART1 PA9/PA10
void Serial_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // PA9 TX 复用推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 RX 上拉输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 串口配置 9600波特率
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    // 开启接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    // 中断优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

// 串口中断服务函数（解析3个数字）
void USART1_IRQHandler(void)
{
    uint8_t ch;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        ch = USART_ReceiveData(USART1);

        if(cnt == 0 && ch == 0xAA)  // 帧头
        {
            buf[cnt++] = ch;
        }
        else if(cnt > 0 && cnt < 7) // 接收数据
        {
            buf[cnt++] = ch;
        }
        else if(cnt == 7 && ch == 0xFF) // 帧尾，解析完成
        {
            // 解析3个数字
            OpenMV_Data1 = (buf[1] << 8) | buf[2];
            OpenMV_Data2 = (buf[3] << 8) | buf[4];
            OpenMV_Data3 = (buf[5] << 8) | buf[6];

            // 限制范围 0~500
            if(OpenMV_Data1>500) OpenMV_Data1=500;
            if(OpenMV_Data2>500) OpenMV_Data2=500;
            if(OpenMV_Data3>500) OpenMV_Data3=500;

            cnt = 0; // 重置计数
        }
        else
        {
            cnt = 0; // 出错重置
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
