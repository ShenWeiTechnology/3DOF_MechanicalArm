#include "stm32f10x.h"
#include "Delay.h"

extern uint8_t sys_state;
extern uint8_t clean2;
extern uint8_t v;

uint8_t Key_Value = 0;

void ExitKey_Init(void)
{
    // 1. 开时钟：GPIOB + AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    // 禁用JTAG，释放PB4
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    // 2. 配置GPIO为上拉输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 关闭所有中断线（防止初始化误触发）
    EXTI->IMR &= ~(EXTI_Line4 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7);

    // 4. 绑定GPIO引脚到EXTI中断线
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);

    // 5. 配置EXTI中断参数
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    // ====================== 核心修改：双边沿触发（按下+松开都触发） ======================
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

    EXTI_InitStructure.EXTI_Line = EXTI_Line4;  EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;  EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;  EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;  EXTI_Init(&EXTI_InitStructure);

    // 6. 配置NVIC中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    // PB4对应 EXTI4_IRQn
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;    NVIC_Init(&NVIC_InitStructure);
    // PB5/6/7对应 EXTI9_5_IRQn
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   NVIC_Init(&NVIC_InitStructure);

    // 7. 清除所有中断挂起位
    EXTI->PR = EXTI_Line4 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7;
    Key_Value = 0;
}

// ====================== PB4的中断服务函数 ======================
void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) == SET)
    {
        // 双边沿触发，直接赋值，适配电平翻转
        Key_Value = 1;
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

// ====================== PB5/6/7的中断服务函数 ======================
void EXTI9_5_IRQHandler(void)
{
    // PB5 中断
    if (EXTI_GetITStatus(EXTI_Line5) == SET)
    {
        Key_Value = 2;
        EXTI_ClearITPendingBit(EXTI_Line5);
    }

    // PB6 中断
    if (EXTI_GetITStatus(EXTI_Line6) == SET)
    {
        Key_Value = 3;
        EXTI_ClearITPendingBit(EXTI_Line6);
    }

    // PB7 中断
    if (EXTI_GetITStatus(EXTI_Line7) == SET)
    {
        // 对应原PB10的功能（修改系统状态）
        sys_state = 1;
        clean2 = 0;
        v = 0;
        Key_Value = 4;
        EXTI_ClearITPendingBit(EXTI_Line7);
    }
}
