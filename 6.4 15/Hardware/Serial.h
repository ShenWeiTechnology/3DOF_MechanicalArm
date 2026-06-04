#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

// 接收 OpenMV 的 3 个数字 (0~500)
extern uint16_t OpenMV_Data1;
extern uint16_t OpenMV_Data2;
extern uint16_t OpenMV_Data3;

// 串口初始化函数
void Serial_Init(void);

#endif
