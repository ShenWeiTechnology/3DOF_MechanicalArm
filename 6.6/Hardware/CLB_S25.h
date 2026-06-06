#ifndef __CLB_S25_H
#define __CLB_S25_H

#include "stm32f10x.h"

void CLB_S25_USART2_Init(void);
void CLB_S25_SetServoAngle(uint8_t servo_id, int16_t angle);

#endif
