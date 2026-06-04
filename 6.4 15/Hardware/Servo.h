#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

void Servo_Init(void);
void Servo_SetAngle1(float Angle);
void Servo_SetAngle2(float Angle);
void Servo_SetAngle3(float Angle);
//void Servo_SetAngle1(uint16_t Angle);
//void Servo_SetAngle2(uint16_t Angle);
//void Servo_SetAngle3(uint16_t Angle);

#endif
