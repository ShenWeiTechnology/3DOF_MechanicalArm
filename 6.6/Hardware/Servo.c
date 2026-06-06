//#include "stm32f10x.h"                  // Device header
//#include "PWM.h"

///**
//  * 函    数：舵机初始化
//  * 参    数：无
//  * 返 回 值：无
//  */
//void Servo_Init(void)
//{
//	PWM_Init();									//初始化舵机的底层PWM
//}






#include "stm32f10x.h"
#include "PWM.h"
///**
//  * 函    数：舵机设置角度
//  * 参    数：Angle 要设置的舵机角度，范围：0~180
//  * 返 回 值：无
//  */
//void Servo_SetAngle1(float Angle)
//{
//	PWM_SetCompare2(Angle / 180 * 2000 + 500);	//设置占空比
//												//将角度线性变换，对应到舵机要求的占空比范围上
//}

//#include "stm32f10x.h"
//#include "PWM.h"
///**
//  * 函    数：舵机设置角度
//  * 参    数：Angle 要设置的舵机角度，范围：0~180
//  * 返 回 值：无
//  */
//void Servo_SetAngle2(float Angle)
//{
//	PWM_SetCompare3(Angle / 180 * 2000 + 500);	//设置占空比
//												//将角度线性变换，对应到舵机要求的占空比范围上
//}

//#include "stm32f10x.h"
//#include "PWM.h"
///**
//  * 函    数：舵机设置角度
//  * 参    数：Angle 要设置的舵机角度，范围：0~180
//  * 返 回 值：无
//  */
//void Servo_SetAngle3(float Angle)
//{
//	PWM_SetCompare4(Angle / 180 * 2000 + 500);	//设置占空比
//												//将角度线性变换，对应到舵机要求的占空比范围上
//}

/**
  * 函    数：舵机初始化
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Init(void)
{
    PWM_Init();   // 初始化底层3路PWM（你之前写的TIM2_CH2/CH3/CH4）
}

/**
  * 函    数：舵机1（CH2/PA1）设置角度
  * 参    数：Angle 要设置的舵机角度，范围：0~180
  * 返 回 值：无
  */
void Servo_SetAngle1(float Angle)
{
    // 0° → 500，180° → 2500，线性映射
    PWM_SetCompare2(Angle / 180 * 2000 + 500);
}

///**
//  * 函    数：舵机2（CH3/PA2）设置角度
//  * 参    数：Angle 要设置的舵机角度，范围：0~180
//  * 返 回 值：无
//  */
//void Servo_SetAngle2(float Angle)
//{
//	//-50度---500，130度----2500
//    PWM_SetCompare3(Angle / 180 * 2000 + 500);
//}

///**
//  * 函    数：舵机3（CH4/PA3）设置角度
//  * 参    数：Angle 要设置的舵机角度，范围：0~180
//  * 返 回 值：无
//  */
//void Servo_SetAngle3(float Angle)  
//{
//	//-45度---500，135度---2500
//    PWM_SetCompare4(Angle / 180 * 2000 + 500);
//}



/**
  * 函    数：舵机2（中臂 -50° ~ 130° → ID0）
  * 范    围：-50° → 500，130° → 2500
  */
void Servo_SetAngle2(float Angle)
{
    // 把 -50~130 映射到 500~2500
    float Compare = (Angle + 50) / 180 * 2000 + 500;
    PWM_SetCompare3((uint16_t)Compare);
}

/**
  * 函    数：舵机3（小臂 -45° ~ 135° → ID1）
  * 范    围：-45° → 500，135° → 2500
  */
void Servo_SetAngle3(float Angle)
{
    // 把 -45~135 映射到 500~2500
    float Compare = (Angle + 45) / 180 * 2000 + 500;
    PWM_SetCompare3((uint16_t)Compare);
}
