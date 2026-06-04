#ifndef __PID_H    
#define __PID_H
#include <stdint.h> // 包含固定尺寸整数类型（uint8_t、int16_t等）

// PID结构体（PID参数和状态）
typedef struct 
	{
		float Kp;        // 比例系数：决定响应速度
		float Ki;        // 积分系数：消除静态误差
		float Kd;        // 微分系数：抑制抖动、超调
		float integral;  // 积分累加值（累计偏差）
		float prev_err;  // 上一次的偏差值（计算微分用）
	} PID_Handle;

	
	
typedef struct {
    // PID基本参数（和位置式完全相同）
    float Kp;
    float Ki;
    float Kd;

    // ----------------新增：增量式PID必需的历史变量----------------
    float e_prev1;   // 上一次偏差 e(k-1)
    float e_prev2;   // 上上次偏差 e(k-2)
    float u_prev;    // 上一次控制量 u(k-1)（相对于舵机中位的偏移）
} PID_Handle2;
	
	
void PID_Init(PID_Handle *pid, float Kp, float Ki, float Kd);// 初始化PID参数
void PID_Init2(PID_Handle2 *pid, float Kp, float Ki, float Kd);// 初始化PID参数
float PID_Calc(PID_Handle *pid, int16_t err);// 计算PID输出
float PID_Inc_Calc(PID_Handle2 *pid, float err);//增量式
uint16_t Servo_PWM_Update(PID_Handle *pid, int16_t camera_err);// 根据视觉偏差计算舵机PWM值
uint16_t Servo_PWM_Update2(PID_Handle2 *pid, int16_t camera_err);


#endif
