#ifndef __INVERSE_H
#define __INVERSE_H

#include "stm32f10x.h"
#include "math.h"

// 机械臂硬件参数
#define L1            83.0f    // 大臂长度(mm)
#define L2            109.0f+83.0f// 小臂长度(mm)
#define BASE_HEIGHT   106.0f     // 底座高度(mm)
#define ARM_OFFSET    12.0f     // 末端补偿(mm)
#define PI            3.1415926535f

/**
 * @brief  三轴机械臂逆运动学解算
 * @param  x: 目标点X坐标（左右方向，右为正）
 * @param  y: 目标点Y坐标（前后方向，前为正）
 * @param  z: 目标点Z坐标（上下方向，上为正）
 * @param  theta0: 输出底座舵机角度(ID2，单位：度)
 * @param  theta1: 输出大臂舵机角度(ID0，单位：度) 
 * @param  theta2: 输出小臂舵机角度(ID1，单位：度) 
 * @retval 0:解算成功 1:目标点超出工作空间
 */
uint8_t inverse_kinematics(float x, float y, float z, float *theta0, float *theta1, float *theta2);

/**
 * @brief  封装好的测试函数：机械臂移动到指定坐标
 * @param  x,y,z 目标坐标
 * @retval 无
 */
uint8_t arm_go_to_point(float x, float y, float z);

/**
 * @brief  直线插补平滑移动（自动计算步数，固定步长）
 * @param  x1,y1,z1: 起点坐标
 * @param  x2,y2,z2: 终点坐标
 * @param  speed: 舵机移动速度（0~1000）
 * @retval 0:成功 1:起点不可达 2:终点不可达
 */
uint8_t move_smooth_auto(float x1, float y1, float z1, float x2, float y2, float z2, uint16_t speed);

/**
 * @brief  直线插补平滑移动（手动指定步数，调试用）
 * @param  x1,y1,z1: 起点坐标
 * @param  x2,y2,z2: 终点坐标
 * @param  steps: 手动指定插补步数
 * @param  speed: 舵机移动速度（0~1000）
 * @retval 0:成功 1:起点不可达 2:终点不可达
 */
uint8_t move_smooth_manual(float x1, float y1, float z1, float x2, float y2, float z2, uint16_t steps, uint16_t speed);

void GetCameraPosition(float a1, float a2, float a3,
                       float *x, float *y, float *z,
                       float *roll, float *pitch, float *yaw);
#endif
