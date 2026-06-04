#ifndef __KALMAN_H
#define __KALMAN_H

#include "stm32f10x.h"
#include <stdio.h>

// 一维卡尔曼滤波器结构体
typedef struct
{
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float P;        // 估计误差协方差
    float K;        // 卡尔曼增益
    float X;        // 状态估计值
    float last_X;   // 上一次状态估计值
} Kalman_Filter;

// 初始化卡尔曼滤波器
void Kalman_Init(Kalman_Filter *kf, float Q, float R, float initial_P, float initial_X);

// 卡尔曼滤波更新
float Kalman_Update(Kalman_Filter *kf, float measurement);

// 重置卡尔曼滤波器
void Kalman_Reset(Kalman_Filter *kf, float initial_X);

#endif
