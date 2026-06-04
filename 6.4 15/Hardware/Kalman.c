#include "Kalman.h"

// 初始化一维卡尔曼滤波器
void Kalman_Init(Kalman_Filter *kf, float Q, float R, float initial_P, float initial_X)
{
    kf->Q = Q;
    kf->R = R;
    kf->P = initial_P;
    kf->X = initial_X;
    kf->K = 0;
}

// 卡尔曼滤波更新（标准一维卡尔曼滤波）
float Kalman_Update(Kalman_Filter *kf, float measurement)
{
    // 1. 先验估计（预测步骤）
    // X_pred = X_last (因为没有控制输入，假设状态不变)
    float X_pred = kf->X;
    
    // 2. 先验误差协方差
    // P_pred = P_last + Q
    float P_pred = kf->P + kf->Q;
    
    // 3. 卡尔曼增益计算
    // K = P_pred / (P_pred + R)
    kf->K = P_pred / (P_pred + kf->R);
    
    // 4. 后验估计（更新步骤）
    // X_est = X_pred + K * (measurement - X_pred)
    kf->X = X_pred + kf->K * (measurement - X_pred);
    
    // 5. 更新误差协方差
    // P_est = (1 - K) * P_pred
    kf->P = (1.0f - kf->K) * P_pred;
    
    // 保存当前值作为下次的last值
    kf->last_X = kf->X;
    
    return kf->X;
}


void Kalman_Reset(Kalman_Filter *kf, float initial_X)
{
    kf->P = 1.0f;
    kf->K = 0.0f;
    kf->X = initial_X;
    kf->last_X = initial_X;
    // Q和R保持不变，不需要重置
}
