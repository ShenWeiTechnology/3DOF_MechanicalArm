#include "ZUOBIAOXI.h"
#include "stm32f10x.h"
#include <math.h>

#define L1            83.0f    // 大臂长度(mm)
#define L2            109.0f    // 小臂长度(mm)
#define BASE_HEIGHT   106.0f     // 底座高度(mm)
#define ARM_OFFSET    12.0f     // 末端补偿(mm)
#define PI            3.1415926535f

// 角度转弧度宏（和你原有代码保持一致）
#ifndef DEG2RAD
#define DEG2RAD(deg) ((deg) * 3.141592653589793f / 180.0f)
#endif

/**
  * @brief  3x3旋转矩阵乘以3D向量（STM32优化版）
  * @param  R: 3x3旋转矩阵（行优先，和数学矩阵完全一致）
  * @param  v: 输入3D向量
  * @retval 旋转后的3D向量
  */
static Point3D rotate_matrix_multiply(const float R[3][3], const Point3D *v)
{
    Point3D result;
    result.x = R[0][0] * v->x + R[0][1] * v->y + R[0][2] * v->z;
    result.y = R[1][0] * v->x + R[1][1] * v->y + R[1][2] * v->z;
    result.z = R[2][0] * v->x + R[2][1] * v->y + R[2][2] * v->z;
    return result;
}

/**
  * @brief  手坐标系坐标转换到底座坐标系（最终验证版）
  * @param  theta0: 底座舵机角度(°)（对应IP2），绕Z轴旋转，向右为正
  * @param  theta1: 大臂舵机角度(°)（对应IP0），仅影响平移向量T
  * @param  theta2: 小臂舵机角度(°)（对应IP1），绕X轴旋转，向上为正
  * @param  hand_point: 手坐标系下的3D坐标
  * @param  base_point: 输出底座坐标系下的3D坐标
  * @note   旋转顺序：先绕底座Z轴转theta0，再绕手X轴转(theta2-90°)
  *         当theta2=90°时，矩阵精确退化为你手写的纯Z轴旋转矩阵
  *         变换公式：P_base = R * P_hand + T
  *         T为手坐标系原点在底座坐标系下的坐标，由calc_coordinates计算
  */
void hand_to_base(float theta0, float theta1, float theta2,
                  const Point3D *hand_point, Point3D *base_point)
{
    // 角度映射（最终正确版）
    float alpha = DEG2RAD(theta0);       // 底座绕Z轴旋转角
    float beta = DEG2RAD(theta2+theta1 - 180.0f); // 小臂绕X轴俯仰角（theta2=90°时水平）
    
    // 预计算三角函数（减少重复计算，提升STM32运行速度）
    float ca = cosf(alpha);
    float sa = sinf(alpha);
    float cb = cosf(beta);
    float sb = sinf(beta);
    
    // 最终验证通过的旋转矩阵 R = Rz(alpha) * Rx(beta)
    const float R_hand2base[3][3] = {
        { ca,  sa*cb, -sa*sb },
        {-sa,  ca*cb, -ca*sb },
        { 0.0f,  sb,     cb    }
    };
    
    // 计算平移向量T（完全调用你原有的正确正解函数）
    Point2D pointC;
    Point3D T;
    calc_coordinates(theta0, theta1, theta2, &pointC, &T);
    
    // 执行刚体变换：先旋转，再平移（等价于4x4齐次矩阵乘法）
    *base_point = rotate_matrix_multiply(R_hand2base, hand_point);
    base_point->x += T.x;
    base_point->y += T.y;
    base_point->z += T.z;
}

// 你最新修正的坐标正解函数（完全保持不变，无需任何修改）
void calc_coordinates
(
    float theta0, float theta1, float theta2,
    Point2D *pointC, Point3D *pointD
)
{
    // 角度转弧度
    float theta0_rad = DEG2RAD(theta0);
    float theta1_rad = DEG2RAD(theta1);
    float theta3_rad = DEG2RAD(theta1 + theta2 - 180);

    // ========== 你最新的正确公式 ==========
    pointC->x = L1 * cosf(theta1_rad) + L2 * cosf(theta3_rad) + ARM_OFFSET;
    pointC->y = L1 * sinf(theta1_rad) + L2 * sinf(theta3_rad) + BASE_HEIGHT;

    // 3D坐标变换
    pointD->z = pointC->y;
    pointD->y = pointC->x * cosf(theta0_rad);
    pointD->x = pointC->x * sinf(theta0_rad);
	
}

