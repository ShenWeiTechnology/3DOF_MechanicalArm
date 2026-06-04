#ifndef __ARM_TRACK_H
#define __ARM_TRACK_H


#include "Kalman.h"
#include "stm32f10x.h"
#include <math.h>
#include "Inverse.h"


// 关节角度限位（与逆运动学内部限幅一致，双重安全保护）
#define JOINT1_MIN             -180.0f  /* 底座舵机ID2最小角度(°) */
#define JOINT1_MAX             180.0f   /* 底座舵机ID2最大角度(°) */
#define JOINT2_MIN             10.0f    /* 大臂舵机ID0最小角度(°) */
#define JOINT2_MAX             180.0f   /* 大臂舵机ID0最大角度(°) */
#define JOINT3_MIN             45.0f    /* 小臂舵机ID1最小角度(°) */
#define JOINT3_MAX             180.0f   /* 小臂舵机ID1最大角度(°) */

// 相机参数
//#define CAM_REF_DISTANCE       200.0f   /* 目标平面参考距离(mm) *（没用）
#define CAM_HFOV_DEG           68.2f    /* 相机水平视场角(°) */
#define CAM_VFOV_DEG           51.8f    /* 相机垂直视场角(°) */
#define CAM_IMAGE_W            320      /* OpenMV图像宽度(像素) */
#define CAM_IMAGE_H            240      /* OpenMV图像高度(像素) */

// 追踪逻辑参数
#define TARGET_LOST_TIMEOUT_MS 500      /* 目标丢失超时时间(ms) */

/************************ 外部全局变量声明 ************************/
// 目标角度（纯数学角度，不含舵机补偿）
extern volatile float target_angle1; // 底座舵机ID2
extern volatile float target_angle2; // 大臂舵机ID0
extern volatile float target_angle3; // 小臂舵机ID1

// 当前角度（纯数学角度，不含舵机补偿）
extern volatile float current_angle1;
extern volatile float current_angle2;
extern volatile float current_angle3;
  
// 小球相对相机坐标
extern volatile float ball_cam_x;   // 相对相机X坐标(mm)
extern volatile float ball_cam_y;   // 相对相机Y坐标(mm)
extern volatile float ball_cam_z;   // 相对相机Z坐标(mm)

/************************ 外部依赖函数声明 ************************/
/**
 * @brief  正运动学解算：根据关节角度计算摄像头世界位姿
 * @note   
 */
extern void GetCameraPosition(float a1, float a2, float a3,
                              float *x, float *y, float *z,
                              float *roll, float *pitch, float *yaw);

/************************ 本模块函数声明 ************************/
/**
 * @brief  初始化小球追踪卡尔曼滤波器
 * @note   main函数初始化阶段调用一次
 */
void ArmTrack_KalmanInit(void);

/**
 * @brief  重置小球追踪状态和滤波器
 * @note   目标丢失后重新检测时调用
 */
void ResetBallTracking(void);

/**
 * @brief  一站式小球追踪处理函数
 * @param  px: OpenMV输出的目标X像素坐标
 * @param  py: OpenMV输出的目标Y像素坐标
 * @param  distance_mm: 目标到相机的距离，无测距传CAM_REF_DISTANCE
 * @retval 0=处理失败，1=处理成功
 */
uint8_t ProcessBallTracking(uint16_t px, uint16_t py, uint16_t distance_mm);

/**
 * @brief  判断目标是否丢失
 */
uint8_t ArmTrack_IsTargetLost(uint32_t last_detect_ms, uint32_t curr_ms);


// GetBallRelativeCamera 函数声明
uint8_t  GetBallRelativeCamera(uint16_t px, uint16_t py, uint16_t distance_mm);


#endif // __ARM_TRACK_H
