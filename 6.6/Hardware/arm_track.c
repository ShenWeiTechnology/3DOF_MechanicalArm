#include "arm_track.h"

/************************ 全局变量定义 ************************/
volatile float ball_cam_x = 0;      // 相对相机X
volatile float ball_cam_y = 0.0f;   // 相对相机Y
volatile float ball_cam_z = 0.0f;   // 相对相机Z

/************************ 内部滤波变量 ************************/
static Kalman_Filter kf_cam_x, kf_cam_y, kf_cam_z;

/************************ 函数实现 ************************/

// 初始化滤波
void ArmTrack_KalmanInit(void)
{
    Kalman_Init(&kf_cam_x, 0.02f, 0.4f, 1.0f, 0.0f);
    Kalman_Init(&kf_cam_y, 0.02f, 0.4f, 1.0f, 0.0f);
    Kalman_Init(&kf_cam_z, 0.03f, 0.6f, 1.0f, 0.0f);
}

// 重置滤波
void ResetBallTracking(void)
{
    Kalman_Reset(&kf_cam_x, 0.0f);
    Kalman_Reset(&kf_cam_y, 0.0f);
    Kalman_Reset(&kf_cam_z, 0.0f);
}

// 核心：OpenMV像素 → 小球【相对摄像头】的坐标
uint8_t GetBallRelativeCamera(uint16_t px, uint16_t py, uint16_t distance_mm)
{
    // 输入检查
    if (px >= CAM_IMAGE_W || py >= CAM_IMAGE_H||px<=0||py<=0)
    {
        ball_cam_x = 1;
        ball_cam_y = 1;
        ball_cam_z = 1;
        return 1;//报错返回1
    }

    // 1. 像素转归一化坐标 (-1 ~ 1)
    float norm_x = (px - CAM_IMAGE_W/2.0f) / (CAM_IMAGE_W/2.0f);
    float norm_y = (CAM_IMAGE_H/2.0f - py) / (CAM_IMAGE_H/2.0f);

    // 2. 转弧度
    float angle_h = norm_x * (CAM_HFOV_DEG/2.0f) * PI/180.0f;
    float angle_v = norm_y * (CAM_VFOV_DEG/2.0f) * PI/180.0f;

    // 3. 计算【小球相对摄像头】的坐标（核心！）
    float x = distance_mm * cosf(angle_v) * sinf(angle_h);
    float y = distance_mm * cosf(angle_v) * cosf(angle_h);
    float z = distance_mm * sinf(angle_v);

//    // 4. 卡尔曼平滑
//    ball_cam_x = Kalman_Update(&kf_cam_x, x);
//    ball_cam_y = Kalman_Update(&kf_cam_y, y);
//    ball_cam_z = Kalman_Update(&kf_cam_z, z);
	
	  // 4. 跳过卡尔曼
    ball_cam_x = (int)x;
    ball_cam_y = (int)y;
    ball_cam_z = (int)z;
	
	return 0;
}



