#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include <stdio.h>
#include "math.h"
#include "fashion_star_uart_servo.h"
#include "Inverse.h"
#include "arm_track.h"


//舵机角度补偿，输入舵机前加上补偿量，从舵机读出后减去补偿量
float Angle_Compensation0=0.0f;
float Angle_Compensation1=4.5f;
float Angle_Compensation2=-45.5f;


// 目标角度（纯数学角度，不含舵机补偿）
volatile float target_angle1 = 0.0f;  // 底座舵机ID2
volatile float target_angle2 = 90.0f; // 大臂舵机ID0
volatile float target_angle3 = 80.0f; // 小臂舵机ID1

// 当前角度（纯数学角度，不含舵机补偿）
volatile float current_angle1 = 0.0f;
volatile float current_angle2 = 90.0f;
volatile float current_angle3 = 80.0f;

//=============================函数1：逆运动学解算三个舵机角度============================================================================================================================

uint8_t inverse_kinematics(float x, float y, float z, float *theta0, float *theta1, float *theta2)
{
    // 1. 计算底座旋转角θ0 —— 输出 -180 ~ 180不限制
    *theta0 = atan2(x, y) * 180.0f / PI;

    // 2. 计算水平投影长度
    float r = sqrtf(x*x + y*y) - ARM_OFFSET;
    if (r < 0) return 1;

	
    // 3. 计算大臂关节到目标点的直线距离hypotenuse
	float delta_z = BASE_HEIGHT - z;
	float hypotenuse = sqrtf(delta_z*delta_z + r*r);
	// 工作空间边界检查
	if (hypotenuse > L1 + L2 || hypotenuse < fabsf(L1 - L2))  
	{
		return 1;
	}

	
	// 4. 计算大臂角度θ1（纯数学计算）
	//θ1_1
	float cos_theta1_1 = (L1*L1 + hypotenuse*hypotenuse - L2*L2) / (2 * L1 * hypotenuse);
	if(cos_theta1_1 > 1.0f)  cos_theta1_1 = 1.0f;
	if(cos_theta1_1 < -1.0f) cos_theta1_1 = -1.0f;
	float theta1_1 = acosf(cos_theta1_1) * 180.0f / PI;
	//θ1_2
	float theta1_2 = 0.0f;
	if (fabsf(delta_z) < 0.1f)
	{
		theta1_2 = 0.0f;
	}
	else if (delta_z > 0)
	{
		theta1_2 = (atan2(r, delta_z) * 180.0f / PI) - 90.0f;
	}
	else
	{
		theta1_2 = atan2(fabsf(delta_z), r) * 180.0f / PI;
	}
	//θ1 = θ1_1 + θ1_2
	*theta1 = theta1_1 + theta1_2;

	
    // 5. 计算小臂角度θ2（纯数学计算，得到两臂夹角）
	float cos_theta2 = (L1*L1 + L2*L2 - hypotenuse*hypotenuse) / (2 * L1 * L2);
	if(cos_theta2 > 1.0f)  cos_theta2 = 1.0f;
	if(cos_theta2 < -1.0f) cos_theta2 = -1.0f;
	*theta2 = acosf(cos_theta2) * 180.0f / PI;
	
	
	// ====================== 【舵机角度限幅】 ======================
	
	// 中臂舵机限幅（匹配范围：10°~180°）
	if(*theta1 < 10.0f)  *theta1 = 10.0f;  
	if(*theta1 > 180.0f)  *theta1 = 180.0f;  

	// 小臂舵机限幅（匹配范围：-45°~180°）
	if(*theta2 < 45.0f)  *theta2 = 45.0f;
	if(*theta2 > 180.0f)  *theta2 = 180.0f;

	// 底座舵机限幅 θ0：限制全范围
	if(*theta0 < -180)  *theta0 = -180;
	if(*theta0 > 180)   *theta0 = 180;

	// ================================================================
    return 0;
}




uint8_t flag_2 = 0;




//================================函数2：机械臂移动到指定坐标=========================================================================================================================

/**
 * @brief  机械臂移动到指定坐标
 * @param  x,y,z 目标坐标
 * @retval 无
 */
uint8_t arm_go_to_point(float x, float y, float z)
{
	float theta0, theta1, theta2;
	// 1. 计算角度
	
	flag_2=inverse_kinematics(x, y, z, &theta0, &theta1, &theta2);
	
	// 2. 用正确舵机ID控制 
	if(flag_2==0)
	{
		FSUS_SetServoAngle(&usart2, 2, theta0+Angle_Compensation0   , 200, 0, 0); // 底座舵机ID2 
		FSUS_SetServoAngle(&usart2, 0, theta1+Angle_Compensation1	, 200, 0, 0); // 大臂舵机ID0 
		FSUS_SetServoAngle(&usart2, 1, theta2+Angle_Compensation2	, 200, 0, 0); // 小臂舵机ID1 
	}
		
	// 3. 等待到位
	Delay_ms(10);
	
	return flag_2;
}


//================================函数3：路径规划：走直线=========================================================================================================================

// ====================== 核心参数配置（可根据需求调整） ======================
#define STEP_LENGTH     1.25f   // 每步移动距离(mm)，推荐1.0~2.0mm/步
#define MIN_STEPS       5       // 最小步数（防止距离太短步数太少）
#define MAX_STEPS       200     // 最大步数（防止距离太长计算量太大）
// ==========================================================================

uint8_t move_smooth_auto(float x1, float y1, float z1, float x2, float y2, float z2, uint16_t speed)
{
    float theta0, theta1, theta2;//目标角度
//    float current0, current1, current2;//当前角度
    
    // 前置检查：起点终点是否可达
    if (inverse_kinematics(x1, y1, z1, &theta0, &theta1, &theta2) != 0) return 1;
    if (inverse_kinematics(x2, y2, z2, &theta0, &theta1, &theta2) != 0) return 2;
    
    // 计算两点之间的总距离
    float dx_total = x2 - x1;
    float dy_total = y2 - y1;
    float dz_total = z2 - z1;
    float total_distance = sqrtf(dx_total*dx_total + dy_total*dy_total + dz_total*dz_total);
    
    // 距离为0，直接返回
    if (total_distance < 0.1f) return 0;
    
    // 自动计算步数：总距离 ÷ 每步距离
    uint16_t steps = (uint16_t)(total_distance / STEP_LENGTH);
    
    // 步数限制：不小于最小步数，不大于最大步数
    if (steps < MIN_STEPS) steps = MIN_STEPS;
    if (steps > MAX_STEPS) steps = MAX_STEPS;
    
    // 计算每步增量
    float dx = dx_total / (float)steps;
    float dy = dy_total / (float)steps;
    float dz = dz_total / (float)steps;

    // 插补循环
    for (uint16_t i = 0; i <= steps; i++)
    {
        float x = x1 + dx * (float)i;
        float y = y1 + dy * (float)i;
        float z = z1 + dz * (float)i;

        if (inverse_kinematics(x, y, z, &theta0, &theta1, &theta2) == 0)
        {

			FSUS_SetServoAngle(&usart2, 2, theta0+Angle_Compensation0   , 50, 0, 0); // 底座舵机ID2 
			FSUS_SetServoAngle(&usart2, 0, theta1+Angle_Compensation1	, 50, 0, 0); // 大臂舵机ID0 
			FSUS_SetServoAngle(&usart2, 1, theta2+Angle_Compensation2	, 50, 0, 0); // 小臂舵机ID1 
            
			
			Delay_ms(50);
			
//			// 等待到位（不好使）
//            uint32_t timeout = 0;
//            do
//            {
//                FSUS_QueryServoAngle(&usart2, 2, &current0);
//                FSUS_QueryServoAngle(&usart2, 0, &current1);
//                FSUS_QueryServoAngle(&usart2, 1, &current2);
//                Delay_ms(5);
//                timeout++;
//            } while ((fabsf(current0 - theta0) > 1.0f || 
//                      fabsf(current1 - theta1) > 1.0f || 
//                      fabsf(current2 - theta2) > 1.0f) && timeout < 10);
        }
    }
    
    return 0;
}


//================================函数3.5：手动版走直线=========================================================================================================================

// 手动指定步数版本（用于调试）
uint8_t move_smooth_manual(float x1, float y1, float z1, float x2, float y2, float z2, uint16_t steps, uint16_t speed)
{
    float theta0, theta1, theta2;
    float current0, current1, current2;
    
    if (inverse_kinematics(x1, y1, z1, &theta0, &theta1, &theta2) != 0) return 1;
    if (inverse_kinematics(x2, y2, z2, &theta0, &theta1, &theta2) != 0) return 2;
    
    float dx = (x2 - x1) / (float)steps;
    float dy = (y2 - y1) / (float)steps;
    float dz = (z2 - z1) / (float)steps;

    for (uint16_t i = 0; i <= steps; i++)
    {
        float x = x1 + dx * (float)i;
        float y = y1 + dy * (float)i;
        float z = z1 + dz * (float)i;

        if (inverse_kinematics(x, y, z, &theta0, &theta1, &theta2) == 0)
        {
            FSUS_SetServoAngle(&usart2, 2, theta0, speed, speed, 0);
            FSUS_SetServoAngle(&usart2, 0, theta1+3.0f, speed, speed, 0);
            FSUS_SetServoAngle(&usart2, 1, theta2-45.0f, speed, speed, 0);
            
            uint32_t timeout = 0;
            do
            {
                FSUS_QueryServoAngle(&usart2, 2, &current0);
                FSUS_QueryServoAngle(&usart2, 0, &current1);
                FSUS_QueryServoAngle(&usart2, 1, &current2);
                Delay_ms(5);
                timeout++;
            } while ((fabsf(current0 - theta0) > 1.0f || 
                      fabsf(current1 - theta1) > 1.0f || 
                      fabsf(current2 - theta2) > 1.0f) && timeout < 10);
        }
    }
    
    return 0;
}

















//================================函数4：角度差值计算（辅助）=========================================================================================================================
static float angle_diff(float a, float b)
{
    float diff = a - b;
    while(diff > 180.0f) diff -= 360.0f;
    while(diff < -180.0f) diff += 360.0f;
    return diff;
}


//================================函数1.5：逆运动学解算三个舵机角度-2（似乎没什么用）=========================================================================================================================
//逆运动学核心函数
 int InverseKinematics_Base_2(float x, float y, float z, int elbow_up,float *theta0, float *theta1, float *theta2)
{
    // 1. 计算底座旋转角θ0 —— 输出 -180 ~ 180不限制
    *theta0 = atan2(x, y) * 180.0f / PI;

    // 2. 计算水平投影长度
    float r = sqrtf(x*x + y*y) - ARM_OFFSET;
    if (r < 0.0f) return 1;

    // 3. 计算大臂关节到目标点的直线距离
	float delta_z = BASE_HEIGHT - z;
	float hypotenuse = sqrtf(delta_z*delta_z + r*r);

	// 4. 计算大臂角度θ1（纯数学计算）
	float cos_theta1_1 = (L1*L1 + hypotenuse*hypotenuse - L2*L2) / (2 * L1 * hypotenuse);
	if(cos_theta1_1 > 1.0f)  cos_theta1_1 = 1.0f;
	if(cos_theta1_1 < -1.0f) cos_theta1_1 = -1.0f;
	float theta1_1 = acosf(cos_theta1_1) * 180.0f / PI;
	
	float theta1_2 = 0.0f;
	
	if (fabsf(delta_z) < 0.1f)
	{
		theta1_2 = 0.0f;
	}
	else if (delta_z > 0)
	{
		theta1_2 = (atan2(r, delta_z) * 180.0f / PI) - 90.0f;
	}
	else
	{
		theta1_2 = atan2(fabsf(delta_z), r) * 180.0f / PI;
	}

	*theta1 = theta1_1 + theta1_2;


	// ====================== 【大臂舵机角度限幅】 ======================
	// 限制最终舵机角度，匹配硬件：最小10°，最大180°		
	if(*theta1 < 10.0f)  *theta1 = 10.0f;  
	if(*theta1 > 180.0f)  *theta1 = 180.0f;  
	// =========================================================================
	
	
   // 5. 计算小臂角度θ2（纯数学计算，得到两臂夹角）
	float cos_theta2 = (L1*L1 + L2*L2 - hypotenuse*hypotenuse) / (2 * L1 * L2);
	if(cos_theta2 > 1.0f)  cos_theta2 = 1.0f;
	if(cos_theta2 < -1.0f) cos_theta2 = -1.0f;

	*theta2 = acosf(cos_theta2) * 180.0f / PI;
		
	if(elbow_up) {
       *theta2 = -*theta2;
  } //1: elbow up, 0: elbow down

	// 小臂舵机限幅（匹配范围：45°~180°）
	if(*theta2 < 45.0f)   *theta2 = 45.0f;
	if(*theta2 > 180.0f)  *theta2 = 180.0f;

 
	// ===================== 最终限幅（舵机支持负数） =====================
	// 底座 θ0：限制全范围
	if(*theta0 < -180.0f)   *theta0 = -180.0f;
	if(*theta0 >  180.0f)   *theta0 = 180.0f;

	if(*theta0< JOINT1_MIN || *theta0 > JOINT1_MAX ||
     *theta1 < JOINT2_MIN || *theta1 > JOINT2_MAX ||
     *theta2 < JOINT3_MIN || *theta2 > JOINT3_MAX) {
       return 2;  // 关节角度超限    
		 }
    return 0;
}






//================================函数6：寻找最优解（可能也没什么用）=========================================================================================================================
int InverseKinematics_Auto(float x_mm, float y_mm, float z_mm)
{
     float ang1_down, ang2_down, ang3_down;
    float ang1_up,   ang2_up,   ang3_up;
    int status_down, status_up;
    int valid_down = 0, valid_up = 0;

    status_down = InverseKinematics_Base_2(x_mm, y_mm, z_mm, 0,
                                         &ang1_down, &ang2_down, &ang3_down);
    if (status_down == 0) valid_down = 1;

    status_up = InverseKinematics_Base_2(x_mm, y_mm, z_mm, 1,
                                       &ang1_up, &ang2_up, &ang3_up);
    if (status_up == 0) valid_up = 1;

    // 修复：优先返回不可达错误
    if (!valid_down && !valid_up) {
        if (status_down == 1 || status_up == 1) {
            return 1;
        } else {
            return 2;
        }
    }

    if (valid_down && !valid_up) {
        target_angle1 = ang1_down;
        target_angle2 = ang2_down;
        target_angle3 = ang3_down;
    }
    else if (!valid_down && valid_up) {
        target_angle1 = ang1_up;
        target_angle2 = ang2_up;
        target_angle3 = ang3_up;
    }
    else {
        float diff_down = fabsf(angle_diff(ang1_down, current_angle1)) +
                          fabsf(ang2_down - current_angle2) +
                          fabsf(ang3_down - current_angle3);

        float diff_up   = fabsf(angle_diff(ang1_up,   current_angle1)) +
                          fabsf(ang2_up   - current_angle2) +
                          fabsf(ang3_up   - current_angle3);

        if (diff_up < diff_down) {
            target_angle1 = ang1_up;
            target_angle2 = ang2_up;
            target_angle3 = ang3_up;
        } else {
            target_angle1 = ang1_down;
            target_angle2 = ang2_down;
            target_angle3 = ang3_down;
        }
    }

    return 0;
}




















////已知机械臂三个关节转了多少度，精确算出末端摄像头现在在空间中的具体位置和朝向（现在在哪里）
///**
// * @brief  正运动学解算：根据关节角度计算末端摄像头的世界位姿
// * @param  a1: 底座角度(度)，纯数学角度，不含补偿
// * @param  a2: 大臂角度(度)，纯数学角度，不含补偿
// * @param  a3: 小臂角度(度)，纯数学角度，不含补偿
// * @param  x,y,z: 输出摄像头世界坐标(mm)
// * @param  roll,pitch,yaw: 输出摄像头姿态(弧度)roll：横滚角（绕 Y 轴转），三自由度机械臂无法改变，恒为 0
//												pitch：俯仰角（绕 X 轴转），摄像头上下摆动的角度
//												yaw：偏航角（绕 Z 轴转），摄像头左右摆动的角度
// */
//void GetCameraPosition(float a1, float a2, float a3,
//                       float *x, float *y, float *z,
//                       float *roll, float *pitch, float *yaw)
//{
//    // 角度转弧度
//    float rad1 = a1 * PI / 180.0f;
//    float rad2 = a2 * PI / 180.0f;
//    float rad3 = a3 * PI / 180.0f;
//    
//    // 计算末端水平投影长度
//    float r = L1 * sinf(rad2) + L2 * sinf(rad2 + rad3) + ARM_OFFSET;
//    
//    // 计算世界坐标
//    *x = r * sinf(rad1);
//    *y = r * cosf(rad1);
//    *z = BASE_HEIGHT - L1 * cosf(rad2) - L2 * cosf(rad2 + rad3);
//    
//    // 计算摄像头姿态
//    *roll = 0.0f;          // 三自由度机械臂无法改变横滚角
//    *pitch = rad2 + rad3;  // 俯仰角
//    *yaw = rad1;           // 偏航角
//}
