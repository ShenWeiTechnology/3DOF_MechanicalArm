#include "PID.h"
#include "OLED.h"

/**
  * @brief  PID初始化函数
  * @param  pid: PID结构体指针
  * @param  Kp: 比例系数
  * @param  Ki: 积分系数
  * @param  Kd: 微分系数
  * @retval 无
  */
void PID_Init(PID_Handle *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;         // 设置比例值
//	OLED_ShowNum(4,5,pid->Kp,4);
    pid->Ki = Ki;         // 设置积分值
    pid->Kd = Kd;         // 设置微分值
    pid->integral = 0;    // 积分值清零
    pid->prev_err = 0;    // 上一次偏差清零
}




/**
 * @brief  增量式PID初始化函数
 * @param  pid: PID结构体指针
 * @param  Kp: 比例系数
 * @param  Ki: 积分系数
 * @param  Kd: 微分系数
 */
void PID_Init2(PID_Handle2 *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    // 历史变量全部初始化为0
    pid->e_prev1 = 0.0f;
    pid->e_prev2 = 0.0f;
    pid->u_prev = 0.0f; // 初始控制量为0，舵机默认在中位
}









/**
  * @brief  PID计算核心函数
  * @param  pid: PID结构体指针
  * @param  err: 当前偏差（目标值-实际值）
  * @retval PID计算后的控制量
  */
float PID_Calc(PID_Handle *pid, int16_t err)
{
    float P = pid->Kp * err;// 1. 比例项 P：偏差越大，输出越大

    pid->integral += err; // 2. 积分项 I：累计偏差，消除静差
    if (pid->integral > 160)  pid->integral = 160;
    if (pid->integral < -160) pid->integral = -160;// 积分限幅，防止积分饱和（数值过大失控）
    float I = pid->Ki * pid->integral; 

    float D = pid->Kd * (err - pid->prev_err); // 3. 微分项 D：根据偏差变化率，提前刹车、防抖
	
    pid->prev_err = err;// 保存当前偏差，供下一次计算使用
    return P + I + D;// PID三项相加，输出总控制量
//	return P;
}






/**
 * @brief  增量式PID计算函数
 * @param  pid: PID结构体指针
 * @param  err: 当前偏差值
 * @retval 控制量增量Δu(k)
 */
float PID_Inc_Calc(PID_Handle2 *pid, float err)
{
    float delta_u;

    // 增量式PID核心公式（严格按照数学推导实现）
//    delta_u = pid->Kp * (err - pid->e_prev1)
//            + pid->Ki * err
//            + pid->Kd * (err - 2 * pid->e_prev1 + pid->e_prev2);
	    delta_u = pid->Kp * (err )
            + pid->Ki * err
            + pid->Kd * (err - 2 * pid->e_prev1 + pid->e_prev2);

    // 更新历史偏差（必须在计算完成后更新，顺序不能错！）
    pid->e_prev2 = pid->e_prev1;
    pid->e_prev1 = err;

    return delta_u;
}















/**
  * @brief  视觉偏差转舵机PWM值（直接输出给舵机）
  * @param  pid: PID结构体指针
  * @param  camera_err: 视觉模块（OpenMV）传来的偏差
  * @retval 舵机可用的PWM值（500~2500）
  */
uint16_t Servo_PWM_Update(PID_Handle *pid, int16_t camera_err)
{
    // 1. 偏差限幅：限制偏差范围，防止失控
    if (camera_err > 160)  camera_err = 160;
    if (camera_err < -160) camera_err = -160;

    // 2. 调用PID函数，计算控制量
    float control = PID_Calc(pid, camera_err);

    // 3. 控制量限幅：防止输出过大
    if (control > 1000)  control = 1000;
    if (control < -1000) control = -1000;

    // 4. 计算舵机PWM：1500为中位，±1000为左右范围
    int16_t pwm = 1500 + (int16_t)control;

    // 5. PWM安全限幅（舵机标准范围：500~2500）
    if (pwm > 2500) pwm = 2500;
    if (pwm < 500)  pwm = 500;

    // 返回最终可直接使用的舵机PWM
    return pwm;
}

//uint16_t Servo_PWM_Update2(PID_Handle *pid, int16_t camera_err)
//{
//    // 1. 偏差限幅：限制偏差范围，防止失控
//    if (camera_err > 120)  camera_err = 120;
//    if (camera_err < -120) camera_err = -120;

//    // 2. 调用PID函数，计算控制量
//    float control = PID_Calc(pid, camera_err);

////    // 3. 控制量限幅：防止输出过大
////    if (control > 1000)  control = 1000;
////    if (control < -1000) control = -1000;

//    // 4. 计算舵机PWM：1500为中位，±1000为左右范围
//    int16_t pwm = 1500 + (int16_t)control;

//    // 5. PWM安全限幅（舵机标准范围：500~2500）
//    if (pwm > 2500) pwm = 2500;
//    if (pwm < 500)  pwm = 500;

//    // 返回最终可直接使用的舵机PWM
//    return pwm;
//}


uint16_t Servo_PWM_Update2(PID_Handle2 *pid, int16_t camera_err)
{
    // 1. 偏差限幅：限制偏差范围，防止失控（完全保留原逻辑）
    if (camera_err > 120)  camera_err = 120;
    if (camera_err < -120) camera_err = -120;

    // 2. 调用增量式PID，计算控制量增量Δu
    float delta_control = PID_Inc_Calc(pid, (float)camera_err);

    // 3. 累加得到当前绝对控制量 u(k) = u(k-1) + Δu(k)
    float control = pid->u_prev + delta_control;
	OLED_ShowNum(2, 1, delta_control, 5);

    // 4. 控制量限幅（必须启用！原注释掉的代码现在是核心）
    // 作用：防止控制量超出舵机行程，同时抑制积分饱和
    if (control > 1000)  control = 1000;
    if (control < -1000) control = -1000;

    // 5. 更新上一次控制量（必须在限幅之后更新！否则积分饱和依然存在）
    pid->u_prev = control;

    // 6. 计算舵机PWM：1500为中位，±1000为左右范围（完全保留原逻辑）
    int16_t pwm = 1500 + (int16_t)control;

    // 7. PWM安全限幅（舵机标准范围：500~2500）（完全保留原逻辑）
    if (pwm > 2500) pwm = 2500;
    if (pwm < 500)  pwm = 500;

    // 返回最终可直接使用的舵机PWM
    return pwm;
}








