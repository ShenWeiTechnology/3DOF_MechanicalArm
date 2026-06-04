#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "ExitKey.h"
#include "Servo.h"
#include "Serial.h"
#include "Serial2.h"
#include <stdio.h>
#include "PID.h" 
#include "PWM.h"
#include "math.h"
#include "usart.h"
#include "sys_tick.h"
#include "fashion_star_uart_servo.h"
#include "Inverse.h"
#include "Kalman.h"
#include "arm_track.h"

PID_Handle servo_pid;
PID_Handle2 servo_pid2;
PID_Handle2 servo_pid3;

//舵机角度补偿，输入舵机前加上补偿量，从舵机读出后减去补偿量
extern float Angle_Compensation0;
extern float Angle_Compensation1;
extern float Angle_Compensation2;



	// 使用串口1作为舵机控制的端口
// <接线说明>
// STM32F103 PA9(Tx) 	<----> 串口舵机转接板 Rx
// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
// STM32F103 GND 		<----> 串口舵机转接板 GND
// STM32F103 V5 		<----> 串口舵机转接板 5V
// <注意事项>
// 使用前确保已设置usart.h里面的USART1_ENABLE为1
// 设置完成之后, 将下行取消注释
   Usart_DataTypeDef* servoUsart = &usart2; 

//// 舵机控制相关的参数
// 时间间隔ms  
// 可以尝试修改设置更小的时间间隔，例如500ms
   uint16_t interval = 2000; 
// 舵机执行功率 mV 默认为0	
   uint16_t power = 0;
// 设置舵机角度的时候, 是否为阻塞式 
// 0:不等待 1:等待舵机旋转到特定的位置; 
uint8_t wait = 0; 






// 舵机1状态定义
#define SERVO1_STOP_FO 	 	0   // 停止_接下来正转360°
#define SERVO1_FORWARD  	1   // 正转360°
#define SERVO1_STOP_RE 		2   // 停止_接下来反转360°
#define SERVO1_REVERSE 	 	3   // 反转360°

#define SERVO2_STOP_FO 	 	0   // 停止_接下来正转360°
#define SERVO2_FORWARD  	1   // 正转360°
#define SERVO2_STOP_RE 		2   // 停止_接下来反转360°
#define SERVO2_REVERSE 	 	3   // 反转360°

#define SERVO3_STOP_FO 	 	0   // 停止_接下来正转360°
#define SERVO3_FORWARD  	1   // 正转360°
#define SERVO3_STOP_RE 		2   // 停止_接下来反转360°
#define SERVO3_REVERSE 	 	3   // 反转360°

// 全局变量
uint8_t  Servo1_State = SERVO1_STOP_FO;
uint32_t Servo1_Timer = 0;
uint32_t Servo2_Timer = 0;
uint32_t Servo3_Timer = 0;
const    uint32_t SERVO1_RUN_TIME = 190;  // 转360°的时间，单位ms
const    uint32_t SERVO2_RUN_TIME = 1;  // 转360°的时间，单位ms
const    uint32_t SERVO3_RUN_TIME = 1;  // 转360°的时间，单位ms


volatile uint32_t SysTick_Count = 0;  // 系统毫秒计数器



//变量

uint8_t mode_first_enter = 0;

//视觉模块
extern uint16_t OpenMV_Data1;
extern uint16_t OpenMV_Data2;
extern uint16_t OpenMV_Data3;

//系统控制

uint8_t cursor_pos = 1;			//光标
uint8_t page_offset = 0;		//翻页
uint8_t sys_state = 0;

//屏幕
uint8_t clean1 = 0;
uint8_t clean2 = 0;
uint8_t clean3 = 0;
uint8_t v = 0;					//清屏标志

//舵机

uint8_t Flag;

uint8_t Servo1_State;
uint8_t Servo2_State;
uint8_t Servo3_State;

int16_t Angle,Angle_key;
uint8_t Angle_flag=1;

int8_t Servo1_Dir = 1;
int8_t Servo2_Dir = 1;
int8_t Servo3_Dir = 1;

int16_t Servo1_Angle = 0;
int16_t Servo2_Angle = 90;
int16_t Servo3_Angle = 80;

uint8_t Selected_Servo = 1;

//按键
extern uint8_t Key_Value;
extern int delta_control;

//屏幕函数
void Welcome_Interface(void);
void Menu_Interface(void);
void Mode_Run(uint8_t mode);


// 各模式业务函数
void Mode1_Run(void);
void Mode2_Run(void);
void Mode3_Run(void);
void Mode4_Run(void);
void Mode5_Run(void);

void Mode_Exit_Reset(void);  // 新增：模式退出统一重置函数



// ===================== 模式退出统一重置函数（核心修复）=====================
void Mode_Exit_Reset(void)
{
    // 1. 清屏标志重置
    v = 0;
    clean1 = 0;
    clean2 = 0;
    
    // 2. 模式进入标志重置
    mode_first_enter = 0;
    
    // 3. 舵机状态重置（回到安全位置）
    Servo1_State = SERVO1_STOP_FO;
    Servo_SetAngle1(90);  // 底座回中
    
    // 4. 舵机定时器重置
    Servo1_Timer = 0;
    Servo2_Timer = 0;
    Servo3_Timer = 0;
//    
//    // 5. PID控制器重置（关键！清除积分累积）
//    PID_Reset(&servo_pid);
//    PID_Reset2(&servo_pid2);
//    PID_Reset2(&servo_pid3);
    
    // 6. 按键值清零
    Key_Value = 0;
}



int main(void)
{
	
	
	
	//初始化
	OLED_Init();
	ExitKey_Init();
	Servo_Init();
	Serial_Init();
	Serial2_Init();
	
	// 嘀嗒定时器初始化
	SysTick_Init();
   // 串口初始化
	Usart_Init();

   
	
	//主循环
    while(1)
    {
        if(sys_state == 0)
        {
            Welcome_Interface();
            if(Key_Value != 0)
            {
                sys_state = 1;
                cursor_pos = 1;
                page_offset = 0;
                Key_Value = 0;
                Delay_ms(30);
            }
        }
        else if(sys_state == 1)
        {
			if(!clean1)
			{
				OLED_Clear();
				clean1=1;
			}
			
            Menu_Interface();
            
            if(Key_Value != 0)
            {
                switch(Key_Value)
                {
                    case 1:
                        if(cursor_pos > 1)
                        {
                            cursor_pos--;
                            OLED_Clear();
                            if(cursor_pos < page_offset + 1) page_offset--;
                        }
                        break;
                    
                    case 2:
                        if(cursor_pos < 5)
                        {
                            cursor_pos++;
                            OLED_Clear();
                            if(cursor_pos > page_offset + 4) page_offset++;
                        }
                        break;
                    
                    case 3:
                        sys_state = cursor_pos + 1;
                        if(sys_state==2) v=0;
                        OLED_Clear();
                        break;
					case 4:
						sys_state = 1;
						clean1 = 0;
						clean2 = 0;
						OLED_Clear();
						Key_Value = 0;
						break;
                }
                Key_Value = 0;
            }
        }
        else if(sys_state >= 2 && sys_state <= 6)
        {
			if(!clean2)
			{
				OLED_Clear();
				clean2=1;
			}
            Mode_Run(sys_state - 1);
        }
		Delay_ms(1);
    }
}

//各个页面函数
void Welcome_Interface(void)
{
	
    OLED_ShowString(1, 3, "Welcome!");
}

void Menu_Interface(void)
{
	mode_first_enter=0;
//	OLED_Clear();
	if(!clean3)
	{
		OLED_Clear();
		clean3=1;
	}
	
    if(page_offset == 0)
    {
        OLED_ShowString(1, 1, "Mode1");
        OLED_ShowString(2, 1, "Mode2");
        OLED_ShowString(3, 1, "Mode3");
        OLED_ShowString(4, 1, "Mode4");
		
		if(cursor_pos == 1)  OLED_ShowString(1, 15, ">>");
		if(cursor_pos == 2)  OLED_ShowString(2, 15, ">>");
		if(cursor_pos == 3)  OLED_ShowString(3, 15, ">>");
		if(cursor_pos == 4)  OLED_ShowString(4, 15, ">>");
    }
    else if(page_offset == 1)
    {
        OLED_ShowString(1, 1, "Mode2");
        OLED_ShowString(2, 1, "Mode3");
        OLED_ShowString(3, 1, "Mode4");
        OLED_ShowString(4, 1, "Mode5");
		
		if(cursor_pos == 2)  OLED_ShowString(1, 15, ">>");
		if(cursor_pos == 3)  OLED_ShowString(2, 15, ">>");
		if(cursor_pos == 4)  OLED_ShowString(3, 15, ">>");
		if(cursor_pos == 5)  OLED_ShowString(4, 15, ">>");
    }
}


// ===================== 模式调度函数（5个模式完整版）=====================
void Mode_Run(uint8_t mode)
{
    switch(mode)
    {
        // ========== 模式1 ==========
        case 1:
            if(mode_first_enter == 0)
            {
                OLED_Clear();
                OLED_ShowString(1,1,"Running Mode 1");
                mode_first_enter = 1;
                v = 0; // 舵机模式屏幕重置
            }
            Mode1_Run();
            break;

        // ========== 模式2 ==========
        case 2:
            if(mode_first_enter == 0)
            {
                OLED_Clear();
                OLED_ShowString(1,1,"Running Mode 2");
                mode_first_enter = 1;
            }
            Mode2_Run();
            break;

        // ========== 模式3 ==========
        case 3:
            if(mode_first_enter == 0)
            {
                OLED_Clear();
                OLED_ShowString(1,1,"Running Mode 3");
                mode_first_enter = 1;
            }
            Mode3_Run();
            break;

        // ========== 模式4 ==========
        case 4:
            if(mode_first_enter == 0)
            {
                OLED_Clear();
                OLED_ShowString(1,1,"Running Mode 4");
                mode_first_enter = 1;
				
				PID_Init(&servo_pid, 1.5f, 0.00f, 0.08f);// 初始化PID参数
				PID_Init2(&servo_pid2, 0.5f, 0.0f, 0.08f);// 初始化PID参数
				PID_Init2(&servo_pid3, 0.3f, 0.0f, 0.15f);// 初始化PID参数
				
				
				
            }
            Mode4_Run();
            break;

        // ========== 模式5 ==========
        case 5:
            if(mode_first_enter == 0)
            {
                OLED_Clear();
                OLED_ShowString(1,1,"Running Mode 5");
                mode_first_enter = 1;
            }
            Mode5_Run();
            break;

        default: break;
    }
}





// ===================== 模式1：舵机控制（最终版：60→90→120循环）=====================
void Mode1_Run(void)
{
    // 舵机2和3的角度状态变量（记录当前位置）
    static uint8_t servo2_angle_state = 0; // 0: -50.0度  1: 130.0度
    static uint8_t servo3_angle_state = 0; // 0: -45.0度  1: 135.0度
    
    // 串行舵机控制参数（和模式5完全一致）
    Usart_DataTypeDef* servoUsart = &usart2; 
    uint16_t interval = 2000; 
    uint16_t power = 0;
    uint8_t wait = 0;

    OLED_ShowNum(4, 1, SysTick_Count, 3);
    OLED_ShowNum(4, 6, Servo1_Timer, 3);

    SysTick_Count++;

    if (v == 0)
    {
        OLED_ShowString(1, 1, "Servo Control");
        v = 1;
    }

    uint8_t key = 0;

    // ================= 读取按键 =================
    if(Key_Value != 0)
    {
        key = Key_Value;
        Key_Value = 0;
    }

    // ================= 按键2：切换舵机 =================
    if (key == 2)
    {
        Selected_Servo++;

        if (Selected_Servo > 3)
        {
            Selected_Servo = 1;
        }
    }

    // ================= 按键3：舵机2/3回预设角度 =================
    if(key == 3)
    {
        // 舵机2回到-50.0度
        servo2_angle_state = 0;
        FSUS_SetServoAngle(servoUsart, 0, 94.5, interval, power, wait);
        //3
        // 舵机3回到-45.0度
        servo3_angle_state = 0;
        FSUS_SetServoAngle(servoUsart, 1, 44.5, interval, power, wait);
    }

    // ================= 按键1：控制舵机 =================
    if (key == 1)
    {
        // ================= 舵机1（360°舵机）【完全保留原有逻辑】=================
        if (Selected_Servo == 1)
        {
            if(Servo1_State == SERVO1_STOP_FO)
            {
                Servo1_State = SERVO1_FORWARD;
                Servo1_Timer = SysTick_Count;
            }
            else if(Servo1_State == SERVO1_FORWARD)
            {
                Servo1_State = SERVO1_STOP_RE;
            }
            else if(Servo1_State == SERVO1_STOP_RE)
            {
                Servo1_State = SERVO1_REVERSE;
                Servo1_Timer = SysTick_Count;
            }
            else if(Servo1_State == SERVO1_REVERSE)
            {
                Servo1_State = SERVO1_STOP_FO;
            }
        }

        // ================= 舵机2（串行总线舵机）【新逻辑】=================
        if (Selected_Servo == 2)
        {
            if(servo2_angle_state == 0)
            {
                // 运动到130.0度
                FSUS_SetServoAngle(servoUsart, 0, 130.0, interval, power, wait);
                servo2_angle_state = 1;
            }
            else
            {
                // 运动到-50.0度
                FSUS_SetServoAngle(servoUsart, 0, -50.0, interval, power, wait);
                servo2_angle_state = 0;
            }
        }

        // ================= 舵机3（串行总线舵机）【新逻辑】=================
        if (Selected_Servo == 3)
        {
            if(servo3_angle_state == 0)
            {
                // 运动到135.0度
                FSUS_SetServoAngle(servoUsart, 1, 135.0, interval, power, wait);
                servo3_angle_state = 1;
            }
            else
            {
                // 运动到-45.0度
                FSUS_SetServoAngle(servoUsart, 1, -45.0, interval, power, wait);
                servo3_angle_state = 0;
            }
        }
    }

    // =========================================================
    // 舵机1：360°舵机自动停止【完全保留原有逻辑，一点未改】
    // =========================================================

    if(Servo1_State == SERVO1_FORWARD)
    {
        if(SysTick_Count - Servo1_Timer >= SERVO1_RUN_TIME)
        {
            Servo1_State = SERVO1_STOP_RE;
        }
    }

    if(Servo1_State == SERVO1_REVERSE)
    {
        if(SysTick_Count - Servo1_Timer >= SERVO1_RUN_TIME + 5)
        {
            Servo1_State = SERVO1_STOP_FO;
        }
    }

    // =========================================================
    // 【已删除】舵机2和3原来的PWM缓慢运动代码
    // =========================================================

    // =========================================================
    // 舵机驱动
    // =========================================================

    // 舵机1驱动【完全保留原有逻辑，一点未改】
    switch(Servo1_State)
    {
        case SERVO1_STOP_RE:
        case SERVO1_STOP_FO:
//            Servo_SetAngle1(90);
            break;

        case SERVO1_FORWARD:
            FSUS_SetServoAngle(servoUsart, 2, -180, interval, power, wait);
            break;

        case SERVO1_REVERSE:
            FSUS_SetServoAngle(servoUsart, 2, 180, interval, power, wait);
            break;

        default:
//            Servo_SetAngle1(90);
            break;
    }

    // 【已删除】舵机2和3原来的PWM输出代码

    // =========================================================
    // OLED显示【已更新为串行舵机角度】
    // =========================================================

    OLED_ShowString(2, 2, "S1:");

    if(Servo1_State == SERVO1_STOP_RE || Servo1_State == SERVO1_STOP_FO)
    {
        OLED_ShowString(2, 5, "STOP");
    }
    else if(Servo1_State == SERVO1_FORWARD)
    {
        OLED_ShowString(2, 5, "CW  ");
    }
    else if(Servo1_State == SERVO1_REVERSE)
    {
        OLED_ShowString(2, 5, "CCW ");
    }

    OLED_ShowString(2, 10, "S2:");
    // 显示舵机2当前角度
    if(servo2_angle_state == 0)
        OLED_ShowString(2, 13, "-50 ");
    else
        OLED_ShowString(2, 13, "130 ");

    OLED_ShowString(3, 2, "S3:");
    // 显示舵机3当前角度
    if(servo3_angle_state == 0)
        OLED_ShowString(3, 5, "-45 ");
    else
        OLED_ShowString(3, 5, "135 ");

    // ================= 光标 =================

    OLED_ShowString(2, 1, " ");
    OLED_ShowString(2, 9, " ");
    OLED_ShowString(3, 1, " ");

    if (Selected_Servo == 1)
    {
        OLED_ShowString(2, 1, ">");
    }

    if (Selected_Servo == 2)
    {
        OLED_ShowString(2, 9, ">");
    }

    if (Selected_Servo == 3)
    {
        OLED_ShowString(3, 1, ">");
    }
}
// ===================== 模式2 ===============================================================
void Mode2_Run(void)
{
    if(mode_first_enter)
    {
        OLED_Clear();
        mode_first_enter = 0;
        OLED_ShowString(1, 1, "Mode3");
    }

    while(1)
    {
        // 自动步数测试（单次）
	    move_smooth_auto(-50.0f,109.0f+12.0f,83.0f+106.0f+50.0f,
                  		  50.0f,109.0f+12.0f,83.0f+106.0f+50.0f, 200);//3个目标角度+3个当前角度+速度
		
        move_smooth_auto( 50.0f,109.0f+12.0f,83.0f+106.0f+50.0f,
                  		  50.0f,109.0f+12.0f,83.0f+106.0f-50.0f, 200);//3个目标角度+3个当前角度+速度
		
		move_smooth_auto( 50.0f,109.0f+12.0f,83.0f+106.0f-50.0f,
                  		 -50.0f,109.0f+12.0f,83.0f+106.0f-50.0f, 200);//3个目标角度+3个当前角度+速度
		
		move_smooth_auto(-50.0f,109.0f+12.0f,83.0f+106.0f-50.0f,
                  		 -50.0f,109.0f+12.0f,83.0f+106.0f+50.0f, 200);//3个目标角度+3个当前角度+速度
        

//        // 手动步数测试（单次，同距离对比）
//        move_smooth_manual(0,200,106, 100,150,106, 100, 250);//3个目标角度+3个当前角度+步数+速度
		
		// 按键4退出
        if(Key_Value == 4)
        {
            sys_state = 1;
            mode_first_enter = 0;
            OLED_Clear();
            Key_Value = 0;
            return;
        }
        Delay_ms(10);
    }
}






// ===================== 模式3：在第7、8行显示小球相对相机坐标 =====================
void Mode3_Run(void)
{
	
	
	extern volatile float ball_cam_x;
	
	
	OLED_ShowNum(2, 1, OpenMV_Data1, 5);
	OLED_ShowNum(3, 1, OpenMV_Data2, 5);
	OLED_ShowNum(4, 1, OpenMV_Data3, 5);	
    if(mode_first_enter)
    {
        OLED_Clear();
        mode_first_enter = 0;
    }
    
    while(1)
    {				
		//前三列显示原始相机坐标
		OLED_ShowNum(2, 1, OpenMV_Data1, 3);
		OLED_ShowNum(3, 1, OpenMV_Data2, 3);
		OLED_ShowNum(4, 1, OpenMV_Data3, 3);	
		
		uint8_t flag_1 = 0;

        flag_1 = GetBallRelativeCamera(OpenMV_Data1, OpenMV_Data2, OpenMV_Data3);
		
		
		
		if(flag_1==1)
		{
			
		}
        
		

        // 第7行第1列开始
        OLED_ShowString(2, 7, " X:");
//		OLED_ShowSignedNum
        OLED_ShowSignedNum(2, 10,(int)ball_cam_x, 4);
        OLED_ShowString(3, 7, " Y:");
        OLED_ShowNum(3, 10, (uint32_t)fabsf(ball_cam_y), 4);
		OLED_ShowString(4, 7, " Z:");
        OLED_ShowNum(4, 10, (uint32_t)fabsf(ball_cam_z), 4);
        
        if(Key_Value == 4)
        {
            sys_state = 1;
            mode_first_enter = 0;
            OLED_Clear();
            Key_Value = 0;
            return;
        }
//        Delay_ms(100);
    }
}


// ===================== 模式4 ===============================================================
void Mode4_Run(void)
{
    float theta0, theta1, theta2;
    FSUS_STATUS status0, status1, status2;
    // 定义字符串缓冲区，存储转换后的浮点数（预留足够长度：符号+整数+小数点+5位小数=最多9字符）
    char buf0[10], buf1[10], buf2[10];
    
    // 机械臂移动到初始位置
//    arm_go_to_point(50.0f, 109.0f+12.0f-30.0f, 83.0f+106.0f-50.0f);
	arm_go_to_point(50, 109+12, 83.0f+106.0f);
    
    // 首次进入清屏并显示静态标签
    if(mode_first_enter)
    {
        OLED_Clear();
        mode_first_enter = 0;
        OLED_ShowString(1, 1, "Mode4");
        OLED_ShowString(2, 1, "ID2:"); // 底座
        OLED_ShowString(3, 1, "ID0:"); // 大臂 
        OLED_ShowString(4, 1, "ID1:"); // 小臂 
    }
    
    while(1)
    {
        // 读取三个舵机角度
        status0 = FSUS_QueryServoAngle(&usart2, 2, &theta0);     // 底座ID2 
        status1 = FSUS_QueryServoAngle(&usart2, 0, &theta1);     // 大臂ID0 
        status2 = FSUS_QueryServoAngle(&usart2, 1, &theta2);     // 小臂ID1 
        
		
        // 1. 底座ID2显示：直接显示原始舵机角度，保留5位小数
        OLED_ShowString(2, 5, "        "); // 用8个空格清空原有内容（比原来多，防止浮点数残留）
        if(status0 == FSUS_STATUS_SUCCESS)
        {
            // %.5f：保留小数点后5位，对应float的6-7位有效数字
            sprintf(buf0, "%.5f", theta0-Angle_Compensation0);
            OLED_ShowString(2, 5, buf0);
        }
        else
        {
            OLED_ShowString(2, 5, "ERR");
        }
        
        // 2. 大臂ID0显示：显示数学计算角度（舵机角度+42.5°），保留5位小数
        OLED_ShowString(3, 5, "        ");
        if(status1 == FSUS_STATUS_SUCCESS)
        {
            sprintf(buf1, "%.5f", theta1-Angle_Compensation1);
            OLED_ShowString(3, 5, buf1);
        }
        else
        {
            OLED_ShowString(3, 5, "ERR");
        }
        
        // 3. 小臂ID1显示：显示数学计算角度（舵机角度+86.5°），保留5位小数
        OLED_ShowString(4, 5, "        ");
        if(status2 == FSUS_STATUS_SUCCESS)
        {
            sprintf(buf2, "%.5f", theta2-Angle_Compensation2);
            OLED_ShowString(4, 5, buf2);
        }
        else
        {
            OLED_ShowString(4, 5, "ERR");
        }
        // -----------------------------------------------------------------
        
        // 按键4退出
        if(Key_Value == 4)
        {
            sys_state = 1;
            mode_first_enter = 0;
            OLED_Clear();
            Key_Value = 0;
            Servo_SetAngle1(90);
            return;
        }
        
        Delay_ms(100);
    }
}




	
// ===================== 模式5 ===============================================================
#include "ZUOBIAOXI.h" 

void Mode5_Run(void)
{
	//实时状态量的初始值用初始位置
	float curAngle0 = 0.0;
	float curAngle1 = 90.0;
	float curAngle2 = 90.0;
	
	//程序进行标志位
	uint8_t flag_1 = 0;
	extern uint8_t flag_2;

	

	flag_1 = GetBallRelativeCamera(OpenMV_Data1, OpenMV_Data2, OpenMV_Data3);
	
	
	
	if(flag_1==0)
	{
		
		
		OLED_ShowSignedNum(2, 1, (int)ball_cam_x, 5);
		OLED_ShowSignedNum(3, 1, (int)ball_cam_y, 5);
		OLED_ShowSignedNum(4, 1, (int)ball_cam_z, 5);
		
		
		//定义结构体存入 目标点在相机坐标系下坐标
		Point3D Hand ;
		Hand.x = ball_cam_x;
		Hand.y = ball_cam_y;
		Hand.z = ball_cam_z;
		
		//定义结构体接收 目标点在底座坐标系下坐标
		Point3D  Base;
		
		// 读取3个舵机角度（curAngle0.1.2分别对应底座，大臂，小臂舵机，IP为2 0 1 ）
		FSUS_QueryServoAngle(&usart2, 2, &curAngle0); //底座
		FSUS_QueryServoAngle(&usart2, 0, &curAngle1); //大臂
		FSUS_QueryServoAngle(&usart2, 1, &curAngle2); //小臂

		//将舵机读出角度映射后再进行计算
		float curAngle0_calculate = curAngle0-Angle_Compensation0;
		float curAngle1_calculate = curAngle1-Angle_Compensation1;
		float curAngle2_calculate = curAngle2-Angle_Compensation2;
		
		hand_to_base(curAngle0_calculate,curAngle1_calculate,curAngle2_calculate,&Hand, &Base);
		
		OLED_ShowSignedNum(2, 8, (int)Base.x, 5);
		OLED_ShowSignedNum(3, 8, (int)Base.y, 5);
		OLED_ShowSignedNum(4, 8, (int)Base.z, 5);
		

		
		float theta00, theta01, theta02;
		
		flag_2=inverse_kinematics(Base.x, Base.y, Base.z, &theta00, &theta01, &theta02);
		
		
//		arm_go_to_point(Base.x, Base.y, Base.z);

//		arm_go_to_point(20, 258, 171);
		
//		Delay_ms(1000);
			
	}
	else
	{

	}
		OLED_ShowNum(2, 15, flag_1, 1);
		OLED_ShowNum(3, 15, flag_2, 1);
}




	
	
	


	



	
	
	

	


	



