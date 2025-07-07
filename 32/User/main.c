#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "Motor.h"
#include "Serial.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
float Pitch,Roll,Yaw;								//俯仰角默认跟中值一样，翻滚角，偏航角
int16_t ax,ay,az,gx,gy,gz;							//加速度，陀螺仪角速度
int cy=0,cx=0;
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
//----------------------pid角度环控制

float kp=0.8,ki=0.01,kd=0.1;
float erro=0,erro_last=0,I=0;
float pid_value=0;
void pid_motor_forward(int angle,int speed)
{
	erro=0-angle;
	I+=erro;
	if(I>1000)
	{
		I=1000;
	}
	if(I<-1000)
	{
		I=-1000;
		
	}
	pid_value=kp*erro+ki*I+kd*(erro-erro_last);
	if(pid_value>speed)
	{
		pid_value=speed;
	}
	if(pid_value<-speed)
	{
		pid_value=-speed;
	}
  MotorA_SetSpeed((int)(speed-pid_value));
	MotorB_SetSpeed((int)(speed+pid_value));
	erro_last=erro;
}// 左边角度为正
void pid_motor_inplace(int angle,int speed)
{
	erro=angle-Yaw;//陀螺仪pid时，angle-YAW
	I+=erro;
	if(I>1000)
	{
		I=1000;
	}
	if(I<-1000)
	{
		I=-1000;
		
	}
	pid_value=kp*erro+ki*I+kd*(erro-erro_last);
	if(pid_value>speed)
	{
		pid_value=speed;
	}
	if(pid_value<-speed)
	{
		pid_value=-speed;
	}
  MotorA_SetSpeed(-(int)pid_value);
	MotorB_SetSpeed(+(int)pid_value);
	erro_last=erro;
}

/**
  * @brief  控制小车原地转向指定角度
  * @param  angle 目标角度，负数向左，正数向右。如 -90.0f, 90.0f, 180.0f
  * @param  speed 转向速度（0-1000）
  * @retval 无
  */
void Turn(float angle, uint16_t speed)      //左转，右转，掉头 （已测试，没问题)
{
    float target_yaw;
    float current_yaw;
    
    // 1. 获取当前偏航角并计算目标角度
    MPU6050_DMP_Get_Data(&Pitch, &Roll, &current_yaw);
    target_yaw = current_yaw + angle;
    
    // 2. 角度规范化到 -180 ~ 180 度
    if (target_yaw > 180.0f)
    {
        target_yaw -= 360.0f;
    }
    else if (target_yaw < -180.0f)
    {
        target_yaw += 360.0f;
    }
    
    // 3. 循环控制，直到达到目标角度
    // 允许一定的误差范围，比如±1度
    while (1)
    {
        MPU6050_DMP_Get_Data(&Pitch, &Roll, &current_yaw);
        
        // 判断是否到达目标
        if (fabs(current_yaw - target_yaw) < 1.0f) 
        {
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            break;
        }
        
        // 使用PID进行原地转向
        pid_motor_inplace((int)target_yaw, speed);
    }
    
    // 4. 转向完成，停止电机
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
}

//_________统一串口解析函数_________
// 返回值: 0=启动, 1=左转, 2=右转, 3=循迹数据, -1=无有效数据
int Parse_Serial_Data(void)
{
	int result = -1; // 默认为无效数据
	if (Serial_RxFlag == 1)
	{
		char* p = Serial_RxPacket;
		switch(p[0])
		{
			case '0':
				result = 0; // 启动指令
				break;
			case '1':
				result = 1; // 左转指令
				break;
			case '2':
				result = 2; // 右转指令
				break;
			case '@':
				{
					// 解析循迹数据: "@rho_err,theta_err,mag"
					char* rho_str = strtok(p + 1, ",");
					char* theta_str = strtok(NULL, ",");
					if (rho_str && theta_str)
					{
						// OpenMV发来的theta_err就是我们需要的偏差值
						// 将其赋值给全局变量cx，以复用pid_motor_forward函数
						cx = (int)atof(theta_str); 
						result = 3; // 循迹数据
					}
				}
				break;
		}
		Serial_RxFlag = 0; // 清除标志位
	}
	return result;
}

int main(void)
{
	OLED_Init();
	Serial_Init();
	Motor_Init();
	MPU6050_Init();
	MPU6050_DMP_Init();

	OLED_ShowString(1, 1, "System Ready");
	OLED_ShowString(2, 1, "Wait for Start(0)");

	// 1. 等待启动指令 '0'
	while(Parse_Serial_Data() != 0)
	{
		Delay_ms(100);
	}

	OLED_Clear();
	OLED_ShowString(1, 1, "Start Tracking...");
	
	// 2. 进入主循环：循迹 + 等待转向指令
	while (1)
	{
		int cmd = Parse_Serial_Data();
		
		if (cmd == 3) // 收到循迹数据
		{
			// 调用您原来的PID循迹函数，基础速度150
			pid_motor_forward(cx, 150); 
			OLED_ShowString(1, 1, "Tracking...");
			OLED_ShowString(2, 1, "CX:");
			OLED_ShowSignedNum(2, 4, cx, 5);
		}
		else if (cmd == 1) // 收到左转指令
		{
			OLED_Clear();
			OLED_ShowString(1, 1, "CMD: 1, Turn Left");
			Turn(-90.0f, 400);
			OLED_Clear();
			OLED_ShowString(1, 1, "Continue Tracking...");
		}
		else if (cmd == 2) // 收到右转指令
		{
			OLED_Clear();
			OLED_ShowString(1, 1, "CMD: 2, Turn Right");
			Turn(90.0f, 400);
			OLED_Clear();
			OLED_ShowString(1, 1, "Continue Tracking...");
		}
		// 如果没有收到任何有效指令(cmd == -1)，小车会保持上一个速度状态
		// 如果希望没收到数据就停止，可以在这里加 MotorA_SetSpeed(0); MotorB_SetSpeed(0);
	}
}
