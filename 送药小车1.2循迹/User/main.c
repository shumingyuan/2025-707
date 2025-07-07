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
//_________串口接收解析函数
void dataricive(void)
{
	if (Serial_RxFlag == 1) 
		{
            char* x = strtok(Serial_RxPacket, ",");
            char* y = strtok(NULL, ",");
           
            
              if(x && y)
							{
                // 解析数据
                 cx = atof(x);
                 cy = atof(y);
                
                
                // 显示数据
               OLED_ShowSignedNum(1,5,cx,6); 
               OLED_ShowSignedNum(2,5,cy,6); 
               OLED_ShowChar(4,1,'s');
                
               }
               
              Serial_RxFlag = 0;
    }
}
	
int main(void)
{
	OLED_Init();
	Serial_Init();
	MPU6050_Init();
	MPU6050_DMP_Init();
	Motor_Init();
  OLED_ShowString(1, 1, "x");
  OLED_ShowString(2, 1, "y");
	while (1)
	{   dataricive();
//		MPU6050_DMP_Get_Data(&Pitch,&Roll,&Yaw);				//读取姿态信息(其中偏航角有飘移是正常现象)
//		pid_motor_inplace(0,500);
//		MPU_Get_Gyroscope(&gx,&gy,&gz);
//		MPU_Get_Accelerometer(&ax,&ay,&az);
      pid_motor_forward(cx,150);
//    
//		OLED_ShowSignedNum(2, 1, Pitch, 5);
//		OLED_ShowSignedNum(3, 1, Roll, 5);
//  		OLED_ShowSignedNum(4, 1, Yaw, 5);
//		OLED_ShowSignedNum(2, 8, gx, 5);
//		OLED_ShowSignedNum(3, 8, gy, 5);
//		OLED_ShowSignedNum(4, 8, gz, 5);
	}
}
