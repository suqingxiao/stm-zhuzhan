 #include "timer3.h"
#include "canfestival.h"
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//extern CO_Data serverobject_Data;
//extern INTEGER32 Target_position;

const int length =4;
int ap_angle=10;//角度放大倍数
float pitch,roll,yaw; 		//欧拉角
int amplify_pitch[3],amplify_roll[3],amplify_yaw[3];
int count=0;


//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
//		LED1=!LED1;//DS1翻转
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			switch(count)
			{
				case 0:				
					amplify_roll[0]=(int)(roll*ap_angle);
					amplify_pitch[0]=(int)(pitch*ap_angle);
					amplify_yaw[0]=(int)(yaw*ap_angle);
				break;
				case 1:				
					amplify_roll[1]=(int)(roll*ap_angle);
					amplify_pitch[1]=(int)(pitch*ap_angle);
					amplify_yaw[1]=(int)(yaw*ap_angle);
				break;		
				case 2:				
					amplify_roll[2]=(int)(roll*ap_angle);
					amplify_pitch[2]=(int)(pitch*ap_angle);
					amplify_yaw[2]=(int)(yaw*ap_angle);
				break;				
			}
			count++;
			if(count==3)
				count=0;
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
