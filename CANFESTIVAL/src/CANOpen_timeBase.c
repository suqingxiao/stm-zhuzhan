/**
	***********************************
	* 文件名:	CANOpen_timeBase.c
	* 作者:		stone
	* 版本:		V0.1
	* 日期:		2018-3-29
	* 描述:		CANOPEN底层定时器接口文件
	***********************************
	*/
#include "stm32f4xx_it.h"
#include "canfestival.h"
#include "CANOpen_timeBase.h"
 
/* 功能:	定时器配置
	 参数:	无
	 返回值:无
 */
static void timer_config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	/* 允许TIM2的时钟 */
	//RCC_APB1启用或禁用低速APB (APB1)外围时钟 36Mhz 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	 
	/* 将定时器2寄存器设为初始值 */
	TIM_DeInit(TIM2);
	
	/* 设置定时器2由内部时钟 */
	TIM_InternalClockConfig(TIM2);
	
	/* 预分频值 */
//	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;//72MHZ
		TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;//84MHZ
	/* 时钟分割 */
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	/* 向上计数 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/* 自动重载值 */
	TIM_TimeBaseStructure.TIM_Period = 50000 - 1;
	/* 初始化定时器2 */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* 清除溢出中断标志 */ 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	
	/* 禁止ARR预装载缓冲器 */
	TIM_ARRPreloadConfig(TIM2, DISABLE);
	
	/* 开启TIM2的中断 */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	/* 开启定时器2 */
	TIM_Cmd(TIM2, ENABLE);
}
 
/* 功能:	TIM中断配置
	 参数:	无
	 返回值:无
 */
static void TIM_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 选择TIM2的中断通道 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	/* 抢占式中断优先级设置为2 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	/* 响应式中断优先级设置为1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/* 使能中断 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 中断初始化 */
	NVIC_Init(&NVIC_InitStructure);	
}
 
/* 功能:	canopen时基配置
	 参数:	无
	 返回值:无
 */
void Canopen_Timebase_Config(void)
{
	/* 定时器配置 */
	timer_config();
	/* 中断配置 */
	TIM_nvic_config();
}
 
/* 功能：	设置定时器触发时间
	 参数: 	定时器触发时间
	 返回值:无 
 */
void setTimer(TIMEVAL value)
{
	TIM2->ARR = TIM2->CNT + value;
}
 
/* 功能：	获取上次触发到现在的流逝时间
	 参数: 	无
	 返回值:获取计数器值
 */
TIMEVAL getElapsedTime(void)
{
	return TIM2->CNT;
}
 
/* 定时器2中断向量 */
void TIM2_IRQHandler(void)
{
	/* 定时器2溢出标志位 */
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		/* 定时器2清除溢出标志位 */
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
 
		TimeDispatch();
	}
}

