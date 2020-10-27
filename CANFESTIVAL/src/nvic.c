/**
	***********************************
	* 文件名:	nvic.c
	* 作者:		stone
	* 版本:		V0.1
	* 日期:		2018-3-29
	* 描述:		设置中断控制器
	***********************************
	*/
#include "stm32f4xx_it.h"
#include "nvic.h"
 
/* 功能:	中断嵌套控制器配置
   参数:	无
   返回值:无
 */
void nvic_config_init(void)
{
	/* 选择中断分组4 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
