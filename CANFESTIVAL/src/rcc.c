/**
	***********************************
	* 文件名:	rcc.c
	* 作者:		stone
	* 版本:		V0.1
	* 日期:		2018-3-29
	* 描述:		配置时钟源
	***********************************
	*/
#include "stm32f4xx_it.h"
#include "stm32f4xx_flash.h"
#include "rcc.h"
 
/* 功能:	RCC时钟配置
	 参数:	无
   返回值:无
 */
void rcc_config(void)
{ 
	ErrorStatus HSEStartUpStatus;
 
	/* RCC寄存器设置为默认配置 */
	RCC_DeInit();
	
	/* 打开外部高速时钟 */
	RCC_HSEConfig(RCC_HSE_ON);
	
	/* 等待外部高速时钟稳定 */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	
	if(HSEStartUpStatus == SUCCESS) 
	{ 
		/* 设置HCLK = SYSCLK 配置AHB时钟(HCLK) RCC_SYSCLK_Div1: AHB clock = SYSCLK72*/
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* 设置PCLK2 = HCLK 配置高速APB2时钟72(PCLK2) UART1、SPI1、Timer1、ADC1I/O（AFIO）口等。*/
		/*本身例程设置的是 SYSCLK=168MHZ,HCLK=SYSCLK/1=168MHZ, PCLK1=HCLK/4=42MHZ, PCLK2=HCLK/2=82MHZ */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* 设置PCLK1 = HCLK / 2 配置低速APB1时钟(PCLK1)36 时钟输出供定时器2-7使用CAN、USB、I2C1、I2C2、USART2、USART3、UART4、UART5、SPI2*/
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		/* 设置FLASH代码延时 */
		FLASH_SetLatency(FLASH_Latency_2);
		/* 使能预取址缓存 */
		FLASH_PrefetchBufferCmd(ENABLE);;
		
		/* 设置PLL时钟源为HSE倍频9 72MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE, 8, 288, 4, 8);//原本是144 2 4 我修改到了288 4 8
		/* 使能PLL */
		RCC_PLLCmd(ENABLE);
		/* 等待PLL稳定 */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		
		/* 设置PLL为系统时钟源 */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		/* 等待系统时钟源切换到PLL */
		while(RCC_GetSYSCLKSource() != 0x08);
 
		/* 设置系统节拍器时钟源为FCLK */
		SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	}
}

