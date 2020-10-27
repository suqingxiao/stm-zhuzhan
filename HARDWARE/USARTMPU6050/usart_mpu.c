#include "usart_mpu.h"
#include "stm32f4xx.h"
#include "stdio.h"
//串口2初始化

//extern void CopeSerial2Data(unsigned char ucData);

void usart_mpu_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART2);
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART2
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART1
	
	//USART2   GPIOA.2 TX 3 RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3; //PA.2,3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART2, ENABLE);                    //使能串口2


}

void USART2_IRQHandler(void)
{
 if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
//		CopeSerial2Data((unsigned char)USART2->DR);//处理数据
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }	
}
