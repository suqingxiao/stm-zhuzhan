#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart_mpu.h"
#include "led.h"
#include "key.h"
#include "mpu6050.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h" 
//#include "rcc.h"
//#include "nvic.h"
#include "CANOpen_timeBase.h"
#include "CANOpen_can.h"
#include "canfestival.h"
#include "motor.h"
#include "bsp_can.h"//配置can引脚
#include "can.h"    //can数据结构体
//#include "exti.h"
/*
405 
---CAN2                PB5 PB6
---Bluetooth  USART2
*/

extern CO_Data masterobject_Data;
extern Message Rx_Message;
extern unsigned char leftElmoInitFlag ;  //left elmo initial state     0:fault   1:success
extern unsigned char rightElmoInitFlag ; //right elmo initial state    0:fault   1:success
//extern unsigned char Sdo_Receive_State;
//extern INTEGER32 Target_position;

//unsigned char Switch_Flag=0; //电机开关状态 0：关    1：开
unsigned char State_flag=0;  //电机运行状态 0：停止  1：运行
//unsigned char key=0;           //保存键值
unsigned char Target_Torque=0x64;
extern float angle_x,angle_y,angle_z;
extern float angle_x2,angle_y2,angle_z2;
void Canopen_Init(void);
void Get_DownLeg_Imu(unsigned char sdo_id);
void Get_UpLeg_Imu(unsigned char sdo_id);
void Send_u32ToChar1(int ax,int ay,int az);
void Send_u32ToChar2(int ax,int ay,int az);

int main(void)
{
	unsigned char STM32_L=0x01;//sdo_send中一个参数，对象字典偏移量
//	unsigned char Elmo_L=0x03;
//	unsigned char STM32_R=0x02;
//	unsigned char Elmo_R=0x00;
	
	unsigned char elmo_left_nodeid = 0x03;//节点id 主站重启从站用的
//	unsigned char elmo_right_nodeid = 0x7f;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	Delay_Init(168);  //初始化延时函数
	Uart_Init(115200);//初始化串口波特率为115200
	usart_mpu_init(115200); //初始化串口2波特率为115200
//	Led_Init();				//初始化LED 
//	Key_Init(); 			//按键初始化  
	Canopen_Init();//主站初始化
	delay_ms(5000);//等待从站先启动进入状态
	while(leftElmoInitFlag == 0)
	{	
			restartElmo(elmo_left_nodeid);
	}
	leftElmoInitFlag = 0;
  Motor_Init_Left();
//	Motor_Init_Right();

	while(1)
	{
//		if(State_flag==0)
//		{		
//				Target_Torque=0x64;//2A
//			  delay_ms(500);
//			  Motor_Torque_Set(Elmo_L,Target_Torque);//set torque
//			  State_flag=1;
//			  delay_ms(500);
//	  }
		Get_DownLeg_Imu(STM32_L);
		delay_ms(50);
		Get_UpLeg_Imu(STM32_L);	
		Send_u32ToChar1((int)(angle_x * 10),(int)(angle_y * 10),(int)(angle_z * 10));//发给上位机的
		Send_u32ToChar2((int)(angle_x2 * 10),(int)(angle_y2 * 10),(int)(angle_z2 * 10));
		delay_ms(50);
	}
} 	


/******canopen初始化**********/
void Canopen_Init()
{
	Canopen_Timebase_Config();//canopen时基配置
	Canopen_Can_Config();	    //can总线配置 波特率500kbps	CAN1 PB8--RX PB9--TX			
	setNodeId(&masterobject_Data, 0x00);         //设置主站nodeID 其实没意义
  setState(&masterobject_Data, Initialisation);//CANopen进入pre-op状态 	
//	printf("CANOPEN OK");
}

/*****get down leg IMU1 value*******/
//得到小腿IMU1
void Get_DownLeg_Imu(unsigned char sdo_id)
{
	  unsigned char i=1;
		for(i=1;i<4;)
		{
			Get_IMU1_Angle(sdo_id,i);//Send SDO-->Read IMU Angle | 1->angle_x | 2->angle_y | 3->angle_z |
			i++;
			delay_ms(80);
		}
}
/*****get up leg IMU2 value*******/
//得到大腿IMU2
void Get_UpLeg_Imu(unsigned char sdo_id)
{
	  unsigned char i=1;
		for(i=1;i<4;)
		{
			Get_IMU2_Angle(sdo_id,i);//Send SDO-->Read IMU Angle | 1->angle_x | 2->angle_y | 3->angle_z |
			i++;
			delay_ms(80);
		}
}
/**************发送单个字节数据*******************/
void Send_char(char DataToSend)
{
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);  //等待上次传输完成 
	USART_SendData(USART2,DataToSend); 	 //发送数据到串口1 
}
/**************将int以16进制发送数据IMU1*******************/
//50-->AA 00 00 00 32
void Send_u32ToChar1(int ax,int ay,int az)
{
    char ByteSend[14]={0};//发送的字节
		char i =0;
		ByteSend[0] = 0xAA;//帧头，ByteSend的类型为uchar或char均可
		ByteSend[1] = 0xA1;//imu1帧头		
		ByteSend[2] = (ax>>24) & 0xFF;//
		ByteSend[3] = (ax>>16) & 0xFF;
		ByteSend[4] = (ax>>8)  & 0xFF;
		ByteSend[5] = ax & 0xFF;	
		ByteSend[6] = (ay>>24) & 0xFF;//
		ByteSend[7] = (ay>>16) & 0xFF;
		ByteSend[8] = (ay>>8)  & 0xFF;
		ByteSend[9] = ay & 0xFF;	
		ByteSend[10] = (az>>24) & 0xFF;//
		ByteSend[11] = (az>>16) & 0xFF;
		ByteSend[12] = (az>>8)  & 0xFF;
		ByteSend[13] = az & 0xFF;
		for(i=0;i<14;i++)
		{
			Send_char(ByteSend[i]);
		}		
}
/**************将int以16进制发送数据IMU2*******************/
//50-->AA 00 00 00 32
void Send_u32ToChar2(int ax,int ay,int az)
{
    char ByteSend[14]={0};//发送的字节
		char i =0;
		ByteSend[0] = 0xAA;//帧头，ByteSend的类型为uchar或char均可
		ByteSend[1] = 0xA2;//imu1帧头		
		ByteSend[2] = (ax>>24) & 0xFF;//
		ByteSend[3] = (ax>>16) & 0xFF;
		ByteSend[4] = (ax>>8)  & 0xFF;
		ByteSend[5] = ax & 0xFF;	
		ByteSend[6] = (ay>>24) & 0xFF;//
		ByteSend[7] = (ay>>16) & 0xFF;
		ByteSend[8] = (ay>>8)  & 0xFF;
		ByteSend[9] = ay & 0xFF;	
		ByteSend[10] = (az>>24) & 0xFF;//
		ByteSend[11] = (az>>16) & 0xFF;
		ByteSend[12] = (az>>8)  & 0xFF;
		ByteSend[13] = az & 0xFF;
		for(i=0;i<14;i++)
		{
			Send_char(ByteSend[i]);
		}		
}
 


