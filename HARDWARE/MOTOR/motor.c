#include "motor.h"
#include "sys.h"
//#include "CANOpen_timeBase.h"
#include "CANOpen_can.h"
#include "canfestival.h"
#include "delay.h"


extern CO_Data masterobject_Data;
extern unsigned char leftElmoInitFlag ;  //left elmo initial state     0:fault   1:success
extern unsigned char rightElmoInitFlag ; //right elmo initial state    0:fault   1:success
extern unsigned char leftElmo6040Flag ;//left elmo enable voltage or operation   0:disable 1:enable
extern unsigned char rightElmo6040Flag;//right elmo enable voltage or operation  0:disable 1:enable
extern unsigned char leftElmoTqrFlag;    //left elmo enable operation  0:disable 1:enable
extern unsigned char rightElmoTqrFlag;   //right elmo enable operation 0:disable 1:enable
extern unsigned char leftElmoModeFlag;   //left elmo enable mode       0:disable 1:enable 
extern unsigned char rightElmoModeFlag;  //right elmo enable mode      0:disable 1:enable 
extern unsigned char leftElmoMaxFlag;    //left elmo set max current   0:fault   1:success
extern unsigned char rightElmoMaxFlag;   //right elmo set max current  0:fault   1:success
unsigned char leftElmoRestartFlag  = 0;   //left elmo Restart   0:can not change initial state flag  1:can 
unsigned char rightElmoRestartFlag = 0;  //right elmo Restart   0:can not change initial state flag  1:can 
/*************************命令配置*********************************************/
unsigned char halit_datasend[8]=        {0x2b,0x40,0x60,0x00,0x80,0x00,0x00,0x00};//	

/*****启动电机进入操作状态*******************/
unsigned char vol_enable_datasend[8]=   {0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00};//开电压   //开启点电机1
unsigned char switch_on_datasend[8]=    {0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00};//打开开关
unsigned char operation_en_datasend[8]= {0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00};//启动看06 0f//开启电机2

/*****电机停止 重新启动只用0x06 0x0f就行*****/
//unsigned char stop_datasend[8]=         {0x2b,0x40,0x60,0x00,0x03,0x00,0x00,0x00};	
unsigned char stop_datasend[8]=         {0x2b,0x40,0x60,0x00,0x0b,0x00,0x00,0x00};	

/*****模式切换*****************/
unsigned char velocity_mode_datasend[8]= {0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00};	
unsigned char current_mode_datasend[8]=  {0x2f,0x60,0x60,0x00,0x04,0x00,0x00,0x00};//	

/*****速度 位置模式下要用 力矩好像不用再发了可能时整定时配好了****/
unsigned char acc_datasend[8]=          {0x23,0x83,0x60,0x00,0x50,0xc3,0x00,0x00};	
unsigned char dec_datasend[8]=          {0x23,0x84,0x60,0x00,0x50,0xc3,0x00,0x00};	
unsigned char st_dec_datasend[8]=       {0x23,0x85,0x60,0x00,0x50,0xc3,0x00,0x00};	
unsigned char target_speed_datasend[8]= {0x23,0xff,0x60,0x00,0x10,0x27,0x00,0x00};	

/*****力矩模式的命令*******************/
unsigned char read_rated_current_datasend[8] =   {0x40,0x75,0x60,0x00,0x00,0x00,0x00,0x00};//查询电机额定电流
unsigned char max_current_datasend[8] =   			 {0x2b,0x73,0x60,0x00,0xd0,0x07,0x00,0x00};	
unsigned char target_current_datasend[8] =  		 {0x2b,0x71,0x60,0x00,0x00,0x00,0x00,0x00};//可以先清空电流参数再去使能

/*********get imu1 angle order*******/
unsigned char get_anglex_datasend[8]={0x40,0x04,0x20,0x00,0x00,0x00,0x00,0x00};
unsigned char get_angley_datasend[8]={0x40,0x08,0x20,0x00,0x00,0x00,0x00,0x00};//这个应该时你要用的俯仰角
unsigned char get_anglez_datasend[8]={0x40,0x0B,0x20,0x00,0x00,0x00,0x00,0x00};

/*********get imu2 angle order*******/
unsigned char get_anglex2_datasend[8]={0x40,0x11,0x20,0x00,0x00,0x00,0x00,0x00};
unsigned char get_angley2_datasend[8]={0x40,0x15,0x20,0x00,0x00,0x00,0x00,0x00};//这个应该时你要用的俯仰角
unsigned char get_anglez2_datasend[8]={0x40,0x19,0x20,0x00,0x00,0x00,0x00,0x00};

/******	重启Elmo节点**************/
void restartElmo(unsigned char node_id)
{
	switch(node_id)
	{
		case 0x03:
			leftElmoRestartFlag = 1;break;
		case 0x7f:
			rightElmoRestartFlag = 1;break;
		default:
			break;
	}

	while(leftElmoInitFlag==0)
	{
		masterSendNMTstateChange(&masterobject_Data, node_id, 0x81);
		delay_ms(500);
	}
}

/***************电机初始化*****/
void Motor_Init_Left()//左腿elmoId=0x03,右腿0x7f
{
	while(leftElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 3, halit_datasend);	//deal CAN state machine fault 
		delay_ms(500);
	}
	
	leftElmo6040Flag = 0;
	while(leftElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 3, vol_enable_datasend);	//使能电压
		delay_ms(500);
	}
	
	leftElmo6040Flag = 0;
	while(leftElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 3, switch_on_datasend);	 //重置标志位，使能开关
		delay_ms(500);
	}
	
	leftElmo6040Flag = 0;
	while(leftElmoModeFlag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 3, current_mode_datasend);//设置力矩模式
		delay_ms(500);
	}
	leftElmoModeFlag = 0;
	
	while(leftElmoMaxFlag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 3, max_current_datasend);	//设置最大电流40A
		delay_ms(500);
	}
	
	sendSDO(&masterobject_Data, SDO_CLIENT, 3, target_current_datasend);//设置目标电流值为0
	delay_ms(500);
	
	leftElmo6040Flag = 0;
	while(leftElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 3, operation_en_datasend);	//重置标志位，使能操作
		delay_ms(500);
	}
	

}
//Right elmo init
void Motor_Init_Right()//左腿elmoId=0x03,右腿0x7f
{	
	while(leftElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 3, halit_datasend);	//deal CAN state machine fault 
		delay_ms(500);
	}
	
	leftElmo6040Flag = 0;
	while(rightElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 0, vol_enable_datasend);//使能电压
		delay_ms(500);
	}
	
	rightElmo6040Flag = 0;	
	while(rightElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 0, switch_on_datasend);//重置标志位，使能开关
		delay_ms(500);
	}
	
	rightElmo6040Flag = 0;
	while(rightElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 0, operation_en_datasend);//重置标志位，使能操作
		delay_ms(500);
	}
	
	rightElmo6040Flag = 0;
	while(rightElmo6040Flag==0)
	{
	  sendSDO(&masterobject_Data, SDO_CLIENT, 0, current_mode_datasend);	//设置力矩模式
		delay_ms(500);
	}
}

//Function：电机停止
//param：none
void Motor_Stop(unsigned char sdo_id)
{
		target_current_datasend[4]=0x00;
		sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, target_current_datasend);
		delay_ms(200);
		sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, stop_datasend);
    delay_ms(200);
}

//Function：电机停止之后再启动
//param: none
void Motor_Restart(unsigned char sdo_id)
{
		sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, vol_enable_datasend);
		delay_ms(200);
		sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, operation_en_datasend);
	  delay_ms(200);
}
//Function：电机力矩设置
//param: int torque
void Motor_Torque_Set(unsigned char sdo_id, unsigned char torque)
{
	while(leftElmoTqrFlag==0)
	{
	  target_current_datasend[4]=torque;
		sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, target_current_datasend);
		delay_ms(200);
	}
	leftElmoTqrFlag = 1; //复位标志
}
//Function：获得IMU1信息
//param: angle--angle_x--1\angle_y--2\angle_z--3
void Get_IMU1_Angle(unsigned char sdo_id, char angle)
{
	switch(angle)
	{
			case 1:	
				sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, get_anglex_datasend);
			case 2:	
				sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, get_angley_datasend);
			case 3:	
				sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, get_anglez_datasend);
			default:  
				break;
	}
}
//Function：获得IMU1信息
//param: angle--angle_x--1\angle_y--2\angle_z--3
//       unsigned char sdo_id---发给哪个SDO对象
void Get_IMU2_Angle(unsigned char sdo_id, char angle)
{
	switch(angle)
	{
			case 1:	
				sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, get_anglex2_datasend);
			case 2:	
				sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, get_angley2_datasend);
			case 3:	
				sendSDO(&masterobject_Data, SDO_CLIENT, sdo_id, get_anglez2_datasend);
			default:  
				break;
	}
}




