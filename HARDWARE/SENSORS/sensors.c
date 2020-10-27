#include "sensors.h"
//#include "CANOpen_timeBase.h"
//#include "CANOpen_can.h"
#include "canfestival.h"
#include "delay.h"


extern CO_Data masterobject_Data;
u8 CliSdoNbr_Stm32_L=1;
/*************************命令配置*********************************************/
u8 Read_Imu[8]=     {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00};//等从站配好再配
u8 Read_Encoder[8]= {0x40,0x41,0x60,0x00,0x00,0x00,0x00,0x00};

//Function：获取Imu数据
//param：none
void IMU_Get()
{
		sendSDO(&masterobject_Data, SDO_CLIENT, CliSdoNbr_Stm32_L, Read_Imu);
		delay_ms(20);
}
//Function：获取编码器值
//param：none
void Encoder_Get()
{
		sendSDO(&masterobject_Data, SDO_CLIENT, CliSdoNbr_Stm32_L, Read_Imu);
		delay_ms(20);
}






