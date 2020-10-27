#ifndef __MOTOR_H
#define __MOTOR_H	 



void restartElmo(unsigned char node_id);
void Motor_Init_Left(void);
void Motor_Init_Right(void);
void Motor_Stop(unsigned char sdo_id);
void Motor_Restart(unsigned char sdo_id);
void Motor_Torque_Set(unsigned char sdo_id, unsigned char torque);
void Get_IMU1_Angle(unsigned char sdo_id, char angle);
void Get_IMU2_Angle(unsigned char sdo_id, char angle);	
#endif
