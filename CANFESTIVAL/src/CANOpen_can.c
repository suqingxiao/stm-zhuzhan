/**
	***********************************
	* �ļ���:	CANOpen_can.c
	* ����:		stone
	* �汾:		V0.1
	* ����:		2018-3-29
	* ����:		CANOPENЭ��ײ����߽ӿ�
	***********************************
	*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "CANOpen_can.h"
#include "canfestival.h"
#include "bsp_can.h"
#include "usart.h"
/* CANOPEN�ֵ� */
extern CO_Data masterobject_Data;
unsigned char leftElmoInitFlag = 0 ;  //left elmo initial state     0:fault   1:success
unsigned char rightElmoInitFlag = 0 ; //right elmo initial state    0:fault   1:success
unsigned char leftElmo6040Flag = 0 ;//left elmo enable voltage or operation   0:disable 1:enable
unsigned char rightElmo6040Flag = 0;//right elmo enable voltage or operation  0:disable 1:enable
unsigned char leftElmoTqrFlag = 0;    //left elmo enable operation  0:disable 1:enable
unsigned char rightElmoTqrFlag = 0;   //right elmo enable operation 0:disable 1:enable
unsigned char leftElmoModeFlag = 0;   //left elmo enable mode       0:disable 1:enable 
unsigned char rightElmoModeFlag = 0;  //right elmo enable mode      0:disable 1:enable 
unsigned char leftElmoMaxFlag = 0;    //left elmo set max current   0:fault   1:success
unsigned char rightElmoMaxFlag = 0;   //right elmo set max current  0:fault   1:success
extern u8 leftElmoRestartFlag ;   //left elmo Restart   0:can not change initial state flag  1:can 
extern u8 rightElmoRestartFlag ;  //right elmo Restart   0:can not change initial state flag  1:can 
Message Rx_Message;
float angle_x=1,angle_y=1,angle_z=1;
float angle_x2=1,angle_y2=1,angle_z2=1;
float Data_Transform(UNS8 data[8]);
void Data_Analysis(UNS16 cobid,UNS8 data[8]);

//u8 nodeflag=0; //�ж��ĸ��ڵ�
/*
 * ��������gpio_config
 * ����  ��CAN��GPIO ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void gpio_config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;   	

		/* ʹ��GPIOʱ��*/
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTBʱ�� 	

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
		GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB5,PB6

		//���Ÿ���ӳ������	
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOB5����ΪCAN2
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOB6����ΪCAN2
}


/*
 * ��������nvic_config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬1��1���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void nvic_config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*�ж�����*/
//		NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;	   //CAN RX�ж�
	  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX_IRQ;	   //CAN RX�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
 
/* ����:	CAN���߹���������
	 ����:	��
	 ����ֵ:��
 */
static void can_filter_config(void)
{
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
 
	/* ���ù�����0�飬���óɱ�׼��ʶ���ҵ�7λ��Ϊ0ʱ���� */
//	CAN_FilterInitStructure.CAN_FilterNumber = 0;													/* ���ù�������0 */
	CAN_FilterInitStructure.CAN_FilterNumber = 14;													/* ���ù�������0 */
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;				/* ����ģʽ */
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;			/* 32λģʽ */
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;								/* ��CANOpen�б�׼��ʶ���ĵ�7λ��ʾ�ڵ�ID */
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0004;									/* ��CANOpen��ֻ�ñ�׼��ʶ��������֡/Զ��֡���� */
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;										/* ���ڵ�IDΪ0 */
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;											/* ��׼֡ */
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;	/* ������������FIFO0 */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;								/* ʹ�ܹ����� */
	CAN_FilterInit(&CAN_FilterInitStructure);
}
 
/* ����:	can��������
	 ����:	��
	 ����ֵ:��
 */
void Canopen_Can_Config(void)
{
	CAN_InitTypeDef CAN_InitStructure;
 
	/* ����IO */
	gpio_config();
 
	/* �ж�Ƕ�׿��������� */
	nvic_config();
 
	/* ����CAN����ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
 
	/* CAN1Ĭ�ϲ��� */
//	CAN_DeInit(CAN1);
	/* CAN2Ĭ�ϲ��� */
  CAN_DeInit(CAN2);
	/* ���ṹ������Ĭ�ϲ��� */
	CAN_StructInit(&CAN_InitStructure);

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=ENABLE;	//DISABLE�����Զ����߹���	 ENABLE �ر��Զ����߹���
	CAN_InitStructure.CAN_AWUM=ENABLE;// DISABLE˯��ģʽͨ����������(���CAN->MCR��SLEEPλ) ENABLE �ر��Զ�����
	CAN_InitStructure.CAN_NART=DISABLE;	//ENABLE ��ֹ�����Զ����� DISABLE �Զ��ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ� ��ֹFIFO���ʱ����ԭ���� 
	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
	
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ����(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=CAN_BS1_7tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=7;  //��Ƶϵ��(Fdiv)Ϊbrp+1	���ò�����:36MHz/9/(4+3+1)=500kbps 42MHz/7/(7+4+1)=500kbps
//	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1  
	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN2
 
	/* CAN���߹��������� */
	can_filter_config();	
	/* ���չ����ж� */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}
 
/* CAN1���߽����жϻص����� */
//void CAN1_RX0_IRQHandler(void)

/************************CAN2���ж���Ӧ����********************/
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg message;
//	Message Rx_Message;	
	/* ������Ϣ */
	CAN_Receive(CAN2, CAN_FIFO0, &message);
	/* ��װcanopen���ݰ� */
	Rx_Message.cob_id = message.StdId;											/* ������ͽڵ�ID */
	Rx_Message.rtr = (message.RTR == CAN_RTR_DATA) ? 0 : 1;	/* ��ʶ������ */
	Rx_Message.len = message.DLC;														/* ���ݰ����� */
	memcpy(Rx_Message.data, message.Data, message.DLC);			/* ���� */
	Data_Analysis(Rx_Message.cob_id,Rx_Message.data);       //Analysis sdo data
//canopen���ݰ����䴦������ //��վ���ǲ��� ��ʵ��վ���ô����������������
//canDispatch(&masterobject_Data, &Rx_Message);

}

/* ����:	CAN�������ݺ���
	 ����:	notused can���߶˿�
					message canopen���ݰ�
	����ֵ:	0 �ɹ�
					1 ʧ��
 */
//CAN_PORT notused,
uint8_t canSend(CAN_PORT notused, Message *message)
{
	uint32_t i = 0xFFFFFF;
	CanTxMsg TxMessage;
	uint8_t TransmitMailbox = 0;
 
	/* ��װCAN���ݰ� */
	TxMessage.DLC = message->len;																	/* ���ݳ��� */
	memcpy(TxMessage.Data, message->data, message->len);					/* ���� */
	TxMessage.IDE = CAN_ID_STD;																		/* ��׼ID#CAN_Id_Standard((uint32_t)0x00000000)*/
	TxMessage.StdId = message->cob_id;														/* ��ʶ�� */
	TxMessage.RTR = (message->rtr == CAN_RTR_DATA) ? 0 : 2;				/* ����֡ */
	
	TransmitMailbox = CAN_Transmit(CAN2, &TxMessage);		          /* �������ݰ� */
	while((CAN_TransmitStatus(CAN2, TransmitMailbox) != CANTXOK) && --i);	/* �ȴ����ͳɹ� */
	return (i != 0) ? 0 : 1;                                            	/* �ɹ�0 ��ʱ1 */
}

/****************���ݽ�������************/
void Data_Analysis(UNS16 cobid,UNS8 data[8])
{
//	u8 flag=0;
	switch (cobid)
	{
		case 0x703:
			//elmo_left reset and response boot-up
		  if(leftElmoRestartFlag == 1)
			{
				if(data[0]==0x00)
				{
					leftElmoInitFlag = 1;
				}
		  }
			break;
		case 0x77f:
			//elmo_right reset and response boot-up
		  if(data[0]==0x00)
			{
				rightElmoInitFlag = 1;
			}
			break;
		case 0x581:
//STM32_1
		  if(data[0]==0x43)
			{
				switch(data[1])
				{//angle_x
					case 0x64:
						angle_x = Data_Transform(data);break;
					case 0x69:
						angle_y = Data_Transform(data);break;
					case 0x6c:
						angle_z = Data_Transform(data);break;
					case 0x62:
						angle_x2 = Data_Transform(data);break;
					case 0x6b:
						angle_y2 = Data_Transform(data);break;
					case 0x7a:
						angle_z2 = Data_Transform(data);break;
				}
			}
			break;
		case 0x582:
//			flag=2;//STM32_2��ʱ��û��--0910
		if(data[0]==0x43)
			{
				switch(data[1])
				{//angle_x
					case 0x64:
						angle_x = Data_Transform(data);break;
					case 0x69:
						angle_y = Data_Transform(data);break;
					case 0x6c:
						angle_z = Data_Transform(data);break;
					case 0x62:
						angle_x2 = Data_Transform(data);break;
					case 0x6b:
						angle_y2 = Data_Transform(data);break;
					case 0x7a:
						angle_z2 = Data_Transform(data);break;
				}
			}
			break;
		case 0x583:
			if(data[0]==0x60)//vol | opt |torque set
			{
				switch(data[1])
				{
					case 0x40:
						leftElmo6040Flag = 1;break;//voltage or opration
					case 0x60:
						leftElmoModeFlag = 1;break;//voltage or opration
					case 0x71:
						leftElmoTqrFlag = 1;break;//set torque success
					case 0x73:
						leftElmoMaxFlag = 1;break;//set max current success
				}
			}
			break;
		case 0x5FF:
			break;
  }	
//	Sdo_Receive_State = 1;
}
/*****data_transform******/
float Data_Transform(UNS8 data[8])
{
	int resualt_i=0;
	float resualt_f=0;
	if((data[7] & 0xFF) == 0xFF)//����
	{
		resualt_i=(data[4]+((0x00ff&data[5])<<8)+((0x0000ff&data[6])<<16)+((0x000000ff&data[7])<<24));
		resualt_i=resualt_i-0xffffffff;
		resualt_f=((float)(resualt_i))/1000;
	}
	else
	{
		resualt_i=(data[4]+((0x00ff&data[5])<<8)+((0x0000ff&data[6])<<16)+((0x000000ff&data[7])<<24));
		resualt_f=((float)(resualt_i))/1000;
	}
	return resualt_f;
}
