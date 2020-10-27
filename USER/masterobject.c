
/* File generated by gen_cfile.py. Should not be modified. */

#include "masterobject.h"

/**************************************************************************/
/* Declaration of mapped variables                                        */
/**************************************************************************/
INTEGER32 Target_position = 0x0;		/* Mapped at index 0x607A, subindex 0x00 */

/**************************************************************************/
/* Declaration of value range types                                       */
/**************************************************************************/

#define valueRange_EMC 0x9F /* Type for index 0x1003 subindex 0x00 (only set of value 0 is possible) */
UNS32 masterobject_valueRangeTest (UNS8 typeValue, void * value)
{
  switch (typeValue) {
    case valueRange_EMC:
      if (*(UNS8*)value != (UNS8)0) return OD_VALUE_RANGE_EXCEEDED;
      break;
  }
  return 0;
}

/**************************************************************************/
/* The node id                                                            */
/**************************************************************************/
/* node_id default value.*/
UNS8 masterobject_bDeviceNodeId = 0x00;

/**************************************************************************/
/* Array of message processing information */

const UNS8 masterobject_iam_a_slave = 0;

TIMER_HANDLE masterobject_heartBeatTimers[1];

/*
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

                               OBJECT DICTIONARY

$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
*/

/* index 0x1000 :   Device Type. */
                    UNS32 masterobject_obj1000 = 0x0;	/* 0 */
                    subindex masterobject_Index1000[] = 
                     {
                       { RO, uint32, sizeof (UNS32), (void*)&masterobject_obj1000, NULL }
                     };

/* index 0x1001 :   Error Register. */
                    UNS8 masterobject_obj1001 = 0x0;	/* 0 */
                    subindex masterobject_Index1001[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&masterobject_obj1001, NULL }
                     };

/* index 0x1003 :   Pre-defined Error Field */
                    UNS8 masterobject_highestSubIndex_obj1003 = 0; /* number of subindex - 1*/
                    UNS32 masterobject_obj1003[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex masterobject_Index1003[] = 
                     {
                       { RW, valueRange_EMC, sizeof (UNS8), (void*)&masterobject_highestSubIndex_obj1003, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&masterobject_obj1003[0], NULL }
                     };

/* index 0x1005 :   SYNC COB ID */
                    UNS32 masterobject_obj1005 = 0x0;   /* 0 */

/* index 0x1006 :   Communication / Cycle Period */
                    UNS32 masterobject_obj1006 = 0x0;   /* 0 */

/* index 0x100C :   Guard Time */ 
                    UNS16 masterobject_obj100C = 0x0;   /* 0 */

/* index 0x100D :   Life Time Factor */ 
                    UNS8 masterobject_obj100D = 0x0;   /* 0 */

/* index 0x1014 :   Emergency COB ID */
                    UNS32 masterobject_obj1014 = 0x80 + 0x00;   /* 128 + NodeID */

/* index 0x1016 :   Consumer Heartbeat Time */
                    UNS8 masterobject_highestSubIndex_obj1016 = 0;
                    UNS32 masterobject_obj1016[]={0};

/* index 0x1017 :   Producer Heartbeat Time. */
                    UNS16 masterobject_obj1017 = 0x0;	/* 0 */
                    subindex masterobject_Index1017[] = 
                     {
                       { RW, uint16, sizeof (UNS16), (void*)&masterobject_obj1017, NULL }
                     };

/* index 0x1018 :   Identity. */
                    UNS8 masterobject_highestSubIndex_obj1018 = 4; /* number of subindex - 1*/
                    UNS32 masterobject_obj1018_Vendor_ID = 0x0;	/* 0 */
                    UNS32 masterobject_obj1018_Product_Code = 0x0;	/* 0 */
                    UNS32 masterobject_obj1018_Revision_Number = 0x0;	/* 0 */
                    UNS32 masterobject_obj1018_Serial_Number = 0x0;	/* 0 */
                    subindex masterobject_Index1018[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&masterobject_highestSubIndex_obj1018, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&masterobject_obj1018_Vendor_ID, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&masterobject_obj1018_Product_Code, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&masterobject_obj1018_Revision_Number, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&masterobject_obj1018_Serial_Number, NULL }
                     };

/* index 0x1280 :   Client SDO 1 Parameter. */
                    UNS8 masterobject_highestSubIndex_obj1280 = 3; /* number of subindex - 1*/
                    UNS32 masterobject_obj1280_COB_ID_Client_to_Server_Transmit_SDO = 0x67F;	/* 0 */
                    UNS32 masterobject_obj1280_COB_ID_Server_to_Client_Receive_SDO = 0x5FF;	/* 0 */
                    UNS8 masterobject_obj1280_Node_ID_of_the_SDO_Server = 0x7F;	/* 1 */
                    subindex masterobject_Index1280[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&masterobject_highestSubIndex_obj1280, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1280_COB_ID_Client_to_Server_Transmit_SDO, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1280_COB_ID_Server_to_Client_Receive_SDO, NULL },//d->indextable[4].masterobject_Index1280[2].pobject=masterobject_obj1280_COB_ID_Server_to_Client_Receive_SDO
                       { RW, uint8, sizeof (UNS8), (void*)&masterobject_obj1280_Node_ID_of_the_SDO_Server, NULL }
                     };

/* index 0x1281 :   Client SDO 2 Parameter. */
                    UNS8 masterobject_highestSubIndex_obj1281 = 3; /* number of subindex - 1*/
                    UNS32 masterobject_obj1281_COB_ID_Client_to_Server_Transmit_SDO = 0x601;	/* 0 */
                    UNS32 masterobject_obj1281_COB_ID_Server_to_Client_Receive_SDO = 0x581;	/* 0 */
                    UNS8 masterobject_obj1281_Node_ID_of_the_SDO_Server = 0x1;	/* 2 */
                    subindex masterobject_Index1281[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&masterobject_highestSubIndex_obj1281, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1281_COB_ID_Client_to_Server_Transmit_SDO, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1281_COB_ID_Server_to_Client_Receive_SDO, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&masterobject_obj1281_Node_ID_of_the_SDO_Server, NULL }
                     };

/* index 0x1282 :   Client SDO 3 Parameter. */
                    UNS8 masterobject_highestSubIndex_obj1282 = 3; /* number of subindex - 1*/
                    UNS32 masterobject_obj1282_COB_ID_Client_to_Server_Transmit_SDO = 0x602;	/* 0 */
                    UNS32 masterobject_obj1282_COB_ID_Server_to_Client_Receive_SDO = 0x582;	/* 0 */
                    UNS8 masterobject_obj1282_Node_ID_of_the_SDO_Server = 0x2;	/* 3 */
                    subindex masterobject_Index1282[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&masterobject_highestSubIndex_obj1282, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1282_COB_ID_Client_to_Server_Transmit_SDO, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1282_COB_ID_Server_to_Client_Receive_SDO, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&masterobject_obj1282_Node_ID_of_the_SDO_Server, NULL }
                     };

/* index 0x1283 :   Client SDO 4 Parameter. */
                    UNS8 masterobject_highestSubIndex_obj1283 = 3; /* number of subindex - 1*/
                    UNS32 masterobject_obj1283_COB_ID_Client_to_Server_Transmit_SDO = 0x603;	/* 0 */
                    UNS32 masterobject_obj1283_COB_ID_Server_to_Client_Receive_SDO = 0x583;	/* 0 */
                    UNS8 masterobject_obj1283_Node_ID_of_the_SDO_Server = 0x3;	/* 4 */
                    subindex masterobject_Index1283[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&masterobject_highestSubIndex_obj1283, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1283_COB_ID_Client_to_Server_Transmit_SDO, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&masterobject_obj1283_COB_ID_Server_to_Client_Receive_SDO, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&masterobject_obj1283_Node_ID_of_the_SDO_Server, NULL }
                     };
/* index 0x607A :   Mapped variable Target position */
                    subindex masterobject_Index607A[] = 
                     {
                       { RW, int32, sizeof (INTEGER32), (void*)&Target_position, NULL }
                     };

/**************************************************************************/
/* Declaration of pointed variables                                       */
/**************************************************************************/

const indextable masterobject_objdict[] = 
{
  { (subindex*)masterobject_Index1000,sizeof(masterobject_Index1000)/sizeof(masterobject_Index1000[0]), 0x1000},
  { (subindex*)masterobject_Index1001,sizeof(masterobject_Index1001)/sizeof(masterobject_Index1001[0]), 0x1001},
  { (subindex*)masterobject_Index1017,sizeof(masterobject_Index1017)/sizeof(masterobject_Index1017[0]), 0x1017},
  { (subindex*)masterobject_Index1018,sizeof(masterobject_Index1018)/sizeof(masterobject_Index1018[0]), 0x1018},
  { (subindex*)masterobject_Index1280,sizeof(masterobject_Index1280)/sizeof(masterobject_Index1280[0]), 0x1280},
	{ (subindex*)masterobject_Index1281,sizeof(masterobject_Index1281)/sizeof(masterobject_Index1281[0]), 0x1281},
  { (subindex*)masterobject_Index1282,sizeof(masterobject_Index1282)/sizeof(masterobject_Index1282[0]), 0x1282},
  { (subindex*)masterobject_Index1283,sizeof(masterobject_Index1283)/sizeof(masterobject_Index1283[0]), 0x1283},
  { (subindex*)masterobject_Index607A,sizeof(masterobject_Index607A)/sizeof(masterobject_Index607A[0]), 0x607A},
};

const indextable * masterobject_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode)
{
	int i;
	switch(wIndex){
		case 0x1000: i = 0;break;
		case 0x1001: i = 1;break;
		case 0x1017: i = 2;break;
		case 0x1018: i = 3;break;
		case 0x1280: i = 4;break;
		case 0x1281: i = 5;break;
		case 0x1282: i = 6;break;
		case 0x1283: i = 7;break;
		case 0x607A: i = 8;break;
		default:
			*errorCode = OD_NO_SUCH_OBJECT;
			return NULL;
	}
	*errorCode = OD_SUCCESSFUL;
	return &masterobject_objdict[i];
}

/* 
 * To count at which received SYNC a PDO must be sent.
 * Even if no pdoTransmit are defined, at least one entry is computed
 * for compilations issues.
 */
s_PDO_status masterobject_PDO_status[1] = {s_PDO_status_Initializer};

const quick_index masterobject_firstIndex = {
  0, /* SDO_SVR */
  4, /* SDO_CLT */
  0, /* PDO_RCV */
  0, /* PDO_RCV_MAP */
  0, /* PDO_TRS */
  0 /* PDO_TRS_MAP */
};

const quick_index masterobject_lastIndex = {
  0, /* SDO_SVR */
  7, /* SDO_CLT */
  0, /* PDO_RCV */
  0, /* PDO_RCV_MAP */
  0, /* PDO_TRS */
  0 /* PDO_TRS_MAP */
};

const UNS16 masterobject_ObjdictSize = sizeof(masterobject_objdict)/sizeof(masterobject_objdict[0]); 

CO_Data masterobject_Data = CANOPEN_NODE_DATA_INITIALIZER(masterobject);

