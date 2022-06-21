#include "includes.h"

#if (PROJECT_ID == 1)

#define  CAN_FILTER_STDID(STDID)    ((STDID&0x7FF)<<5)
#define  CAN_FILTER_EXTID_H(EXTID)  ((uint16_t)(((EXTID<<3)>>16) & 0xffff))
#define  CAN_FILTER_EXTID_L(EXTID)  ((uint16_t)(((EXTID<<3)& 0xffff)|0x0004))

#define ATL_EXTID_VLT1_4   0x8009100
#define ATL_EXTID_VLT5_8   0x8019100
#define ATL_EXTID_VLT9_12  0x8029100
#define ATL_EXTID_VLT13_16 0x8039100
#define ATL_EXTID_VLT17_20 0x8049100
#define ATL_EXTID_TEMP1_4  0x10009100
#define ATL_EXTID_TEMP5_8  0x10019100
//总信息
#define ATL_EXTID_INFO0    0x4009100
#define ATL_EXTID_INFO1    0x4019100
#define ATL_EXTID_INFO2    0x4029100
#define ATL_EXTID_INFO3    0x4039100
#define ATL_EXTID_INFO4    0x4049100
#define ATL_EXTID_INFO5    0x4059100
#define ATL_EXTID_INFO6    0x4069100
#define ATL_EXTID_INFO7    0x4079100
#define ATL_EXTID_INFO8    0x4089100
#define ATL_EXTID_INFO9    0x4099100
#define ATL_EXTID_INFO10   0x40A9100
#define ATL_EXTID_INFO11   0x4109100
#define ATL_EXTID_INFO12   0x4119100
#define ATL_EXTID_INFO13   0x4129100
#define ATL_EXTID_INFO14   0x4139100
#define ATL_EXTID_INFO15   0x4149100
#define ATL_EXTID_INFO16   0x4159100
#define ATL_EXTID_INFO17   0x4169100
#define ATL_EXTID_INFO18   0x4179100
#define ATL_EXTID_INFO19   0x4189100
#define ATL_EXTID_INFO20   0x4199100
//#define ATL_EXTID_INFO21   0x4179100
#define ATL_EXTID_INFO22   0x4209100
#define ATL_EXTID_INFO23   0x4219100
#define ATL_EXTID_INFO24   0x4229100

#define ATL_EXTID_INFO25   0x4239100
#define ATL_EXTID_INFO26   0x4249100
#define ATL_EXTID_INFO27   0x4259100
#define ATL_EXTID_INFO28   0x4269100
#define ATL_EXTID_INFO29   0x4279100
#define ATL_EXTID_INFO30   0x4289100
#define ATL_EXTID_INFO31   0x4299100
#define ATL_EXTID_INFO32   0x4309100
#define ATL_EXTID_INFO33   0x4319100
#define ATL_EXTID_INFO34   0x4329100
#define ATL_EXTID_INFO35   0x4339100
#define ATL_EXTID_INFO36   0x4349100
#define ATL_EXTID_INFO37   0x4359100
#define ATL_EXTID_INFO38   0x4369100
#define ATL_EXTID_INFO39   0x4379100
#define ATL_EXTID_INFO40   0x4389100
#define ATL_EXTID_INFO41   0x4399100


CanTxMessage TxMessage;
ATLCANData_t atlCanData;

uint8_t AtlCanFlag;

uint32_t ATL_RX_EXTID_Table[48] = {
 ATL_EXTID_VLT1_4 ,
 ATL_EXTID_VLT5_8  ,
 ATL_EXTID_VLT9_12  ,
 ATL_EXTID_VLT13_16 ,
 ATL_EXTID_VLT17_20 ,
 ATL_EXTID_TEMP1_4  ,
 ATL_EXTID_TEMP5_8  ,
//总信息
 ATL_EXTID_INFO0    ,
 ATL_EXTID_INFO1    ,
 ATL_EXTID_INFO2    ,
 ATL_EXTID_INFO3    ,
 ATL_EXTID_INFO4    ,
 ATL_EXTID_INFO5    ,
 ATL_EXTID_INFO6    ,
 ATL_EXTID_INFO7    ,
 ATL_EXTID_INFO8    ,
 ATL_EXTID_INFO9    ,
 ATL_EXTID_INFO10   ,
 ATL_EXTID_INFO11   ,
 ATL_EXTID_INFO12   ,
 ATL_EXTID_INFO13   ,
 ATL_EXTID_INFO14   ,
 ATL_EXTID_INFO15   ,
 ATL_EXTID_INFO16   ,
 ATL_EXTID_INFO17   ,
 ATL_EXTID_INFO18   ,
 ATL_EXTID_INFO19   ,
 ATL_EXTID_INFO20   ,
//#define ATL_EXTID_INFO21   0x4179100
 ATL_EXTID_INFO22   ,
 ATL_EXTID_INFO23   ,
 ATL_EXTID_INFO24   ,

 ATL_EXTID_INFO25   ,
 ATL_EXTID_INFO26   ,
 ATL_EXTID_INFO27   ,
 ATL_EXTID_INFO28   ,
 ATL_EXTID_INFO29   ,
 ATL_EXTID_INFO30   ,
 ATL_EXTID_INFO31   ,
 ATL_EXTID_INFO32   ,
 ATL_EXTID_INFO33   ,
 ATL_EXTID_INFO34   ,
 ATL_EXTID_INFO35   ,
 ATL_EXTID_INFO36   ,
 ATL_EXTID_INFO37   ,
 ATL_EXTID_INFO38   ,
 ATL_EXTID_INFO39   ,
 ATL_EXTID_INFO40   ,
 ATL_EXTID_INFO41   ,

};

static void ATL_CANIDDataParser(CanRxMessage rmsg,uint32_t canid)
{
    switch(canid)
    {
        case ATL_EXTID_VLT1_4:
            atlCanData.Bat_Module_Vol[0] =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Vol[1] =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Vol[2] =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Vol[3] =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_VLT5_8:
            atlCanData.Bat_Module_Vol[4] =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Vol[5] =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Vol[6] =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Vol[7] =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_VLT9_12:
            atlCanData.Bat_Module_Vol[8] =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Vol[9] =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Vol[10] =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Vol[11] =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_VLT13_16:
            atlCanData.Bat_Module_Vol[12] =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Vol[13] =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Vol[14] =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Vol[15] =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_VLT17_20:
            atlCanData.Bat_Module_Vol[16] =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Vol[17] =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Vol[18] =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Vol[19] =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_TEMP1_4:
            atlCanData.Bat_Module_Temp[0] =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Temp[1] =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Temp[2] =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Temp[3] =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_TEMP5_8:
            atlCanData.Bat_Module_Temp[4] =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Temp[5] =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Temp[6] =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Temp[7] =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_INFO0:
            //按照电池信息表的定义补充完整
            atlCanData.Bat_Module_Sum_Vol   =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Cur       =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8|(rmsg.Data[4]<<16)|(rmsg.Data[5]<<24);
            atlCanData.Bat_Module_Pack_Vol  =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_INFO1:
            //按照电池信息表的定义补充完整
            atlCanData.Bat_Module_Max_Vol       =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Min_Vol       =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Max_Vol_Delta =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Max_Vol_No    =  rmsg.Data[6];
            atlCanData.Bat_Module_Min_Vol_No    =  rmsg.Data[7];
        break;
        case ATL_EXTID_INFO2:
            //按照电池信息表的定义补充完整
            atlCanData.Bat_Module_Max_Temp          =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Module_Min_Temp          =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Module_Max_Temp_Delta    =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Module_Max_Temp_No       =  rmsg.Data[6];
            atlCanData.Bat_Module_Min_Temp_No       =  rmsg.Data[7];
        break;
        case ATL_EXTID_INFO3:
            //按照电池信息表的定义补充完整
            atlCanData.Bat_SocUser    =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_SOH        =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_SOC        =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_Heat_Vol   =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_INFO4:
            atlCanData.Bat_Max_Chg_Cur      =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Max_Dch_Cur      =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_MaxPulse_ChgCur  =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.Bat_MaxPulse_DchCur  =  rmsg.Data[6]|(uint16_t)rmsg.Data[7]<<8;
        break;
        case ATL_EXTID_INFO5:
            atlCanData.Bat_Total_In_Ah    =  rmsg.Data[0]|(rmsg.Data[1]<<8)|(rmsg.Data[2]<<16)|(rmsg.Data[3]<<24);
            atlCanData.Bat_Total_Out_Ah   =  rmsg.Data[4]|(rmsg.Data[5]<<8)|(rmsg.Data[6]<<16)|(rmsg.Data[7]<<24);
        break;
        case ATL_EXTID_INFO6:
            atlCanData.Bat_Actual_Capacity  =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_RemainCapacity   =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
            atlCanData.Bat_Cycle_count      =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
        break;
        case ATL_EXTID_INFO7:
            atlCanData.Bat_Avg_V    =  rmsg.Data[0]|(uint16_t)rmsg.Data[1]<<8;
            atlCanData.Bat_Avg_T    =  rmsg.Data[2]|(uint16_t)rmsg.Data[3]<<8;
        break;
        case ATL_EXTID_INFO8:
            atlCanData.BMS_Run_Status    =  rmsg.Data[0];
            atlCanData.BMS_Work_Status   =  rmsg.Data[1];
            atlCanData.BMS_Mos_Status    =  rmsg.Data[2];
            atlCanData.BMS_Hard_Status   =  rmsg.Data[3];
            atlCanData.BMS_Run_Flg       =  rmsg.Data[4]|(uint16_t)rmsg.Data[5]<<8;
            atlCanData.WakeUp_Flg        =  rmsg.Data[6];
            atlCanData.Reset_status      =  rmsg.Data[7];
        break;
        case ATL_EXTID_INFO9:
            atlCanData.BMS_Bal_Status[0]=rmsg.Data[0]|(rmsg.Data[1]<<8)|(rmsg.Data[2]<<16)|(rmsg.Data[3]<<24);
            atlCanData.BMS_Bal_Status[1]=rmsg.Data[4]|(rmsg.Data[5]<<8)|(rmsg.Data[6]<<16)|(rmsg.Data[7]<<24);
        break; 
        case ATL_EXTID_INFO10:
            atlCanData.BMS_Bal_Status[0]=rmsg.Data[0]|(rmsg.Data[1]<<8)|(rmsg.Data[2]<<16)|(rmsg.Data[3]<<24);    
        break;
        case ATL_EXTID_INFO11:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_Battery_Alarm1[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO12:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_Battery_Alarm2[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO13:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_System_Alarm1[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO14:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_System_Alarm2[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO15:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_System_Alarm3[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO16:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_HardWare_Alarm1[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO17:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_HardWare_Alarm2[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO18:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_HardWare_Alarm3[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO19:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_HardWare_Alarm4[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO20:
            for(int i=0;i<8;i++)
            {
                atlCanData.BMS_HardWare_Alarm5[i]=rmsg.Data[i];
            }
        break;
        case ATL_EXTID_INFO22:
            atlCanData.PACK_ISO_R_P =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.PACK_ISO_R_N =   rmsg.Data[2]|(rmsg.Data[3]<<8); 
        break;
        case ATL_EXTID_INFO23:
            atlCanData.Bat_Chg_Remain_Time =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.Bat_Dsg_Remain_Time =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.Bat_Max_Allow_Chg_Power =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.Bat_Max_Allow_Dsg_Power =   rmsg.Data[6]|(rmsg.Data[7]<<8); 
        break;
        case ATL_EXTID_INFO24:
            atlCanData.Bat_Max_Allow_FeedBack_Cur   =   rmsg.Data[0]|(rmsg.Data[1]<<8)|(rmsg.Data[2]<<16)|(rmsg.Data[3]<<24);    
            atlCanData.Bat_Max_Allow_FeedBack_Power =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.Bat_SOE =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break;
        case ATL_EXTID_INFO25:
            atlCanData.Bat_Max_Dch_30s_Cur   =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.Bat_DsgCur_30s        =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.Bat_Max_Dch_5s_Cur    =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.Bat_DsgCur_5s         =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break;
        case ATL_EXTID_INFO26:
            atlCanData.Bat_ChgCur_Contiue       =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.Bat_ChgCur_Instant       =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.Bat_DsgCur_Continue      =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.Bat_DsgCur_Instant       =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break;
        case ATL_EXTID_INFO27:
            atlCanData.BMS_BalTemp_1       =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.BMS_BalTemp_2       =   rmsg.Data[2]|(rmsg.Data[3]<<8);
        break;
        case ATL_EXTID_INFO29:
            atlCanData.BMS_HeatTemp_1       =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.BMS_HeatTemp_2       =   rmsg.Data[2]|(rmsg.Data[3]<<8);
        break;
        case ATL_EXTID_INFO31:
            atlCanData.BMS_EnvTemp       =   rmsg.Data[0]|(rmsg.Data[1]<<8);
        break;
        case ATL_EXTID_INFO32:
            atlCanData.BMS_ICTemp       =   rmsg.Data[0]|(rmsg.Data[1]<<8);
        break;
        case ATL_EXTID_INFO33:
            atlCanData.Usoc      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.UsocRsvd  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.SOH       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.Rsoc0     =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break;
        case ATL_EXTID_INFO34:
            atlCanData.Rsoc1      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.AsocFilt  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.AsocOCV       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.AsocAh0     =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break; 
        case ATL_EXTID_INFO35:
            atlCanData.AsocAH1      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.AsocFull0  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.AsocFull1       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.AsocEmpty0     =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break; 
        case ATL_EXTID_INFO36:
            atlCanData.AsocEmpty1      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.PassChg0  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.PassChg1       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.QMax0     =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break; 
        case ATL_EXTID_INFO37:
            atlCanData.QMax1      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.FCC  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.RemCap       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.Cycle     =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break; 
        case ATL_EXTID_INFO38:
            atlCanData.TotalChgAh_L      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.TotalChgAh_H  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.TotalDsgAh_L       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.TotalDsgAh_H     =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break; 
        case ATL_EXTID_INFO39:
            atlCanData.TotalChgwh_L      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.TotalChgwh_H  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.TotalDsgWh_L       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
            atlCanData.TotalDsgWh_H     =   rmsg.Data[6]|(rmsg.Data[7]<<8);
        break; 
        case ATL_EXTID_INFO40:
            atlCanData.Flag      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
        break; 
        case ATL_EXTID_INFO41:
            atlCanData.Source_12V      =   rmsg.Data[0]|(rmsg.Data[1]<<8);
            atlCanData.Source_5V  =   rmsg.Data[2]|(rmsg.Data[3]<<8);
            atlCanData.Sourve_3_3V       =   rmsg.Data[4]|(rmsg.Data[5]<<8);
        break;  
        //按照电池基本信息表定义的关系，补充完整

    }
}


static uint8_t RYCAN_Send0x45b(uint8_t *buf,uint8_t len)
{
    uint8_t i;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x45b;
    TxMessage.DLC = len;

    for(i = 0;i< len;i++)
    {
        TxMessage.Data[i] = *(buf + i);
    }
    return CANTxMessage(CAN,&TxMessage);
}

//定义iaphanle协议，包装iaphanle数据接收协议
static void CanIapHandle(CanRxMessage rmsg)
{
    uint8_t i;
    uint8_t dlen = rmsg.DLC;
    //stdid - 0x45a 升级写数据id 
    //stdid - 0x45b 升级回写数据id

    if(rmsg.StdId == 0x45a)
    {
        for(i = 0;i<dlen;i++)
        {
            NIU_ModbusRecvHandle(rmsg.Data[i]);
        }
    }else
    {

        for(i = 0;i < 48;i++)
        {
            if((rmsg.ExtId & 0x1FFFFFFF) == ATL_RX_EXTID_Table[i])
            {
                //调用相应的CANID 解析到数据结构
                ATL_CANIDDataParser(rmsg,ATL_RX_EXTID_Table[i]);
            }
        }
    }
    
}

void RYCAN_Init(void)
{
    //定义接收的IDlist
    uint16_t idList[2] = {0x06f2,0x045a};
    //设置IdMask过滤器

    idList[0] = CAN_FILTER_STDID(idList[0]);
    idList[1] = CAN_FILTER_STDID(idList[1]);
    BxCanPortInit();
    BxCanConfig(RYCAN_RxProcess,idList,0,0x00000000,1);//全部接收CAN数据包
    //bsp_StartAutoTimer(TMR_ONEBUS_CHECK,100);
}

//500ms 周期性发送
uint8_t RYCAN_SendSilence()
{
    static uint8_t LifeCnt = 0;

    TxMessage.IDE = CAN_Extended_Id;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.ExtId = 0x18FF0091;
    //TxMessage.StdId = 0x45b;
    TxMessage.DLC = 8;
    TxMessage.Data[0] = 0x00;
    TxMessage.Data[1] = 0x00;
    TxMessage.Data[2] = 0x00;
    TxMessage.Data[3] = 0x00;
    TxMessage.Data[4] = 0x00;
    TxMessage.Data[5] = 0x00;
    TxMessage.Data[6] = 0x00;
    TxMessage.Data[7] = LifeCnt++;
    return CANTxMessage(CAN,&TxMessage);
}

void RYCAN_SendData(uint8_t *buf,uint16_t len)
{
    uint16_t i = 0;
    uint16_t dl = len;
    uint8_t ret;
    //发送stdid - 0x45b 
    //分块，分片传送
    do{
        if(dl >8)
        {

            //ret = RYCAN_Send0x45b(buf+i,8);
            do{                
                ret = RYCAN_Send0x45b(buf+i,8);
                if(ret != CAN_TxSTS_NoMailBox)
                {
                    break;
                }
                bsp_DelayUS(1000);
            }while(1);
            
            i += 8;
            dl -=8;
        }else
        {            
            do{                
                ret = RYCAN_Send0x45b(buf+i,dl);
                if(ret != CAN_TxSTS_NoMailBox)
                {
                    break;
                }
                bsp_DelayUS(1000);
            }while(1);
            break;
        }        
    }while(1);   
}

uint8_t RYCAN_TxProcess(void)
{
    uint8_t TransmitMailbox = 0;
	uint16_t Time_out=0xFFFF;
    //if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {
        TransmitMailbox=RYCAN_SendSilence();
    }
}

void RYCAN_RxProcess(CanRxMessage rxm)
{
    if(rxm.StdId == 0x6f2)
    {
        // RYCAN_WriteRomHandle(rxm);
        // RYCAN_ReadRomHandle(rxm);
    }else
    {
        CanIapHandle(rxm);
        AtlCanFlag=0x04;
    }
}

/** 
* @brief  	得到CAN模式标志位
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_can_flag()
{
    return AtlCanFlag;
}
/** 
* @brief  	清除CAN标记位
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_can_clear()
{
    AtlCanFlag=0;
    return AtlCanFlag;
}

/** 
* @brief  	can总线上报电池单体电压
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_vol_cell(uint8_t num)
{
	return atlCanData.Bat_Module_Vol[num];
}
/** 
* @brief  	can总线上报电池电流
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint32_t get_can_cur_value(void)
{
	return atlCanData.Bat_Module_Cur;
}
/** 
* @brief  	can总线上报单体最大电压
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_max_cell_vol(void)
{
	return atlCanData.Bat_Module_Max_Vol;
}
/** 
* @brief  	can总线上报单体最小电压
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_min_cell_vol(void)
{
	return atlCanData.Bat_Module_Min_Vol;
}
/** 
* @brief  	can总线上报单体压差
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_vol_delta(void)
{
	return atlCanData.Bat_Module_Max_Vol_Delta;
}
/** 
* @brief  	can上报电池最大温度
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_max_temp(void)
{
    if(atlCanData.Bat_Module_Max_Temp==0)
    {
        return atlCanData.Bat_Module_Max_Temp;
    }
	return atlCanData.Bat_Module_Max_Temp-400;
}
/** 
* @brief  	can上报电池最小温度
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_min_temp(void)
{
    if(atlCanData.Bat_Module_Min_Temp==0)
    {
         return atlCanData.Bat_Module_Min_Temp;
    }
	return atlCanData.Bat_Module_Min_Temp-400;
}
/** 
* @brief  	can上报电池总压
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_max_vol(void)
{
	return atlCanData.Bat_Module_Sum_Vol;
}
/** 
* @brief    can上报电池SOC
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_soc(void)
{
	return atlCanData.Bat_SOC;
}
/** 
* @brief    can上报电池SOH
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_soh(void)
{
	return atlCanData.Bat_SOH;
}
/** 
* @brief  	can上报电池最大充电电流
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_max_ch_cur(void)
{
    if(atlCanData.Bat_Max_Chg_Cur==0)
    {
        return atlCanData.Bat_Max_Chg_Cur;
    }
	return (atlCanData.Bat_Max_Chg_Cur-30000)/10;
}
/** 
* @brief  	can上报电池最大放电电流
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_max_dcg_cur(void)
{
	return atlCanData.Bat_Max_Dch_Cur;
}
/** 
* @brief  	can上报电池循环次数
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_can_bat_circle(void)
{
	return atlCanData.Cycle;
}
/** 
* @brief  	bms返回faultcode 蓝牙询问查�??
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint32_t get_can_bat_fault()
{
    uint8_t checkfault;
    uint8_t faultnum;
    for(int i;i<8;i++)
    {
        faultnum=atlCanData.BMS_Battery_Alarm1[i];
        faultnum=atlCanData.BMS_Battery_Alarm2[i];
        faultnum=atlCanData.BMS_System_Alarm1[i];
        faultnum=atlCanData.BMS_System_Alarm2[i];
        faultnum=atlCanData.BMS_System_Alarm3[i];
        faultnum=atlCanData.BMS_HardWare_Alarm1[i];
        faultnum=atlCanData.BMS_HardWare_Alarm2[i];
        faultnum=atlCanData.BMS_HardWare_Alarm3[i];
        faultnum=atlCanData.BMS_HardWare_Alarm4[i];
        faultnum=atlCanData.BMS_HardWare_Alarm5[i];
        if(faultnum)
        {
            checkfault|=0x0200;
            break;
        }
    }
    uint8_t BAT_OT=(atlCanData.BMS_Battery_Alarm1[7]>>4)|atlCanData.BMS_Battery_Alarm2[1];
    uint8_t BAT_UT=atlCanData.BMS_Battery_Alarm2[0]|(atlCanData.BMS_Battery_Alarm2[1]>>4); 
    uint8_t TOTAL_OV=atlCanData.BMS_Battery_Alarm1[1]>>4;
    uint8_t TOTAL_UV=atlCanData.BMS_Battery_Alarm1[2];
    uint8_t BAT_CHG_OI=atlCanData.BMS_Battery_Alarm1[2]>>4|atlCanData.BMS_Battery_Alarm1[3];
    uint8_t BAT_DSG_OI=atlCanData.BMS_Battery_Alarm1[4]>>4|atlCanData.BMS_Battery_Alarm1[5];
    uint8_t CELL_OV=atlCanData.BMS_Battery_Alarm2[4]>>4;
    uint8_t CELL_UV=atlCanData.BMS_Battery_Alarm2[3]>>4|atlCanData.BMS_Battery_Alarm2[4];
    uint8_t MOS_OI=atlCanData.BMS_System_Alarm1[4]|atlCanData.BMS_System_Alarm1[5]|atlCanData.BMS_System_Alarm1[6];

    if(BAT_OT)
    {
        checkfault|0x01;
    }
    if(BAT_UT)
    {
        checkfault|0x02;
    }
    if(TOTAL_OV)
    {
        checkfault|0x04;
    }
    if(TOTAL_UV)
    {
        checkfault|0x08;
    }
    if(BAT_CHG_OI)
    {
        checkfault|0x10;
    }
    if(BAT_DSG_OI)
    {
        checkfault|0x20;
    }
    if(CELL_OV)
    {
        checkfault|0x40;
    }
    if(CELL_UV)
    {
        checkfault|0x80;
    }
    if(MOS_OI)
    {
        checkfault|0x100;
    }

    checkfault|=get_device_fault_code();

    return checkfault;
}

#endif




