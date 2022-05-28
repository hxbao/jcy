#ifndef _ATLCANPROCOL_H
#define _ATLCANPROCOL_H
#include "includes.h"

typedef struct {
    uint16_t Bat_Module_Vol[20];//单体电压 1mv offset 0 0-5000 
    uint16_t Bat_Module_Temp[8];//0.1摄氏度 offset 400 -100-200
    uint32_t Bat_Module_Sum_Vol;//0.1V 0 0-1000
    uint32_t Bat_Module_Cur;//电池电流 1mA offset 3000000 0~0xFFFFFFFF
    uint16_t Bat_Module_Pack_Vol;//0.1 0 0-1000
    uint16_t Bat_Module_SOP; //0.01kw 0-500
    uint16_t Bat_Module_Max_Vol; //单体最大电压 2 uint16 mV 1mV 0 0~5000
    uint16_t Bat_Module_Min_Vol; //单体最小电压 2 uint16 mV 1mV 0 0~5000
    uint16_t Bat_Module_Max_Vol_Delta;//单体最大压差 2 uint16 mV 1mV 0 0~5000
    uint8_t Bat_Module_Max_Vol_No; //单体最大电压位置 1 uint8 0~30
    uint8_t Bat_Module_Min_Vol_No; //单体最小电压位置 1 uint8 0~30
    uint16_t Bat_Module_Max_Temp;           //电池最大温度 2 int16 ℃ 0.1℃ 400 -100~200
    uint16_t Bat_Module_Min_Temp;//电池最小温度 2 int16 ℃ 0.1℃ 400-100~200
    uint16_t Bat_Module_Max_Temp_Delta;
    uint8_t Bat_Module_Max_Temp_No;//位置
    uint8_t Bat_Module_Min_Temp_No;//位置
    uint16_t Bat_Max_Chg_Cur;//电池最大允许充电持续电流 2 uint16 A 0.1A 30000 -1000~1000
    uint16_t Bat_Max_Dch_Cur;//电池最大允许放电持续电流 2 uint16 A 0.1A 30000 -1000~1000
    uint16_t Bat_Actual_Capacity;//电池实际容量 2 uint16 Ah 0.01Ah 0 0~500
    uint16_t Bat_RemainCapacity;// 电池剩余容量 2 uint16 Ah 0.01Ah 0 0~500
    uint32_t Bat_Total_In_Ah;//电池充电总Ah数 4 uint32 Ah 1Ah 0
    uint32_t Bat_Total_Out_Ah;//电池放电总Ah数 4 uint32 Ah 1Ah 0
    uint16_t Bat_Full_Chg_Capacity;//电池满充容量 2 uint16 Ah 0.01Ah 0
    uint16_t Bat_Cycle_count;// 电池充放电循环次数 2 uint1
    uint16_t Bat_Avg_V;// pack平均电压 2 uint16 mV 1mV 0 0~5000
    uint16_t Bat_Avg_T;//（signed short） pack平均温度 2 int16 ℃ 0.1℃ 400 -100~200
    uint16_t Bat_Heat_Vol;// 加热膜电压 2 uint16 V 0.1V 0 0~1000
    uint8_t BMS_Run_Status;// 电池运行状态 1 uint8 0~255
    uint8_t BMS_Work_Status;//电池工作状态 1 uint8 0~255
    uint8_t BMS_Mos_Status;// MOS工作状态 1 uint8 0~255
    uint8_t BMS_Hard_Status;// 硬件信号状态 1 uint8 0~255
    uint32_t BMS_Bal_Status[2];// 电池均衡状态 4 uint32
    uint16_t PACK_ISO_R_P; //PACK正绝缘电阻 2 uint16 KΩ 0.1KΩ 0 0~65535
    uint16_t PACK_ISO_R_N; //PACK负绝缘电阻 2 uint16 KΩ 0.1KΩ 0 0~65535
    uint32_t BMS_Systemy_Error; //PACK系统错误 4 uint32
    uint32_t BMS_Hard_Error;    //PACK硬件错误 4 uint32
    uint16_t Bat_SOC;          //电池剩余电量 2 uint16 % 0.10% 0 0~100
    uint16_t Bat_SOH;          //电池健康状态 2 uint16 % 0.10% 0 0~100
    uint16_t Bat_SocUser;      //用户使用的SOC
    uint16_t Bat_MaxPulse_ChgCur;
    uint16_t Bat_MaxPulse_DchCur;
    uint16_t BMS_Run_Flg;
    uint8_t  WakeUp_Flg;
    uint8_t  Reset_status;
    uint8_t  BMS_Battery_Alarm1[8];
    uint8_t  BMS_Battery_Alarm2[8];
    uint8_t  BMS_System_Alarm1[8];
    uint8_t  BMS_System_Alarm2[8];
    uint8_t  BMS_System_Alarm3[8];
    uint8_t  BMS_HardWare_Alarm1[8];
    uint8_t  BMS_HardWare_Alarm2[8];
    uint8_t  BMS_HardWare_Alarm3[8];
    uint8_t  BMS_HardWare_Alarm4[8];
    uint8_t  BMS_HardWare_Alarm5[8];
    uint16_t Bat_Chg_Remain_Time;
    uint16_t Bat_Dsg_Remain_Time;
    uint16_t Bat_Max_Allow_Chg_Power;
    uint16_t Bat_Max_Allow_Dsg_Power;
    uint32_t Bat_Max_Allow_FeedBack_Cur;
    uint16_t Bat_Max_Allow_FeedBack_Power;
    uint16_t Bat_SOE;
    uint16_t Bat_Max_Dch_30s_Cur;
    uint16_t Bat_DsgCur_30s;
    uint16_t Bat_Max_Dch_5s_Cur;
    uint16_t Bat_DsgCur_5s;
    uint16_t  Bat_ChgCur_Contiue;
    uint16_t  Bat_ChgCur_Instant;
    uint16_t  Bat_DsgCur_Continue;
    uint16_t  Bat_DsgCur_Instant;
    uint16_t  BMS_BalTemp_1;
    uint16_t  BMS_BalTemp_2;
    uint16_t  BMS_HeatTemp_1;
    uint16_t  BMS_HeatTemp_2;
    uint16_t  BMS_EnvTemp;
    uint16_t  BMS_ICTemp;
    uint16_t  Usoc;
    uint16_t  UsocRsvd;
    uint16_t  SOH;
    uint16_t  Rsoc0;
    uint16_t  Rsoc1;
    uint16_t  AsocFilt;
    uint16_t  AsocOCV;
    uint16_t  AsocAh0;
    uint16_t  AsocAH1;
    uint16_t  AsocFull0;
    uint16_t  AsocFull1;
    uint16_t  AsocEmpty0;
    uint16_t  AsocEmpty1;
    uint16_t  PassChg0;
    uint16_t  PassChg1;
    uint16_t  QMax0;
    uint16_t  QMax1;
    uint16_t  FCC;
    uint16_t  RemCap;
    uint16_t  Cycle;
    uint16_t  TotalChgAh_L;
    uint16_t  TotalChgAh_H;
    uint16_t  TotalDsgAh_L;
    uint16_t  TotalDsgAh_H;
    uint16_t  TotalChgwh_L;
    uint16_t  TotalChgwh_H;
    uint16_t  TotalDsgWh_L;
    uint16_t  TotalDsgWh_H;
    uint16_t  Flag;
    uint16_t  Source_12V;
    uint16_t  Source_5V;
    uint16_t  Sourve_3_3V;
}ATLCANData_t;

extern ATLCANData_t atlCanData;

void RYCAN_Init(void);
uint8_t RYCAN_SendSilence();
uint8_t RYCAN_TxProcess(void);

#endif
