#ifndef _ATL485_H
#define _ATL485_H

#include "includes.h"

//modbus 基地址
#define ATL485_BASE_ADDR_SILENT       1

#define ATL485_BASE_ADDR_CELLV        1000
#define ATL485_BASE_ADDR_CELLT        2000
#define ATL485_BASE_ADDR_BATD         3000
#define ATL485_BASE_ADDR_BATSTA       4000
#define ATL485_BASE_ADDR_ERRHIS       5000
#define ATL485_BASE_ADDR_SOCDEBUG     6000
#define ATL485_BASE_ADDR_PROJECTINTO  7000
#define ATL485_BASE_ADDR_CALIINFO     8000
#define ATL485_BASE_ADDR_DEBUG        9000
#define ATL485_BASE_ADDR_BATTHRESHOLD  10000
#define ATL485_BASE_ADDR_PARAM1       11000
#define ATL485_BASE_ADDR_PARAM2       12000
#define ATL485_BASE_ADDR_PARAM3       13000
#define ATL485_BASE_ADDR_PARAM4       14000
#define ATL485_BASE_ADDR_CHGHIS       15000
#define ATL485_BASE_ADDR_F099HIS      16000

// extern Atl485_Cellv_t atl485cellv;
// extern Atl485_Cellt_t atl485cellt;
// extern Atl485_batd_t atl485batd;
// extern Atl485_BatState_t atl485batsta;
// extern Atl485_ProjectInfo_t atl485prjInfo;
// extern Atl485_ProjectRunthresd_t atl485runthred;
// extern Atl485_BatErrHis_t atl485baterrhis;
// extern Atl485_ChgHis_t atl485chghis;
// extern Atl485_F099HIS_t atl485f099his;



typedef struct{
    uint16_t cellv[13];
}Atl485_Cellv_t;//单体电压

typedef struct{
    int16_t cellt[10];
}Atl485_Cellt_t;//单体温度

typedef struct{
    uint16_t packSumVolt;   //0.1V
    uint16_t IntPackVolt;   //0.1V
    uint16_t ExtPackVolt;   //0.1V
    uint16_t PackCurrentl; //1mA
    uint16_t PackCurrenth;
    uint16_t SOC;            //0.1%
    uint16_t SOC_User;       //0.1%
    uint16_t SOH;            //0.1%
    uint16_t MaxCellVolt;    //1mV
    uint16_t MaxCellVoltNo;  //最高电压位置
    uint16_t MinCellVolt;
    uint16_t MinCellVoltNo;
    int16_t MaxCellTemp;
    uint16_t MaxCellTempNO;
    uint16_t MinCellTemp;
    uint16_t MinCellTempNo;
    int16_t  Max_CHGCUR1;  //允许最大充电电流1 0.1A
    int16_t  Max_CHGCUR2;  //允许最大充电电流2 0.1A
    int16_t  Max_PerDSGCUR1;//允许最大放电电流1 0.1A
    int16_t  Max_PerDSGCUR2;//允许最大放电电流2 0.1A
    uint16_t Bat_Max_Allow_Chg_Powe1; //允许最大充电功率1 0.01kW
    uint16_t Bat_Max_Allow_Chg_Powe2; //允许最大充电功率2 0.01kW
    uint16_t Bat_Max_Allow_Dsg_Powe1; //允许最大放电功率1 0.01kW
    uint16_t Bat_Max_Allow_Dsg_Powe2; //允许最大放电功率2 0.01kW
    uint16_t TotalCHGCapacity_Lo;     //累计充电容量低位
    uint16_t TotalCHGCapacity_Hi;     //累计充电容量高位
    uint16_t TotalDSGCapacity_Lo;     //累计放电容量低位
    uint16_t TotalDSGCapacity_Hi;     //累计放电容量高位
    uint16_t otalBatCHGSOE_Lo;        //累计充电能量低位
    uint16_t TotalBatCHGSOE_Hi;       //累计充电能量高位 0.1Wh 0 
    uint16_t TotalBatDSGSOE_Lo;       //累计放电能量低位 0.1Wh 0
    uint16_t TotalBatDSGSOE_Hi;       //累计放电能量高位 0.1Wh 0
    uint16_t Cycle ;                  //循环数 0
    uint16_t Design_Cap;              //设计容量 0.1Ah 0
    uint16_t FCC;                     //满充容量 0.1Ah 0
    uint16_t Remain_Cap;              //剩余容量 0.1Ah 0 R
    uint16_t TotalChgTime;            //剩余充电时间 0.1h 0 R
    uint16_t TotalDsgTime;            //剩余放电时间 0.1h 0 R
    uint16_t DeltaV;                  //压差 1mV 0 R
    uint16_t DeltaT;                  //温差 0.1 ℃ 0 R

    uint16_t InsulatorPlus;
    uint16_t InsulatorMinus;
}Atl485_batd_t;//电池概要数据

typedef struct
{
    uint16_t Run_Mode;      //工作模式1
                            // 0：静置（Idle）
                            // 1：充电（Charge）
                            // 2：放电（Discharge）
                            // 3: 反向充电（Feedback Charge）
                            // 4： 满充（Full Charge）
                            // （电流状态)
    uint16_t State_Mode;    //工作模式2 数值 状态机状态字节
    uint16_t Relay_state;   // Actor(继电器/MOS)状态
                            // Bit0：继电器1/充电MOS
                            // Bit1：继电器2/放电MOS
                            // Bit2：继电器3/预充MOS
                            // Bit3：预留
                            // 0：断开；
                            // 1：闭合；
    uint16_t BMS_ALARM_CODE0_9[10]; //故障1-10
    uint16_t BalanceFlag12[2]; //均衡状态1-32
    uint16_t DigitalSignal;    //开关信号
                                // Bit0： ON信号
                                // Bit1： ACC信号
                                // Bit2： CRG信号
                                // Bit3: KEY信号
                                // 0：信号无效；
                                // 1：信号有效；
    uint16_t McuResetStatus;  //byte0 0x00：The high speed clock is not running；
                                                // 0x01：The high speed clock is running. The clock source is an internal
                                                // RC oscillator；
                                                // 0x02：The high speed clock is running. The clock source is an external
                                                // OSC oscillator；
                                                // 0x03: reserved
                                                // 0x04:
                                                // 0x05:
                                                // 0x06:
                                                // 0x07:
                              //byte1 
                                            // 0x01:Low voltage detect reset;
                                            // 0x02:Loss of clock reset;
                                            // 0x03:Loss of lock reset;
                                            // 0x04:CMU Loss of lock reset;
                                            // 0x05:Watch dog reset;
                                            // 0x06:External pin reset(an active-low level on the external RESET pin);
                                            // 0x07:Power on reset;
                                            // 0x08:JTAG generated reset;
                                            // 0x09:core lockup reset(the ARM core indication of a LOCKUP event);
                                            // 0x0A:Software reset(software setting of SYSRESETREQ bit);
                                            // 0x0B:MDM-AP system reset(host debugger system setting of the System
                                            // Reset Request bit); 


}Atl485_BatState_t;

typedef struct
{
    uint16_t IntSwVersion[2];               //软件内部版本
    uint16_t ExtSwVersion;                  //软件外部版本
    uint16_t ProjectCode[6];                //项目编号
    uint16_t Boot1VerMajor;                 //Boot1版本
    uint16_t Boot2VerMajor;
    uint16_t RSVD;
    uint16_t YearMonth;                     //offset 2000
    uint16_t DayRSVD;
    uint16_t SN[2];
    uint16_t BMS_CellNum;
    uint16_t BMS_CellType;                 //0x00：未配置
                                            // 0x01： NCM三元（4.2V,4.3V等不做区
                                            // 分）
                                            // 0x02： LFP铁锂
                                            // 0x03： LMO锰锂
                                            // 0x04： LCO钴酸锂
                                            // 混合体系根据比例较大的体系来设定
    uint16_t BMS_RatedCap;              //额定容量   0.1Ah
    uint16_t BMS_RateVolt;              //额定电压   0.1V
    uint16_t Product_BarCode[19];       //二维码信息
    uint16_t ManufactureName[4];        //生产商
    uint16_t HW_VERSION_Major;          //硬件版本号
}Atl485_ProjectInfo_t;

typedef struct 
{
    uint16_t MaxCellVol;// 1mV 0
    uint16_t MinCellVol;// 1mV 0
    uint16_t MaxDeltaVol;
    uint16_t MaxBatVol;
    uint16_t MinBatVol;
    uint16_t MaxPackVol;
    uint16_t MinPackVol;
    uint16_t VolRsvd0;
    uint16_t VolRsvd1;
    uint16_t VolRsvd2;
    uint16_t VolRsvd3;
    uint16_t VolRsvd4;
    uint16_t MaxCellTemp;
    uint16_t MinCellTemp;
    uint16_t MaxMosTemp;
    uint16_t MinMosTemp;
    uint16_t MaxBalTemp;
    uint16_t MinBalTemp;
    uint16_t MaxResTemp;
    uint16_t MinResTemp;
    uint16_t TempRsvd0;
    uint16_t TempRsvd1;
    uint16_t TempRsvd2;
    uint16_t TempRsvd3;
    uint16_t TempRsvd4;
    uint16_t TempRsvd5;
    uint16_t  MaxDsgCurrent;
    uint16_t MaxDsgCurPul;
    uint16_t MaxChgCur;
    uint16_t MaxChgCurPul;
    uint16_t MaxFeedBackCur;
    uint16_t MaxFeedBackCurPul;
    uint16_t MaxPdsgCur;
    uint16_t CurRsvd0;
    uint16_t CurRsvd1;
    uint16_t CurRsvd2;
    uint16_t MaxChgPower;
    uint16_t MaxDsgPower;
    uint16_t PowerRsvd0;
    uint16_t PowerRsvd1;
    uint16_t PowerRsvd2;
    uint16_t PowerRsvd3;
    uint16_t MaxChgItvlHour;
    uint16_t LatChgItvlHour;
    uint16_t TaskTime0;
    uint16_t TaskTime1;
    uint16_t TaskTime2;
    uint16_t TaskTime3;
    uint16_t TaskTime4;
    uint16_t TimeRsvd0;
    uint16_t CELL_OV_Times;
    uint16_t CELL_UV_Times;
    uint16_t CELL_UNBA_Times;
    uint16_t TOTAL_OV_Times;
    uint16_t TOTAL_UV_Times;
    uint16_t CHG_CON_OI_Times;
    uint16_t FEEDBACK_CON_OI_Times;
    uint16_t DSG_CON_OI_Times;
    uint16_t PRE_OI_Times;
    uint16_t CHG_OT_Times;
    uint16_t CHG_UT_Times;
    uint16_t CHG_T_UNBA_Times;
    uint16_t DSG_OT_Times;
    uint16_t DSG_UT_Times;
    uint16_t DSG_T_UNBA_Times;
    uint16_t PRE_OT__Times;
    uint16_t AFE_COMM_ERR_Times;

}Atl485_ProjectRunthresd_t;

typedef struct {
    uint16_t	TIME1	           ; //int8_t((Lo)	Year		2000
                                   ; //int8_t(Hi)	Month		
    uint16_t	TIME2	           ; //int8_t((Lo)	Date		
                                   ; //int8_t(Hi)	Hour		
    uint16_t	TIME3	           ; //int8_t((Lo)	Minute		
                                   ; //int8_t(Hi)	Second		
    uint16_t	History_Record_Num;	//unt8_t((Lo)			
                                    //unt8_t(Hi)			
    uint16_t	History_Cur_Num	   ; //int8_t((Lo)			
                                   ; //int8_t(Hi)			
    uint16_t	INT_Vol1	       ; //采集内总压		0.1V	0
    uint16_t	INT_Vol2	       ; //累加内总压		0.1V	
    uint16_t	EXT_Vol	           ; //ack外总压		0.1V	
    uint16_t	MaxVol	           ; //单体最大电压		1 mV	
    uint16_t	MinVol	           ; //单体最小电压		1 mV	
    uint16_t	MaxDeltaVol	       ; //单体压差		1 mV	
    uint16_t	History_Data_SOC	;//		0.10%	
    uint16_t	History_Data_SOH	;//		0.10%	
    uint16_t	PACK_ISO_R_P		;//	1kohm	
    uint16_t	PACK_ISO_R_N		;//	1kohm	
    uint16_t	Cycle	           ; //   循环数			
    uint16_t	Bat_ChargeTimes	   ; //   充电次数			
    uint16_t	TotalChgTime	   ; //   剩余充电时间			
    uint16_t	TotalDsgTime	   ; //   剩余放电时间			
    uint16_t	FCC	               ;;//    满充容量		;    0.1Ah	
    uint16_t	Remain_Cap	       ; //   剩余容量		0.1Ah	
    uint16_t	TotalCHGCapacity_Lo;//	    累计充电容量低位		0.1Ah	
    uint16_t	TotalCHGCapacity_Hi;//	    累计充电容量高位		0.1Ah	
    uint16_t	TotalDSGCapacity_Lo;//	    累计放电容量低位		0.1Ah	
    uint16_t	TotalDSGCapacity_Hi;//	    累计放电容量高位		0.1Ah	
    uint16_t	customdef          ; //            自定义                   ……		……	
    uint16_t	Status	           ; //   晶振状态位		……	
                                     //        MCU System Reset Status	   系统上电复位状态			
    uint16_t	Run_Mode	       ; //       工作模式1			
    uint16_t	State_Mode	       ; //   工作模式2			
;
    uint16_t	Relay_state	;
                                //Actor(继电器/MOS)状态	Bit0：继电器1/充电MOS Bit1：继电器2/放电MOS Bit2：继电器3/预充MOS Bit3：预留	
                                //0：断开；
                                //1：闭合；	

    uint16_t	ndef1;
                        // 自定义	
                        // 开关信号(自定义)	Bit0：ON信号Bit1：ACC信号Bit2：CRG信号	
                        // 0：信号无效；
                        // 1：信号有效；	
    uint16_t	BalanceFlag1;//	均衡状态1	Bit0~Bit15：
                        // 1~16节均衡状态		
    uint16_t	BalanceFlag2;	//均衡状态2	Bit0~Bit15：
                        // 17~32节均衡状态		

    int16_t	    Pack_Current;//    电池组电流	0.1A    30000

    uint16_t	Bat_Max_Allow_Chg_Power1;//允许最大充电功率1 0.01kW	

    uint16_t	Bat_Max_Allow_Dsg_Power1;//允许最大放电功率1 0.01kW	
    uint16_t	ndef2;                  //	自定义			
    uint16_t	BMS_ALARM_CODE0_19[20]; //	故障1			
	
    int16_t	MaxDsgCurr_3S;             //	3s内最大放电电流		0.1A	30000
    int16_t	MaxChgCurr_3S;             //	3s内最大充电电流		0.1A	30000
    int16_t	LastMaxDsgCurr;             //上次记录到本次最大放电电流		0.1A	30000
    int16_t	LastMaxChgCurr;             //上次记录到本次最大充电电流		0.1A	30000
    int16_t	LastMaxCellTemp;         	//上次记录到本次最大电芯温度		0.1 ℃	400
    int16_t	LastMaxChgMosTemp;          //上次记录到本次最大充电MOS温度		0.1 ℃	400
    int16_t	LastMaxDsgMosTemp;       	//上次记录到本次最大放电MOS温度		0.1 ℃	400
    int16_t	LastMaxPdsgMosTemp;      	//上次记录到本次最大预放电阻温度		0.1 ℃	400
    int16_t	LastMaxPcbTemp;             //	上次记录到本次最大PCB温度		0.1 ℃	400
    uint16_t	LastMaxCellVolt;//	上次记录到本次最大单体电压		1mV	
    uint16_t	LastMinCellVolt; //	上次记录到本次最小单体电压		1mV	
    int16_t	Max_Temp;           //	单体最高温度		0.1 ℃	400
    int16_t	Min_Temp;//	单体最低温度		0.1 ℃	400
    int16_t	DeltaT;//	温差		0.1 ℃	
    int16_t	CellTemp1;//	Cell温度1		0.1 ℃	400
    int16_t	CellTemp2;//	Cell温度2		0.1 ℃	400
    int16_t	CellTemp3;//	Cell温度3		0.1 ℃	400
    int16_t	CellTemp4;//	Cell温度4		0.1 ℃	400
    int16_t	CellTemp5;//	Cell温度5		0.1 ℃	400
    int16_t	CellTemp6;//	Cell温度6		0.1 ℃	400
    int16_t	BalTemp1;//	Bal温度1		0.1 ℃	400
    int16_t	BalTemp2;//	Bal温度2		0.1 ℃	400
    int16_t	BalTemp3;//	Bal温度3		0.1 ℃	400
    int16_t	BalTemp4;//	Bal温度4		0.1 ℃	400
    int16_t	MosTemp1;//	Mos温度1		0.1 ℃	400
    int16_t	MosTemp2;//	Mos温度2		0.1 ℃	400
    int16_t	MosTemp3;//	Mos温度3		0.1 ℃	400
    int16_t	MosTemp4;//	Mos温度4		0.1 ℃	400
    int16_t	PCBTemp1;//	PCB温度1		0.1 ℃	400
    int16_t	PCBTemp2;//	PCB温度2		0.1 ℃	400
    int16_t	HeatTemp1;//	加热膜温度1		0.1 ℃	400
    int16_t	HeatTemp2;//	加热膜温度2		0.1 ℃	400
    uint16_t	CellVol1;//	单体电压1		1mV	



}Atl485_BatErrHis_t;

typedef struct {
    uint16_t  Charge_Start_Time[3];    //uint8_t((Lo)	Year		2000	

                                    //充电起始时间
                                    // uint8_t(Hi)	Month			
                                    // uint8_t((Lo)	Date			
                                    // uint8_t(Hi)	Hour			
                                    // uint8_t((Lo)	Minute			
                                    // uint8_t(Hi)	Second			
    uint16_t	Charge_Keep_Time;	//uint8_t(Hi)	Minute			充电时长
                                    //uint8_t((Lo)				
    int16_t	 Average_Charge_Current; //uint8_t((Lo)	Average_Current_L	0.1A	30000	平均充电电流
                                    //uint8_t(Hi)	    Average_Current_H			
    int16_t	 Max_Charge_Current;  	//uint8_t((Lo)	Max_Current_L	0.1A	30000	最大充电电流
                                    //uint8_t(Hi)	    Max_Current_H			
    uint16_t Start_Charge_MaxVol;   //uint8_t((Lo)	Start_Charge_MaxVol_L	1 mV	0	充电前最大单体电压
                                    //uint8_t(Hi)	Start_Charge_MaxVol_H			
    uint16_t Start_Charge_MinVol;	//uint8_t((Lo)	Start_Charge_MinVol_L	1 mV	0	充电前最小单体电压
                                    //uint8_t(Hi)	Start_Charge_MinVol_H			
    uint16_t  Stop_Charge_MaxVol;	//uint8_t((Lo)	Stop_Charge_MaxVol_L	1 mV	0	充电结束最大单体电压
                                    //uint8_t(Hi)	Stop_Charge_MaxVol_H			
    uint16_t  Stop_Charge_MinVol;	//uint8_t((Lo)	Stop_Charge_MinVol_L	1 mV	0	充电结束最小单体电压
                                    //uint8_t(Hi)	Stop_Charge_MinVol_H			
    uint16_t	Start_Charge_SOC;	//uint8_t((Lo)	Start_Charge_SOC_L	0.10%	0	充电前SOC
                                    //uint8_t(Hi)	Start_Charge_SOC_H			
    uint16_t	Stop_Charge_SOC;    //uint8_t((Lo)	Stop_Charge_SOC_L	0.10%	0	充电结束SOC
                                    //uint8_t(Hi)	Stop_Charge_SOC_H			
    uint16_t	Start_Charge_Pack_Volt;//	uint8_t((Lo)	Start_Charge_Pack_Volt_L	0.1V	0	充电前电池总压
                                            //uint8_t(Hi)	Start_Charge_Pack_Volt_H			
    uint16_t	Stop_Charge_Pack_Volt;	//uint8_t((Lo)	Stop_Charge_Pack_Volt_L	0.1V	0	充电结束电池总压
                                        //uint8_t(Hi)	Stop_Charge_Pack_Volt_H			
    uint16_t	rev;                    //自定义		预留，			自定义


}Atl485_ChgHis_t;//充电履历


typedef struct {
    uint16_t	TIME1;  //uint8_t((Lo)	Year		2000
                        //记录时间
                        //uint8_t(Hi)	Month		0	
    uint16_t	TIME2;	//uint8_t((Lo)	Date		0	
                        // uint8_t(Hi)	Hour		0	
    uint16_t	TIME3;	//uint8_t((Lo)	Minute		0	
                        //uint8_t(Hi)	Second		0	
    uint16_t	TotalRecordNum;				//0	记录总条数
    uint16_t	CurRecordNum;				//0	当前读取编号
    uint16_t	Cur;	                    //0~6553.5 A		0.1A	0	充电电流
    uint16_t	Temp;	                    //-40℃~125℃		0.1℃	40℃	电芯温度
    uint16_t	CellVolt;	                //0~5000 mV		1mV	0	单体电芯电压
    uint16_t	CalcRes;	                //0~65535 Ohm		1Ohm	0	计算阻值
}Atl485_F099HIS_t;

typedef enum
{
    onebusmode,
    atlmode
}uart_mode;


#define ATl_OneBusModbusSend(a,b) Uart2SendData(a,b)
#define ATl_ModbusSend(a,b) Uart1SendData(a,b)

void ATL_ModbusRecvHandle(uint8_t rdata);
void ATLModbusPoll(void);
void ATLOneBusModbusPoll(void);
uint16_t get_atl485_bat_vol_cell(uint8_t num);
uint16_t get_atl485_bat_vol_dec(void);
uint16_t get_atl485_bat_soh(void);
uint16_t get_atl485_bat_soc(void);
uint16_t get_atl485_bat_min_temp(void);
uint16_t get_atl485_bat_max_temp(void);
uint16_t get_atl485_bat_max_dsg_cur(void);
uint16_t get_atl485_bat_max_ch_cur(void);
uint8_t get_atl485_bat_type(void);
uint16_t get_atl485_bat_min_cell_vol(void);
uint16_t get_atl485_bat_max_cell_vol(void);
uint16_t get_atl485_bat_max_ext_vol(void);
uint16_t get_atl485_bat_max_cap_vol(void);
uint16_t get_atl485_bat_max_vol(void);
uint8_t get_atl485_bat_sta(void);
void ATLModbusSendSlient(void);
#endif