#ifndef _ATLBLEPROTOCOL_H
#define _ATLBLEPROTOCOL_H

#include "includes.h"
//检测校验回复
typedef struct 
{

}BleResponseHardVerJcErr_t;


//纯硬件版本回复
typedef struct 
{

}BleResponseHardVerJcErr_t;

typedef struct
{
//     uint8_t SOF;		//1	0xF5	　
//     uint16_t Address;	//2	0x5A02	　
// 	uint8_t	CMD ;       //1	0x80	　
//     uint8_t Length;		//1	0x4F	　
//     uint8_t DataType;	//1	数据类型：
//                         // 电池
//                         // 0x01：纯硬件版本
//                         // 0x02：一线通版本
// 	uint8_t	Status;		        //1	0x00	接收数据校验ok
// uint8_t CELL_OV_UV;             //0.5	　	单体过压
//                                 //CELL_UV		0.5	　	单体欠压
// uint8_t CELL_UNBA_TOTAL_OV;		//0.5	　	压差
// 		                        //0.5	　	总压过高
// uint8_t TOTAL_UV_CHG_CON_OI;		//0.5	　	总压过低
// 		                        //0.5	　	充电持续过流
// uint8_t CHG_PUL_OI_FEEDBACK_CON_OI;		//0.5	　	充电瞬时过流
// 		                                //0.5	　	回馈持续过流
// uint8_t FEEDBACK_PUL_OI_DSG_CON_OI		//0.5	　	回馈瞬时过流
// 		                                //0.5	　	放电持续过流
// DSG_PUL_OI	uint8_t	0.5	　	放电瞬时过流
// PRE_OI		0.5	　	预充过流
// CHG_POWER	uint8_t	0.5	　	充电功率过高
// DSG_POWER		　0.5	　	放电功率过高
// HEAT_POWER	uint8_t	0.5	　	加热功率过高
// CHG_OT		0.5	　	充电高温
// CHG_UT	uint8_t	0.5	　	充电低温
// CHG_T_UNBA		0.5	　	充电温差过大
// DSG_OT	uint8_t	0.5	　	放电过温
// DSG_UT		0.5	　	放电低温
// DSG_T_UNBA	uint8_t	　0.5	　	放电温差过大
// T_RISE_FAST		　0.5	　	电芯温升过快
// CELL_OR	uint8_t	0.5	　	单体内阻过高
// CELL_SHORT		0.5	　	单体短路
// CELL_0V	uint8_t	0.5	　	单体0V (LOW_VOL_LOCKPACK)
// CELL_OV_SEVERE		0.5	　	单体严重过压
// SOC_LOW	　	0.5	　	SOC低
// SOC_HIGH		0.5	　	SOC高
// SOH_LOW	uint8_t	0.5	　	SOH低
// FC		0.5	　	浮充
// RESERVE	uint8_t	0.5	　	预留（用0填充，保证字节完整）
// 　		0.5	　	　
// AFE_OV	uint8_t	0.5	　	AFE判断过压
// AFE_UV		0.5	　	AFE判断欠压
// AFE_OT	uint8_t	0.5	　	AFE判断过温
// AFE_UT		0.5	　	AFE判断欠温
// AFE_OI	uint8_t	0.5	　	AFE判断过流
// BAL_OT		0.5	　	均衡过温
// HEATER_OT	uint8_t	0.5	　	加热膜过温
// HEAT_SLOW		0.5	　	电芯加热过慢
// MOS1_OT	uint8_t	0.5	　	MOS1过温(充电）
// MOS2_OT		0.5	　	MOS2过温(放电)
// MOS3_OT	uint8_t	0.5	　	MOS3过温(预放)
// MOS4_OT		0.5	　	MOS4过温
// MOS5_OT	uint8_t	0.5	　	MOS5过温
// MOS6_OT		0.5	　	MOS6过温
// PRE_OT	uint8_t	0.5	　	预充过温
// PRE_OTIME		0.5	　	预充超时
// PRE_FAST	uint8_t	0.5	　	预充过快
// RESERVE1_OT		0.5	　	预留过温
// RESERVE2_OT	uint8_t	0.5	　	预留过温
// RESERVE3_OT		0.5	　	预留过温
// RESERVE4_OT	uint8_t	0.5	　	预留过温
// RESERVE5_OT		0.5	　	预留过温
// RESERVE6_OT	uint8_t	0.5	　	预留过温
// FUSE_OT		0.5	　	FUSE过温
// ISO_UR	uint8_t	0.5	　	绝缘过低
// CHG_OTIME		0.5	　	充电过时
// CHARGER	uint8_t	0.5	　	充电器故障
// CHG_VERSION		0.5	　	充电版本
// CHG_RESERVE	uint8_t	0.5	　	充电预留
// VCU_CMD		0.5	　	VCU命令
// DSG_RESERVE	uint8_t	0.5	　	放电预留
// COMM_INSTRUMENT		0.5	　	仪表通讯
// COMM_CHG	uint8_t	0.5	　	充电通讯
// COMM_VCU		0.5	　	VCU通讯
// COMM_INT	uint8_t	0.5	　	内部通讯
// SHORT		0.5	　	短路
// BATTERY_LEAKAGE	uint8_t	0.5	　	电芯漏液
// EOL_OVER		0.5	　	EOL寿命锁止
// SLEEP	uint8_t	0.5	　	休眠模式
// SHUT		0.5	　	低电量关机
// RESERVE_0	uint8_t	0.5	　	预留_0，数值为0
// RESERVE_1		0.5	　	预留_1，数值为0
// RESERVE_2	uint8_t	0.5	　	预留_2，数值为0
// RESERVE_3		0.5	　	预留_3，数值为0
// RESERVE_4	uint8_t	0.5	　	预留_4，数值为0
// RESERVE_5		0.5	　	预留_5，数值为0
// RESERVE_6	uint8_t	0.5	　	预留_6，数值为0
// RESERVE_7		0.5	　	预留_7，数值为0
// VCELL_OPEN	uint8_t	0.5	　	单体电压采集断线
// VCELL_SHORT		0.5	　	单体电压采集短路
// VCELL_DRIFT	uint8_t	0.5	　	单体电压漂移
// T_OPEN		0.5	　	温度断线
// T_SHORT	uint8_t	0.5	　	温度短路
// T_DRIFR		0.5	　	温度漂移
// CUTRRENT_OPEN	uint8_t	0.5	　	电流检测断线
// CUTRRENT_SHORT		0.5	　	电流检测短路
// CUTRRENT_DRIFT	uint8_t	0.5	　	电流漂移
// VSUM_OPEN		0.5	　	总压检测开路
// VSUM_SHORT	uint8_t	0.5	　	总压检测短路
// VSUM_DRIFT		0.5	　	总压漂移
// T_MCU_OPEN	uint8_t	0.5	　	MCU检测温度开路
// T_MCU_SHORT		0.5	　	MCU检测温度短路
// ISO	uint8_t	0.5	　	绝缘电路异常
// POSACTOR_DRIVE		0.5	　	正极ACTOR不能驱动(MOS控制失效)
// NEGACTOR_DRIVE	uint8_t	0.5	　	负极ACTOR不能驱动
// PREACTOR_DRIVE		0.5	　	加热ACTOR不能驱动
// HEATACTOR_DRIVE	uint8_t	0.5	　	加热ACTOR不能驱动
// ACTOR5_DRIVE		0.5	　	ACTOR5不能驱动
// ACTOR6_DRIVE	uint8_t	0.5	　	ACTOR6不能驱动
// ACTOR7_DRIVE		0.5	　	ACTOR7不能驱动
// ACTOR8_DRIVE	uint8_t	0.5	　	ACTOR8不能驱动
// ACTOR9_DRIVE		0.5	　	ACTOR9不能驱动
// #VALUE!	uint8_t	0.5	　	ACTOR10不能驱动
// POSACTOR_ADHESION		0.5	　	正极ACTOR粘连
// NEGACTOR_ADHESION	uint8_t	0.5	　	负极ACTOR粘连
// PREACTOR_ADHESION		0.5	　	预充ACTOR粘连
// HEATACTOR_ADHESION	uint8_t	0.5	　	加热ACTOR粘连
// ACTOR5_ADHESION		0.5	　	ACTOR5粘连
// ACTOR6_ADHESION	uint8_t	0.5	　	ACTOR6粘连
// ACTOR7_ADHESION		0.5	　	ACTOR7粘连
// ACTOR8_ADHESION	uint8_t	0.5	　	ACTOR8粘连
// ACTOR9_ADHESION		0.5	　	ACTOR9粘连
// ACTOR10_ADHESION	uint8_t	0.5	　	ACTOR10粘连
// FAST_HEATER		0.5	　	加热膜温升过快
// SLOW_HEATER	uint8_t	0.5	　	加热膜温升慢
// RESERVE_COOL		0.5	　	制冷故障
// BAL_SHORT	uint8_t	0.5	　	均衡短路
// BAL_OPEN		0.5	　	均衡断线
// EEPROM_COMM	uint8_t	0.5	　	EEPROM通讯
// EEPROM_FAULT		0.5	　	EEPROM故障
// RTC_COMM	uint8_t	0.5	　	RTC通讯
// RTC_FAULT		0.5	　	RTC故障
// AFE_COMM	uint8_t	0.5	　	AFE通讯
// AFE_FAULT		0.5	　	AFE故障
// DOG_COMM	uint8_t	0.5	　	看门狗通讯
// DOG_FAULT		0.5	　	看门狗故障
// COMM_CAN	uint8_t	0.5	　	CAN通讯
// COMM_485		0.5	　	485通讯
// POWER_BASE_HIGH	uint8_t	0.5	　	基准电压高
// POWER_BASE_LOW		0.5	　	基准电压低
// POWER_12V_HIGH	uint8_t	0.5	　	12V高
// POWER_12V_LOW		0.5	　	12V低
// POWER_5V_HIGH	uint8_t	0.5	　	5V高（5V故障，包括高低）
// POWER_5V_LOW		0.5	　	5V低
// POWER_3_3V_HIGH	uint8_t	0.5	　	3.3V高
// POWER_3_3V_LOW		0.5	　	3.3V低
// FUSE	uint8_t	0.5	　	FUSE
// MCU_CORE		0.5	　	MCU内核
// MCU_RAM	uint8_t	0.5	　	MCU RAM
// MCU_FLASH		0.5	　	MCU Flash
// MCU_INTERRUPT	uint8_t	0.5	　	MCU中断
// MCU_POWER		0.5	　	MCU电源
// MCU_AD	uint8_t	0.5	　	MCU电压
// MCU_CLOCK		0.5	　	MCU时钟
// MCU_GPIO	uint8_t	0.5	　	MCU GPIO
// MCU_EEPROM		0.5	　	MCU EEPROM
// MCU_BUS	uint8_t	0.5	　	MCU总线
// RESERVE_8		0.5	　0	预留_8，数值为0
// RESERVE_9	uint8_t	0.5	　0	预留_9，数值为0
// RESERVE_10		0.5	　0	预留_10，数值为0
// RESERVE_11	uint8_t	0.5	　0	预留_11，数值为0
// RESERVE_12		0.5	　0	预留_12，数值为0
// RESERVE_13	uint8_t	0.5	　0	预留_13，数值为0
// RESERVE_14		0.5	　0	预留_14，数值为0


};

typedef struct
{
    // SOF	uint8_t	1	0xF5	/	
    // Address	uint16_t	2	0x5A02	/	
    // CMD	uint8_t	1	0x0A	/	
    // Length	uint8_t	1	0x3A	/	
    // Status	uint8_t	1	0x00		接收数据校验ok
    // DataType	uint8_t	1	0x02		数据类型：一线通版本数据
    // CellVolt1	uint16_t	2		mV	电芯电压1
    // CellVolt2	uint16_t	2		mV	电芯电压2
    
    // CellVolt13	uint16_t	2		mV	电芯电压13
    // Current	int16_t	2		0.1A	回路电流，充电为正，放电为负
    // MaxCellVolt	uint16_t	2		mV	最大电芯电压
    // MinCellVolt	uint16_t	2		mV	最小电芯电压
    // MaxCellDeltV	uint16_t	2		mV	最大电芯压差
    // MaxCellTemp	int16_t	2		0.1℃	最大电芯温度
    // MinCellTemp	int16_t	2		0.1℃	最小电芯温度
    // BattSumVolt	uint16_t	2		0.1V	电池总压（电芯电压累加值）
    // BattVolt	uint16_t	2		0.1V	电池总压（检测值）
    // PackVolt	uint16_t	2		0.1V	外总压
    // SOC	uint16_t	2		0.1%	
    // SOH	uint16_t	2		0.1%	
    // MaxChgCurr	uint16_t	2		0.1A	最大充电电流
    // MaxDsgCurr	uint16_t	2		0.1A	最大放电电流
    // FaultCode	uint32_t	4			故障码（检测校验处已发送）：
    // Crc8	uint8_t	1			

}BleResponseStaticData_t;



#endif
