#ifndef _DXBT_H
#define _DXBT_H

#include "includes.h"

#define DXBT24_UARTX 1

#define BLE_NAME  "XNA"	//定义蓝牙名称
//=============================================================================
//接收数据帧值
//=============================================================================
#define BT24_RX_FIRST 0x5F
#define BT24_RX_ADDRH 0x02
#define BT24_RX_ADDRL 0x5A
//=============================================================================
// BT帧的字节顺序
//=============================================================================
#define BT24_FRAME_FIRST 			0
#define BT24_FRAME_ADDRH 			1
#define BT24_FRAME_ADDRL 			2
#define BT24_FRAME_CMDTYPE			3
#define BT24_FRAME_LENGTH			4
#define BT24_FRAME_DATATYPE			5
#define BT24_FRAME_STATE			6
#define BT24_FRAME_FAULT			7
#define BT24_FRAME_RESULT			8

#define BT24_SET_OPERA_STA          5
#define BT24_SET_DSG_CUR            6
#define BT24_SET_DSG_END_VOL        8
#define BT24_SET_CH_CUR             12
#define BT24_SET_CH_END_VOL         14
#define BT24_SET_DataType           18
//=============================================================================
//接收数据帧类型
//=============================================================================
#define         APP_CHECK_CMD         		0x80    	//检测校验
#define         DEVICE_PARAM_CMD            0x0A   		//电池状态
#define         START_DC_CMD                0x91       	//开始放电                       	
#define         GET_DC_CMD                 	0x0B       	//得到放电结果
#define         GET_CODE_CMD                0x0C       	//得到放电结果
#define         GET_FW_CMD                  0x01       	//固件升级
#define         FW_UPDATA_CHECK             0x02        //推送新版本检验
#define         FW_UPDATA_CONTENT           0x03        //升级内容
#define         FW_UPDATA_OVER              0x04        //推送结束
#define         FW_UPDATA_RESULT            0x05        //查询升级结果
//=============================================================================
//发送数据帧值
//=============================================================================
#define         BT24_TX_FIRST         		0xF5    	
#define         BT24_TX_ADDRH            	0x5A   		//电池高位
#define         BT24_TX_ADDRL             	0x02       	//地址地位                       	
#define         BT24_TX_CHECK_CMD           0x80       	//检测校验指令
#define         BT24_TX_DP_CMD              0x0A       	//获取静态数据指令
#define         BT24_TX_DC_CMD              0x91       	//放电指令
#define         BT24_TX_GET_CMD             0x0B       	//询问放电结果指令

//=============================================================================
//回复检测校验命令
//=============================================================================
typedef struct
{
    uint8_t DataType;                       //数据类型
    uint8_t Fault_Code;                       //错误代码
	uint8_t CELL_OV;							//单体过压  4位
    uint8_t CELL_UV;                             //单体欠压 4位
    uint8_t CELL_UNBA;                      //压差          4位
    uint8_t TOTAL_OV;                      //总压过高       4位
    uint8_t TOTAL_UV;                      //总压过低       4位
    uint8_t CHG_CON_OI;                      //充电持续电流 4位
    uint8_t CHG_PUL_OI;                      //充电瞬时电流 4位
    uint8_t FEEDBACK_CON_OI;                      //回馈持续电流 4位
    uint8_t FEEDBACK_PUL_OI;                      //回馈瞬时电流 4位
    uint8_t DSG_CON_OI;                      //放电持续电流 4位
    uint8_t DSG_PUL_OI;                      //放电瞬时电流 4位
    uint8_t PRE_OI;                      //预充过流 4位
    uint8_t CHG_POWER;                      //充电功率过高 4位
    uint8_t DSG_POWER;                      //放电功率过高 4位
    uint8_t HEAT_POWER;                      //加热功率过高 4位
    uint8_t CHG_OT;                      //充电高温 4位
    uint8_t CHG_UT;                      //充电低温 4位
    uint8_t CHG_T_UNBA;                      //充电温差过大 4位
    uint8_t DSG_OT;                      //放电过温 4位
    uint8_t DSG_UT;                      //放电低温 4位
    uint8_t SG_T_UNBA;                      //放电温差过大 4位
    uint8_t T_RISE_FAST;                      //电芯温升过快 4位
    uint8_t CELL_OR;                      //单体内阻过高 4位
    uint8_t CELL_SHORT;                      //单体短路 4位
    uint8_t CELL_0V;                      //单体0V 4位
    uint8_t CELL_OV_SEVERE;                      //严重过压 4位
    uint8_t SOC_LOW;                      //SOC低 4位
    uint8_t SOC_HIGH;                      //SOC高 4位
    uint8_t SOH_LOW;                      //SOH低 4位
    uint8_t FC;                      //浮充 4位
    uint8_t AFE_OV;                      //AFE判断过压 4位
    uint8_t AFE_UV;                      //AFE判断欠压 4位
    uint8_t AFE_OT;                      //AFE判断过温 4位
    uint8_t AFE_UT;                      //AFE判断欠温 4位
    uint8_t AFE_OI;                      //AFE判断过流 4位
    uint8_t BAL_OT;                      //均衡过温 4位
    uint8_t HEATER_OT;                      //加热膜过温 4位
    uint8_t HEAT_SLOW;                      //电芯加热过慢 4位
    uint8_t MOS1_OT;                      //MOS1过温(充电)
    uint8_t MOS2_OT;                      //MOS2过温(放电)
    uint8_t MOS3_OT;                      //MOS3过温(预电)
    uint8_t MOS4_OT;                      //MOS4过温
    uint8_t MOS5_OT;                      //MOS5过温
    uint8_t MOS6_OT;                      //MOS6过温
    uint8_t PRE_OT;                      //预充过温
    uint8_t PRE_OTIME;                      //预充超时
    uint8_t PRE_FAST;                      //预充过快
    uint8_t RESERVE1_OT;                      //预留过温
    uint8_t RESERVE2_OT;                      //预留过温
    uint8_t RESERVE3_OT;                      //预留过温
    uint8_t RESERVE4_OT;                      //预留过温
    uint8_t RESERVE5_OT;                      //预留过温
    uint8_t RESERVE6_OT;                      //预留过温
    uint8_t FUSE_OT;                      //FUSE过温
    uint8_t ISO_UR;                      //绝缘过低
    uint8_t CHG_OTIME;                      //充电过时
    uint8_t CHARGER;                      //充电器故障
    uint8_t CHG_VERSION;                      //充电版本
    uint8_t CHG_RESERVE;                      //充电预留
    uint8_t VCU_CMD;                      //VCU命令
    uint8_t DSG_RESERVE;                      //放电预留
    uint8_t COMM_INSTRUMENT;                      //仪表通讯
    uint8_t COMM_CHG;                      //充电通讯
    uint8_t COMM_VCU;                      //VCU通讯
    uint8_t COMM_INT;                      //内部通讯
    uint8_t SHORT;                      //短路
    uint8_t EOL_OVER;                      //EOL寿命锁止
    uint8_t SLEEP;                      //休眠模式
    uint8_t SHUT;                      //低电量关机
    uint8_t RESERVE_0;                      //预留_0，数值为0
    uint8_t RESERVE_1;                      //预留_1，数值为0
    uint8_t RESERVE_2;                      //预留_2，数值为0
    uint8_t RESERVE_3;                      //预留_3，数值为0
    uint8_t RESERVE_4;                      //预留_4，数值为0
    uint8_t RESERVE_5;                      //预留_5，数值为0
    uint8_t RESERVE_6;                      //预留_6，数值为0
    uint8_t RESERVE_7;                      //预留_7，数值为0
    uint8_t VCELL_OPEN;                      //单体电压采集断线
    uint8_t VCELL_SHORT;                      //单体电压采集短路
    uint8_t VCELL_DRIFT;                      //单体电压漂移
    uint8_t T_OPEN;                      //温度断线
    uint8_t T_SHORT;                      //温度短路
    uint8_t T_DRIFR;                      //温度漂移
    uint8_t CUTRRENT_OPEN;                      //电流检测断线
    uint8_t CUTRRENT_SHORT;                      //电流检测短路
    uint8_t CUTRRENT_DRIFT;                      //电流漂移
    uint8_t VSUM_OPEN;                      //总压检测开路
    uint8_t VSUM_SHORT;                      //总压检测短路
    uint8_t VSUM_DRIFT;                      //总压漂移
    uint8_t T_MCU_OPEN;                      //MCU检测温度开路
    uint8_t T_MCU_SHORT;                      //MCU检测温度短路
    uint8_t ISO;                      //绝缘电路异常
    uint8_t POSACTOR_DRIVE;                      //正极ACTOR不能驱动(MOS控制失效)
    uint8_t NEGACTOR_DRIVE;                      //负极ACTOR不能驱动
    uint8_t PREACTOR_DRIVE;                      //加热ACTOR不能驱动
    uint8_t HEATACTOR_DRIVE;                      //加热ACTOR不能驱动
    uint8_t ACTOR5_DRIVE;                      //ACTOR5不能驱动
    uint8_t ACTOR6_DRIVE;                      //ACTOR6不能驱动
    uint8_t ACTOR7_DRIVE;                      //ACTOR7不能驱动
    uint8_t ACTOR8_DRIVE;                      //ACTOR8不能驱动
    uint8_t ACTOR9_DRIVE;                      //ACTOR9不能驱动
    uint8_t ACTOR10_DRIVE;                      //ACTOR10不能驱动
    uint8_t POSACTOR_ADHESION;                      //正极ACTOR粘连
    uint8_t NEGACTOR_ADHESION;                      //负极ACTOR粘连
    uint8_t PREACTOR_ADHESION;                      //预充ACTOR粘连
    uint8_t HEATACTOR_ADHESION;                      //加热ACTOR粘连
    uint8_t ACTOR5_ADHESION;                      //ACTOR5粘连
    uint8_t ACTOR6_ADHESION;                      //ACTOR6粘连
    uint8_t ACTOR7_ADHESION;                      //ACTOR7粘连
    uint8_t ACTOR8_ADHESION;                      //ACTOR8粘连
    uint8_t ACTOR9_ADHESION;                      //ACTOR9粘连
    uint8_t ACTOR10_ADHESION;                      //ACTOR10粘连
    uint8_t FAST_HEATER;                      //加热膜温升过快
    uint8_t SLOW_HEATER;                      //加热膜温升慢
    uint8_t RESERVE_COOL;                      //制冷故障
    uint8_t BAL_SHORT;                      //均衡短路
    uint8_t BAL_OPEN;                      //均衡断线
    uint8_t EEPROM_COMM;                      //EEPROM通讯
    uint8_t EEPROM_FAULT;                      //EEPROM故障
    uint8_t RTC_COMM;                      //RTC通讯
    uint8_t RTC_FAULT;                      //RTC故障
    uint8_t AFE_COMM;                      //AFE通讯
    uint8_t AFE_FAULT;                      //AFE故障
    uint8_t DOG_COMM;                      //看门狗通讯
    uint8_t DOG_FAULT;                      //看门狗故障
    uint8_t COMM_CAN;                      //CAN通讯
    uint8_t COMM_485;                      //485通讯
    uint8_t POWER_BASE_HIGH;                      //基准电压高
    uint8_t POWER_BASE_LOW;                      //基准电压低
    uint8_t POWER_12V_HIGH;                      //12V高
    uint8_t POWER_12V_LOW;                      //12V低
    uint8_t POWER_5V_HIGH;                      //5V高（5V故障，包括高低）
    uint8_t POWER_5V_LOW;                      //5V低
    uint8_t POWER_3_3V_HIGH;                      //3.3V高
    uint8_t POWER_3_3V_LOW;                      //3.3V低
    uint8_t FUSE;                      //FUSE
    uint8_t MCU_CORE;                      //MCU内核
    uint8_t MCU_RAM;                      //MCU RAM
    uint8_t MCU_FLASH;                      //MCU Flash
    uint8_t MCU_INTERRUPT;                      //MCU中断
    uint8_t MCU_POWER;                      //MCU电源
    uint8_t MCU_AD;                      //MCU电压
    uint8_t MCU_CLOCK;                      //MCU 时钟
    uint8_t MCU_GPIO;                      //MCU GPIO
    uint8_t MCU_EEPROM;                      //MCU EEPROM
    uint8_t MCU_BUS;                      //MCU 总线
    uint8_t RESERVE_8;                      //预留_8，数值为0
    uint8_t RESERVE_9;                      //预留_9，数值为0
    uint8_t RESERVE_10;                      //预留_10，数值为0
    uint8_t RESERVE_11;                      //预留_11，数值为0
    uint8_t RESERVE_12;                      //预留_12，数值为0
    uint8_t RESERVE_13;                      //预留_13，数值为0
    uint8_t RESERVE_14;                      //预留_14，数值为0
    uint8_t CRC_CHECK;                      //CRC校验
}BleResponseCheckCmdErr_t;
//=============================================================================
//回复放电/静态数据/放电结果
//=============================================================================
typedef struct 
{
    uint8_t  DataType;                       //数据类型
    uint8_t  DSG_Result;                      //执行命令
    uint16_t CellVolt1;                      //电芯电压1
    uint16_t CellVolt2;                      //电芯电压2
    uint16_t CellVolt3;                      //电芯电压3
    uint16_t CellVolt4;                      //电芯电压4
    uint16_t CellVolt5;                      //电芯电压5
    uint16_t CellVolt6;                      //电芯电压6
    uint16_t CellVolt7;                      //电芯电压7
    uint16_t CellVolt8;                      //电芯电压8
    uint16_t CellVolt9;                      //电芯电压9
    uint16_t CellVolt10;                      //电芯电压10
    uint16_t CellVolt11;                      //电芯电压11
    uint16_t CellVolt12;                      //电芯电压12
    uint16_t CellVolt13;                      //电芯电压13
    uint16_t Current;                      //回路电流，充电为正，放电为负
    uint16_t MaxCellVolt;                      //最大电芯电压
    uint16_t MinCellVolt;                      //最小电芯电压
    uint16_t MaxCellDeltV;                      //最大电芯压差
    uint16_t MaxCellTemp;                      //最大电芯温度
    uint16_t MinCellTemp;                      //最小电芯温度
    uint16_t CellDeltVolt;                      //电芯电压压差
    uint16_t BoardTemp;                      //板子温度
    uint16_t MosTemp;                      //Mos管温度
    uint16_t BattSumVolt;                      //电池总压(电芯电压累加值)
    uint16_t BattVolt;                      //电池总压(检测值)
    uint16_t PackVolt;                      //外总压
    uint16_t SOC;                      //电池总压(检测值)
    uint16_t SOH;                      //电池总压(检测值)
    uint16_t MaxChgCurr;                      //最大充电电流
    uint16_t MaxDsgCurr;                      //最大放电电流
    uint8_t FaultCode;                       //故障码
    uint8_t CRC_CHECK;                      //CRC校验
}BleResponseErr_t;

//=============================================================================
//BLE发送命令结构体
//=============================================================================
typedef struct 
{
    uint8_t  OPERA_STA; //操作模式
    uint16_t DSG_CUR;
    uint32_t DSG_END_VOL;
    uint16_t CH_CUR;
    uint32_t CH_END_VOL;
    uint32_t DataType;
}BleSendErr_t;

//=============================================================================
//BLE设备名称结构体
//=============================================================================
typedef struct 
{
    uint8_t BLE_MARK;
    uint8_t BLE_BAUD_FLAG;
    uint8_t BLE_ID_FLAG;
    uint8_t BLE_ID_HH;
    uint8_t BLE_ID_HL;
    uint8_t BLE_ID_LH;
    uint8_t BLE_ID_LL;
}BleDeviceBuffErr_t;
 
typedef union
{
	uint8_t BLE_ID_BUFF[6];
	BleDeviceBuffErr_t BDB;
}BleDeviceNameErr_t;

void DX_BT24_Init(void);
uint8_t bt24_get_bat_status(void);
uint8_t bt24_get_bat_core(void);
uint8_t bt24_get_bat_spec(void);
uint8_t bt24_get_bat_cap(void);
uint8_t bt24_get_bat_sn(void);
uint8_t bt24_get_bat_opera_status(void);
uint8_t bt24_get_bat_det_mode(void);
uint16_t bt24_get_bat_disch_cur(void);
uint32_t bt24_get_bat_disch_end_vol(void);
uint16_t bt24_get_bat_ch_cur(void);
uint32_t bt24_get_bat_ch_end_vol(void);
uint8_t bt24_get_bat_ch_end_cur(void);
uint8_t bt24_get_bat_set_mode(void);
void bt24_receive_input(uint8_t value);
uint8_t bt24_get_bat_type(void);
void bt24_protocol_init(void);
uint8_t DXBT24_AT_Init(uint8_t *name,uint8_t name_len);
void bt24_recv_service(void);
#endif