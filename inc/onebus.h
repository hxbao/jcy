/******************************************************************************
 * @file           : onebus.h
 * @version        : v1.0
 * @author         : Azreal
 * @creat          : 2021 0804
 ******************************************************************************/

#ifndef __ONEBUS_H
#define __ONEBUS_H

#include "includes.h"

#define ONE_RXD1_CLK RCC_APB2Periph_GPIOA
#define ONE_RXD1_PIN GPIO_PIN_3
#define ONE_RXD1_PORT GPIOA

#define ONE_RXD2_CLK RCC_APB2Periph_GPIOA
#define ONE_RXD2_PIN GPIO_PIN_5
#define ONE_RXD2_PORT GPIOA

#define ONE_TXD1_CLK RCC_APB2Periph_GPIOA
#define ONE_TXD1_PIN GPIO_PIN_2
#define ONE_TXD1_PORT GPIOA

#define ONE_TXD2_CLK RCC_APB2Periph_GPIOA
#define ONE_TXD2_PIN GPIO_PIN_4
#define ONE_TXD2_PORT GPIOA

#define TEST_IO_CLK RCC_APB2Periph_GPIOB
#define TEST_IO_PIN GPIO_PIN_6
#define TEST_IO_PORT GPIOA

#define ACC_IO_CLK RCC_APB2Periph_GPIOA
#define ACC_IO_PIN GPIO_PIN_8
#define ACC_IO_PORT GPIOA
/*********************************************************
接收状态
*********************************************************/
#define bmsreset 0
#define sync1 1
#define sync2 2
#define bms_h 3
#define bms_l 4
/*********************************************************
协议同步头
*********************************************************/
#define pa_syncl_short 85
#define pa_syncl_long 115
#define pa_synch_short 8
#define pa_synch_long 13

typedef struct
{
	int bmscount;
	uint8_t Receflag;
	uint16_t Bytecount;
	uint16_t Bitcount;
	uint8_t Syncflage;
	uint8_t Sflage;
	uint8_t bmstate;
	uint16_t Synch;
	uint16_t Syncl;
	uint8_t buff0;
	uint8_t buff1;
	uint8_t buff2;
	uint8_t buff3;
	uint8_t bmstype;
	uint8_t pb_list[100];
} bms_info;

typedef struct
{
	uint8_t MSG_SOF;			 //报文ID 8位
	uint8_t SEC_PRTL;			 //次级协议   4位
	uint8_t FIR_PRTL;			 //主要通讯协议	4位
	uint8_t BAT_STATUS;			 //电池状态	4位
	uint8_t BAT_CH_STATUS;		 //电池充电状态 3位
	uint8_t BAT_SOC_LEFT;		 //电池剩余SOC
	uint16_t BAT_CH_MAX_VOL;	 //电池最大允许充电电压
	uint16_t BAT_CH_MAX_CUR;	 //电池最大允许充电电流
	uint16_t BAT_CUR_FB;		 //电池最大允许回馈电流
	uint16_t BAT_DIS_MAX_CUR;	 //电池最大允许放电电流
	uint16_t BAT_TOTAL_CH_TIME;	 //电池电量充满剩余时间
	uint16_t BAT_TOTAL_VOL;		 //电池总电压
	uint16_t BAT_TOTAL_CUR;		 //电池总电流
	uint8_t BAT_DIS_MOS_STA;	 //放电MOS状态   2位
	uint8_t BAT_CH_MOS_STA;		 //充电MOS状态	2位
	uint8_t BAT_PRE_DIS_MOS_STA; //预放电MOS状态	2位
	uint16_t BAT_SOE_LEFT;		 //电池剩余能量
	uint16_t BAT_CIR_TIME;		 //循环次数
	uint8_t BAT_HEALTH_STA;		 //电池健康度
	uint8_t BAT_CELL_TEMP_MAX;	 //电池单体最高温度
	uint8_t BAT_CELL_TEMP_MIN;	 //电池单体最低温度
	uint8_t BAT_MOS_TEMP_MAX;	 //电池MOS最低温度
	uint16_t BAT_CELL_VOL_MAX;	 //电池单体最高电压
	uint16_t BAT_CELL_VOL_MIN;	 //电池单体最低电压

	uint8_t BAT_CELL_OV_FAULT;	 //电池单体过压故障 2位
	uint8_t BAT_CELL_UV_FAULT;	 //电池单体欠压故障 2位
	uint8_t BAT_CELL_DV_FAULT;	 //电池单体压差故障 2位
	uint8_t BAT_CELL_ZERO_FAULT; // 0V禁充故障 2位

	uint8_t BAT_TOTAL_OV_FAULT; //总压过压故障 2位
	uint8_t BAT_TOTAL_UV_FAULT; //总压欠压故障 2位
	uint8_t BAT_OV_FB_FAULT;	//电池回馈过压故障 2位
	uint8_t BAT_CH_OC_FAULT;	//电池充电过流故障 2位

	uint8_t BAT_CH_OC_FB_FAULT;	  //电池回馈过流故障 2位
	uint8_t BAT_DIS_OC_FAULT;	  //电池放电过流故障 2位
	uint8_t BAT_DIS_INOC_FAULT;	  //电池放电瞬时过流故障 2位
	uint8_t BAT_PRE_DIS_OC_FAULT; //电池预放电过流故障 2位

	uint8_t BAT_CH_OT_FAULT;  //电池充电高温故障 2位
	uint8_t BAT_CH_UT_FAULT;  //电池充电低温故障 2位
	uint8_t BAT_DIS_OT_FAULT; //电池放电高温故障 2位
	uint8_t BAT_DIS_UT_FAULT; //电池放电低温故障 2位

	uint8_t BAT_DVUT_FAULT;		  //电池温差过大故障 2位
	uint8_t BAT_EQT_FAULT;		  //电池均衡温度过高 2位
	uint8_t BAT_MOS_OT_FAULT;	  //电池MOS温度过高 2位
	uint8_t BAT_PRE_RES_OT_FAULT; //电池预放电阻温度过高 2位

	uint8_t BAT_PRE_CH_OT_FAULT; //电池预放超时故障 2位
	uint8_t BAT_SOC_UL_FAULT;	 //电池SOC过低故障 2位
	uint8_t BAT_INS_UL_FAULT;	 //电池绝缘过低故障 2位
	uint8_t BAT_AFE_OV_FAULT;	 //电池AFE过压故障 2位

	uint8_t BAT_AFE_UV_FAULT;	  //电池AFE欠压故障 2位
	uint8_t BAT_AFE_DIS_OT_FAULT; //电池AFE放电过温故障 2位
	uint8_t BAT_AFE_DIS_UT_FAULT; //电池AFE放电低温故障 2位
	uint8_t BAT_AFE_CH_OT_FAULT;  //电池AFE充电过温故障 2位

	uint8_t BAT_AFE_CH_UT_FAULT;  //电池AFE充电低温故障 2位
	uint8_t BAT_AFE_DIS_OC_FAULT; //电池AFE放电过流故障 2位
	uint8_t BAT_AFE_CH_UC_FAULT;  //电池AFE充电过流故障 2位
	uint8_t BAT_SC_FAULT;		  //电池短路故障 2位

	uint8_t BAT_FC_FAULT;			 //电池浮充故障 2位
	uint8_t BAT_CELL_GET_VOL_FAULT;	 //电池电压采集故障 2位
	uint8_t BAT_CELL_GET_TEMP_FAULT; //电池温度采集故障 2位
	uint8_t BAT_CELL_GET_FW_FAULT;	 //电池前端采集故障 2位

	uint8_t BAT_DIS_MOS_FAULT;	   //电池放电MOS故障 2位
	uint8_t BAT_PRE_DIS_MOS_FAULT; //电池预放电MOS故障 2位
	uint8_t BAT_CH_MOS_FAULT;	   //电池充电MOS故障 2位
	uint8_t BAT_EOL_FAULT;		   //电池寿命终止故障 2位

	uint8_t BAT_FAULT_REVERSE; //故障状态预留
	uint8_t BAT_FW_VERSION;	   //电池硬件版本

	uint8_t BAT_EXT_FW1_VERSION; //电池硬件版本 2位
	uint8_t BAT_EXT_FW2_VERSION; //电池硬件版本 6位

	uint8_t BAT_CH_PHASE;  //电池充电阶段 4位
	uint8_t BAT_CORE_TYPE; //电芯类型 4位

	uint8_t BAT_CORE_TOTAL; //电芯串数

	uint8_t BAT_DESIGN_MF; //电池设计厂家 3位
	uint8_t BAT_PRO_SPEC;  //电池产品规格 5位

	uint8_t BAT_PRO_TIME; //电池产品年份 4位
	uint8_t BAT_PRO_CODE; //电池生产流水码 20位

	uint8_t BAT_CRC;		//校验码
} OneBusStaticData_t;

void TIM_Configuration(uint16_t arr, uint16_t psc);
void bmsOneBusInit(void);
void bmsOneBusParamInit(bms_info *bms);
void _smart_bms_check_pa(bms_info *bms);
void GPIO_ToggleBits(GPIO_Module *GPIOx, uint16_t GPIO_Pin);
void bmsOneBusHandler(uint8_t *bmsRxBuff);
void GetConfigTimeClear(void);
uint32_t GetConfigLoopTime(void);
uint8_t get_onebus_bat_sta(void);
#endif