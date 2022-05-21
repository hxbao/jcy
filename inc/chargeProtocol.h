#ifndef __CHARGEPROTOCOL_H_
#define __CHARGEPROTOCOL_H_

#include "includes.h"

#define DEVICE_BAT_IDLE				0x00		//检测仪空闲状态
#define DEVICE_BAT_SET				0x01		//检测仪设置状态
#define DEVICE_BAT_DISCHARGE	0x02		    //放电
#define DEVICE_BAT_CHARGE			0x03		//充电
#define DEVICE_BAT_COMPLETE		0x05		    //完成


#define DATA_PROCESS_LMT           64 
#define UART_RECV_BUF_LMT          64
#define UART_SEND_BUF_LMT          64	
	
//=============================================================================
//Byte order of the frame
//=============================================================================
#define         HEAD_FIRST                      0
#define         DEV_ADDR		                1        
#define         DATA_LEN                	    2
#define         BAT_STATE_FRAME					3
#define         BAT_CORE_FRAME					4
#define         BAT_SPE_FRAME					5
#define         BAT_CAP_FRAME					6
#define         BAT_SN_FRAME					7	
#define         BAT_OPERA_FRAME					11
#define         BAT_DISCH_CUR_FRAME				12
#define         BAT_DISCH_END_VOL_FRAME			14
#define         BAT_CH_CUR_FRAME				18
#define         BAT_CH_END_VOL_FRAME			20	
#define         BAT_CH_END_CUR_FRAME			24
#define         BAT_SET_MODE_FRAME				26
#define         BAT_TO_DEV_CRC					27
//=============================================================================
//从设备接收帧
//=============================================================================
#define DEV_PROTOCOL_HEAD           0x1E                                            //固定协议头长度
#define DEV_FRAME_FIRST             0x7E                                            //帧头第一字节
#define DEV_FRAME_SECOND            0x0B 
#define DEV_FRAME_TATOL_LEN         0x1F                                             //帧头第二字节
//============================================================================= 
//模块发送帧
//============================================================================= 
#define MOD_FRAME_FIRST             0x5A                                            //帧头第一字节
#define MOD_FRAME_SECOND            0x0A 
#define MOD_FRAME_TATOL_LEN         0x18                                             //帧头第二字节
//============================================================================= 
//数据帧发送方向
//============================================================================= 
typedef enum
{
	DEV2MOD,
	MOD2DEV
}FrameType;
//============================================================================= 
//数据帧参数结构体
//============================================================================= 
typedef struct
{
	uint8_t  BAT_STA;							//电池状态
	uint8_t  BAT_EC;							//电芯体系
	uint8_t  BAT_SPE;							//电池规格
	uint16_t  BAT_CAP;							//电池容量
	uint32_t  BAT_SN;							//电池SN
	uint8_t  BAT_OPERA_STA;						//电池工作状态
	uint8_t	 BAT_DET_MODE;						//设置检测模式
	uint16_t BAT_DISCH_CUR;						//设置放电电流
	uint32_t BAT_DISCH_END_VOL;					//放电终止电压
	uint16_t BAT_CH_CUR;						//设置充电电流
	uint32_t BAT_CH_END_VOL;					//充电终止电压
	uint32_t BAT_CH_END_CUR;					//充电截至电流
	uint32_t BAT_SET_DET_MODE;					//设置工作模式

	uint8_t  DEV_STATUS;						//设备状态
	uint32_t DEV_GET_VOL;						//当前电压
	uint16_t DEV_GET_CUR;						//当前电流
	uint8_t  DEV_TIME_H;						//工作时间 小时
	uint8_t  DEV_TIME_M;						//工作时间 分钟
	uint8_t  DEV_TIME_S;						//工作时间 秒钟
	
	uint16_t DEV_GET_SPE;						//累积容量
	uint16_t  DEV_SET_DISCH_CUR;					//设置放电电流
	uint32_t  DEV_SET_DISCH_END_VOL;				//放电截止电压
	uint16_t  DEV_SET_CH_CUR;					//设置充电电流
	uint32_t  DEV_SET_CH_END_VOL;				//设置充电截止电压
	uint16_t  DEV_SET_CH_END_CUR;				//设置充电截止电流

	uint8_t DEV_BAT_TYPE;						//电池类型
	uint8_t	DEV_FAULT_CODE;						//检测仪故障代码
	uint16_t DEV_BLE_ID;						//蓝牙ID

	int  ch_time;								//充电时间
	int  ch_status;								//充电状态上报
}DeviceResponseCmdErr_t;

void device_uart_write_frame(void);
uint8_t get_device_work_station(void);
uint32_t get_device_vol_value(void);
uint16_t get_device_cur_value(void);
void device_uart_receive_input(unsigned char value);
void device_data_handle(unsigned short offset,DeviceResponseCmdErr_t *DRC);
uint8_t get_device_fault_code(void);
void device_protocol_init(void);
void device_uart_service(void);
uint8_t get_device_bat_type(void);
uint16_t get_device_send_ble_id(void);
void device_drc_init(void);
void get_device_recv_flag(void);
uint8_t get_device_time_h(void);
uint8_t get_device_time_s(void);
uint8_t get_device_time_m(void);
#endif
