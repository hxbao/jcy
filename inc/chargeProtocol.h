#ifndef __CHARGEPROTOCOL_H_
#define __CHARGEPROTOCOL_H_

#include "includes.h"

#define DEVICE_BAT_IDLE				0x00		//����ǿ���״̬
#define DEVICE_BAT_SET				0x01		//���������״̬
#define DEVICE_BAT_DISCHARGE	0x02		    //�ŵ�
#define DEVICE_BAT_CHARGE			0x03		//���
#define DEVICE_BAT_COMPLETE		0x05		    //���


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
//���豸����֡
//=============================================================================
#define DEV_PROTOCOL_HEAD           0x1E                                            //�̶�Э��ͷ����
#define DEV_FRAME_FIRST             0x7E                                            //֡ͷ��һ�ֽ�
#define DEV_FRAME_SECOND            0x0B 
#define DEV_FRAME_TATOL_LEN         0x1F                                             //֡ͷ�ڶ��ֽ�
//============================================================================= 
//ģ�鷢��֡
//============================================================================= 
#define MOD_FRAME_FIRST             0x5A                                            //֡ͷ��һ�ֽ�
#define MOD_FRAME_SECOND            0x0A 
#define MOD_FRAME_TATOL_LEN         0x18                                             //֡ͷ�ڶ��ֽ�
//============================================================================= 
//����֡���ͷ���
//============================================================================= 
typedef enum
{
	DEV2MOD,
	MOD2DEV
}FrameType;
//============================================================================= 
//����֡�����ṹ��
//============================================================================= 
typedef struct
{
	uint8_t  BAT_STA;							//���״̬
	uint8_t  BAT_EC;							//��о��ϵ
	uint8_t  BAT_SPE;							//��ع��
	uint16_t  BAT_CAP;							//�������
	uint32_t  BAT_SN;							//���SN
	uint8_t  BAT_OPERA_STA;						//��ع���״̬
	uint8_t	 BAT_DET_MODE;						//���ü��ģʽ
	uint16_t BAT_DISCH_CUR;						//���÷ŵ����
	uint32_t BAT_DISCH_END_VOL;					//�ŵ���ֹ��ѹ
	uint16_t BAT_CH_CUR;						//���ó�����
	uint32_t BAT_CH_END_VOL;					//�����ֹ��ѹ
	uint32_t BAT_CH_END_CUR;					//����������
	uint32_t BAT_SET_DET_MODE;					//���ù���ģʽ

	uint8_t  DEV_STATUS;						//�豸״̬
	uint32_t DEV_GET_VOL;						//��ǰ��ѹ
	uint16_t DEV_GET_CUR;						//��ǰ����
	uint8_t  DEV_TIME_H;						//����ʱ�� Сʱ
	uint8_t  DEV_TIME_M;						//����ʱ�� ����
	uint8_t  DEV_TIME_S;						//����ʱ�� ����
	
	uint16_t DEV_GET_SPE;						//�ۻ�����
	uint16_t  DEV_SET_DISCH_CUR;					//���÷ŵ����
	uint32_t  DEV_SET_DISCH_END_VOL;				//�ŵ��ֹ��ѹ
	uint16_t  DEV_SET_CH_CUR;					//���ó�����
	uint32_t  DEV_SET_CH_END_VOL;				//���ó���ֹ��ѹ
	uint16_t  DEV_SET_CH_END_CUR;				//���ó���ֹ����

	uint8_t DEV_BAT_TYPE;						//�������
	uint8_t	DEV_FAULT_CODE;						//����ǹ��ϴ���
	uint16_t DEV_BLE_ID;						//����ID

	int  ch_time;								//���ʱ��
	int  ch_status;								//���״̬�ϱ�
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
