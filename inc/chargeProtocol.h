#ifndef __CHARGEPROTOCOL_H_
#define __CHARGEPROTOCOL_H_

#include "includes.h"

#define DEVICE_BAT_IDLE				0x00		//����ǿ���״̬
#define DEVICE_BAT_SET				0x01		//���������״̬
#define DEVICE_BAT_DISCHARGE	0x02		    //�ŵ�
#define DEVICE_BAT_CHARGE			0x03		//���
#define DEVICE_BAT_COMPLETE		0x05		    //���

#define SYSTEM_GLOBAL

#ifdef SYSTEM_GLOBAL
  #define SYSTEM_EXTERN
#else
  #define SYSTEM_EXTERN   extern
#endif

#define DATA_PROCESS_LMT           14 
#define UART_RECV_BUF_LMT          24
#define UART_SEND_BUF_LMT           16	
	
//=============================================================================
//Byte order of the frame
//=============================================================================
#define         HEAD_FIRST                      0
#define         DATA_LEN		                1        
#define         FUNC_TYPE                	    2
#define         BAT_DIS_CUR_H                   3
#define         BAT_DIS_CUR_L                   4
#define         BAT_DIS_VOL_H                   5
#define         BAT_DIS_VOL_L                   6
#define         BAT_CH_CUR_H                    7
#define         BAT_CH_CUR_L                    8
#define         BAT_CH_VOL_H                    9
#define         BAT_CH_VOL_L                    10
#define         BAT_CH_CUR_POINT_H				11
#define         BAT_CH_CUR_POINT_L				12
#define         BAT_CH_TIME     				13

//=============================================================================
#define MCU_RX_VER              0x00                                            //ģ�鷢��֡Э��汾��
#define MCU_TX_VER              0x03                                            //MCU ����֡Э��汾��(Ĭ��)
#define PROTOCOL_HEAD           0x0E                                            //�̶�Э��ͷ����
#define FRAME_FIRST             0x5A                                            //֡ͷ��һ�ֽ�
#define FRAME_SECOND            0x0C                                            //֡ͷ�ڶ��ֽ�
//============================================================================= 

typedef struct
{
	float 	 disch_current;						//�ŵ����
	float 	 disch_voltage_temin;			    //�ŵ���ֹ��ѹ
	float 	 ch_current;						//������
	float 	 ch_voltage_temin;				    //�����ߵ�ѹ
	float 	 ch_current_point;				    //�����
	int  ch_time;								//���ʱ��
	int  ch_status;								//���״̬�ϱ�
}ch_param;

SYSTEM_EXTERN volatile unsigned char data_process_buf[DATA_PROCESS_LMT];     //�������ݴ�����
SYSTEM_EXTERN volatile unsigned char uart_rx_buf[UART_RECV_BUF_LMT];         //���ڽ��ջ���
SYSTEM_EXTERN volatile unsigned char uart_tx_buf[UART_SEND_BUF_LMT];        //���ڷ��ͻ���
//
SYSTEM_EXTERN volatile unsigned char *rx_buf_in;
SYSTEM_EXTERN volatile unsigned char *rx_buf_out;

SYSTEM_EXTERN volatile unsigned char stop_update_flag;                      //ENABLE:ֹͣһ�������ϴ�  DISABLE:�ָ�һ�������ϴ�

extern ch_param cp;
extern uint8_t device_bat_param_array[14];

void device_uart_write_frame(float *ch_p,unsigned char *ch_s,unsigned short len);
uint8_t get_device_work_station(void);
float get_device_vol_value(void);
uint16_t get_device_cur_value(void);
#endif
