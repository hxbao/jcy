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

#define ONEWIre_485_CLK RCC_APB2Periph_GPIOB 
#define ONEWIre_485_PIN GPIO_PIN_1
#define ONEWIre_485_PORT GPIOB

#define bms_data		MCU_GPIO_GetBit(ONE_RXD1_PORT,ONE_RXD1_PIN)
#define TEST_PIN_HIGH()	MCU_GPIO_SetBit(TEST_IO_PORT,TEST_IO_PIN)
#define TEST_PIN_LOW()	MCU_GPIO_ClrBit(TEST_IO_PORT,TEST_IO_PIN)

#define ONEWIre_485_ENABLE()	MCU_GPIO_SetBit(ONEWIre_485_PORT,ONEWIre_485_PIN)
#define ONEWIre_485_DISABLE()	MCU_GPIO_ClrBit(ONEWIre_485_PORT,ONEWIre_485_PIN)

#define ONE_TXD1_HIGH()	MCU_GPIO_SetBit(ONE_TXD1_PORT,ONE_TXD1_PIN)
#define ONE_TXD1_LOW()	MCU_GPIO_ClrBit(ONE_TXD1_PORT,ONE_TXD1_PIN)

#define ONE_TXD2_HIGH()	MCU_GPIO_SetBit(ONE_TXD2_PORT,ONE_TXD2_PIN)
#define ONE_TXD2_LOW()	MCU_GPIO_ClrBit(ONE_TXD2_PORT,ONE_TXD2_PIN)

#define RX_DISABLE()	USART2->CTRL1|=8;USART2->CTRL1&=0XFFFFFFFB
#define RX_ENABLE()		USART2->CTRL1|=4;USART2->CTRL1&=0XFFFFFFFF
/*********************************************************
����״̬
*********************************************************/
#define bmsreset 0
#define sync1 1
#define sync2 2
#define bms_h 3
#define bms_l 4
/*********************************************************
Э��ͬ��ͷ
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
	uint8_t MSG_SOF;			 //����ID 8λ
	uint8_t SEC_PRTL;			 //�μ�Э��   4λ
	uint8_t FIR_PRTL;			 //��ҪͨѶЭ��	4λ
	uint8_t BAT_STATUS;			 //���״̬	4λ
	uint8_t BAT_CH_STATUS;		 //��س��״̬ 3λ
	uint8_t BAT_SOC_LEFT;		 //���ʣ��SOC
	uint16_t BAT_CH_MAX_VOL;	 //��������������ѹ
	uint16_t BAT_CH_MAX_CUR;	 //����������������
	uint16_t BAT_CUR_FB;		 //������������������
	uint16_t BAT_DIS_MAX_CUR;	 //�����������ŵ����
	uint16_t BAT_TOTAL_CH_TIME;	 //��ص�������ʣ��ʱ��
	uint16_t BAT_TOTAL_VOL;		 //����ܵ�ѹ
	uint16_t BAT_TOTAL_CUR;		 //����ܵ���
	uint8_t BAT_DIS_MOS_STA;	 //�ŵ�MOS״̬   2λ
	uint8_t BAT_CH_MOS_STA;		 //���MOS״̬	2λ
	uint8_t BAT_PRE_DIS_MOS_STA; //Ԥ�ŵ�MOS״̬	2λ
	uint16_t BAT_SOE_LEFT;		 //���ʣ������
	uint16_t BAT_CIR_TIME;		 //ѭ������
	uint8_t BAT_HEALTH_STA;		 //��ؽ�����
	uint8_t BAT_CELL_TEMP_MAX;	 //��ص�������¶�
	uint8_t BAT_CELL_TEMP_MIN;	 //��ص�������¶�
	uint8_t BAT_MOS_TEMP_MAX;	 //���MOS����¶�
	uint16_t BAT_CELL_VOL_MAX;	 //��ص�����ߵ�ѹ
	uint16_t BAT_CELL_VOL_MIN;	 //��ص�����͵�ѹ

	uint8_t BAT_CELL_OV_FAULT;	 //��ص����ѹ���� 2λ
	uint8_t BAT_CELL_UV_FAULT;	 //��ص���Ƿѹ���� 2λ
	uint8_t BAT_CELL_DV_FAULT;	 //��ص���ѹ����� 2λ
	uint8_t BAT_CELL_ZERO_FAULT; // 0V������� 2λ

	uint8_t BAT_TOTAL_OV_FAULT; //��ѹ��ѹ���� 2λ
	uint8_t BAT_TOTAL_UV_FAULT; //��ѹǷѹ���� 2λ
	uint8_t BAT_OV_FB_FAULT;	//��ػ�����ѹ���� 2λ
	uint8_t BAT_CH_OC_FAULT;	//��س��������� 2λ

	uint8_t BAT_CH_OC_FB_FAULT;	  //��ػ����������� 2λ
	uint8_t BAT_DIS_OC_FAULT;	  //��طŵ�������� 2λ
	uint8_t BAT_DIS_INOC_FAULT;	  //��طŵ�˲ʱ�������� 2λ
	uint8_t BAT_PRE_DIS_OC_FAULT; //���Ԥ�ŵ�������� 2λ

	uint8_t BAT_CH_OT_FAULT;  //��س����¹��� 2λ
	uint8_t BAT_CH_UT_FAULT;  //��س����¹��� 2λ
	uint8_t BAT_DIS_OT_FAULT; //��طŵ���¹��� 2λ
	uint8_t BAT_DIS_UT_FAULT; //��طŵ���¹��� 2λ

	uint8_t BAT_DVUT_FAULT;		  //����²������� 2λ
	uint8_t BAT_EQT_FAULT;		  //��ؾ����¶ȹ��� 2λ
	uint8_t BAT_MOS_OT_FAULT;	  //���MOS�¶ȹ��� 2λ
	uint8_t BAT_PRE_RES_OT_FAULT; //���Ԥ�ŵ����¶ȹ��� 2λ

	uint8_t BAT_PRE_CH_OT_FAULT; //���Ԥ�ų�ʱ���� 2λ
	uint8_t BAT_SOC_UL_FAULT;	 //���SOC���͹��� 2λ
	uint8_t BAT_INS_UL_FAULT;	 //��ؾ�Ե���͹��� 2λ
	uint8_t BAT_AFE_OV_FAULT;	 //���AFE��ѹ���� 2λ

	uint8_t BAT_AFE_UV_FAULT;	  //���AFEǷѹ���� 2λ
	uint8_t BAT_AFE_DIS_OT_FAULT; //���AFE�ŵ���¹��� 2λ
	uint8_t BAT_AFE_DIS_UT_FAULT; //���AFE�ŵ���¹��� 2λ
	uint8_t BAT_AFE_CH_OT_FAULT;  //���AFE�����¹��� 2λ

	uint8_t BAT_AFE_CH_UT_FAULT;  //���AFE�����¹��� 2λ
	uint8_t BAT_AFE_DIS_OC_FAULT; //���AFE�ŵ�������� 2λ
	uint8_t BAT_AFE_CH_UC_FAULT;  //���AFE���������� 2λ
	uint8_t BAT_SC_FAULT;		  //��ض�·���� 2λ

	uint8_t BAT_FC_FAULT;			 //��ظ������ 2λ
	uint8_t BAT_CELL_GET_VOL_FAULT;	 //��ص�ѹ�ɼ����� 2λ
	uint8_t BAT_CELL_GET_TEMP_FAULT; //����¶Ȳɼ����� 2λ
	uint8_t BAT_CELL_GET_FW_FAULT;	 //���ǰ�˲ɼ����� 2λ

	uint8_t BAT_DIS_MOS_FAULT;	   //��طŵ�MOS���� 2λ
	uint8_t BAT_PRE_DIS_MOS_FAULT; //���Ԥ�ŵ�MOS���� 2λ
	uint8_t BAT_CH_MOS_FAULT;	   //��س��MOS���� 2λ
	uint8_t BAT_EOL_FAULT;		   //���������ֹ���� 2λ

	uint8_t BAT_FAULT_REVERSE; //����״̬Ԥ��
	uint8_t BAT_FW_VERSION;	   //���Ӳ���汾

	uint8_t BAT_EXT_FW1_VERSION; //���Ӳ���汾 2λ
	uint8_t BAT_EXT_FW2_VERSION; //���Ӳ���汾 6λ

	uint8_t BAT_CH_PHASE;  //��س��׶� 4λ
	uint8_t BAT_CORE_TYPE; //��о���� 4λ

	uint8_t BAT_CORE_TOTAL; //��о����

	uint8_t BAT_DESIGN_MF; //�����Ƴ��� 3λ
	uint8_t BAT_PRO_SPEC;  //��ز�Ʒ��� 5λ

	uint8_t BAT_PRO_TIME; //��ز�Ʒ��� 4λ
	uint8_t BAT_PRO_CODE; //���������ˮ�� 20λ

	uint8_t BAT_CRC;		//У����
} OneBusStaticData_t;

void TIM_Configuration(uint16_t arr, uint16_t psc);
void bmsOneBusInit(void);
void bmsOneBusParamInit(bms_info *bms);
void _smart_bms_check_pa(bms_info *bms);
void GPIO_ToggleBits(GPIO_Module *GPIOx, uint16_t GPIO_Pin);
void bmsOneBusHandler(void);
void GetConfigTimeClear(void);
uint32_t GetConfigLoopTime(void);
uint8_t get_onebus_bat_sta(void);
#endif