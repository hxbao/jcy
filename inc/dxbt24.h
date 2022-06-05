#ifndef _DXBT_H
#define _DXBT_H

#include "includes.h"

#define DXBT24_UARTX 1

#define BLE_NAME  "XNA"	//������������
//=============================================================================
//��������ֵ֡
//=============================================================================
#define BT24_RX_FIRST 0x5F
#define BT24_RX_ADDRH 0x02
#define BT24_RX_ADDRL 0x5A
//=============================================================================
// BT֡���ֽ�˳��
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
//��������֡����
//=============================================================================
#define         APP_CHECK_CMD         		0x80    	//���У��
#define         DEVICE_PARAM_CMD            0x0A   		//���״̬
#define         START_DC_CMD                0x91       	//��ʼ�ŵ�                       	
#define         GET_DC_CMD                 	0x0B       	//�õ��ŵ���
#define         GET_CODE_CMD                0x0C       	//�õ��ŵ���
#define         GET_FW_CMD                  0x01       	//�̼�����
#define         FW_UPDATA_CHECK             0x02        //�����°汾����
#define         FW_UPDATA_CONTENT           0x03        //��������
#define         FW_UPDATA_OVER              0x04        //���ͽ���
#define         FW_UPDATA_RESULT            0x05        //��ѯ�������
//=============================================================================
//��������ֵ֡
//=============================================================================
#define         BT24_TX_FIRST         		0xF5    	
#define         BT24_TX_ADDRH            	0x5A   		//��ظ�λ
#define         BT24_TX_ADDRL             	0x02       	//��ַ��λ                       	
#define         BT24_TX_CHECK_CMD           0x80       	//���У��ָ��
#define         BT24_TX_DP_CMD              0x0A       	//��ȡ��̬����ָ��
#define         BT24_TX_DC_CMD              0x91       	//�ŵ�ָ��
#define         BT24_TX_GET_CMD             0x0B       	//ѯ�ʷŵ���ָ��

//=============================================================================
//�ظ����У������
//=============================================================================
typedef struct
{
    uint8_t DataType;                       //��������
    uint8_t Fault_Code;                       //�������
	uint8_t CELL_OV;							//�����ѹ  4λ
    uint8_t CELL_UV;                             //����Ƿѹ 4λ
    uint8_t CELL_UNBA;                      //ѹ��          4λ
    uint8_t TOTAL_OV;                      //��ѹ����       4λ
    uint8_t TOTAL_UV;                      //��ѹ����       4λ
    uint8_t CHG_CON_OI;                      //���������� 4λ
    uint8_t CHG_PUL_OI;                      //���˲ʱ���� 4λ
    uint8_t FEEDBACK_CON_OI;                      //������������ 4λ
    uint8_t FEEDBACK_PUL_OI;                      //����˲ʱ���� 4λ
    uint8_t DSG_CON_OI;                      //�ŵ�������� 4λ
    uint8_t DSG_PUL_OI;                      //�ŵ�˲ʱ���� 4λ
    uint8_t PRE_OI;                      //Ԥ����� 4λ
    uint8_t CHG_POWER;                      //��繦�ʹ��� 4λ
    uint8_t DSG_POWER;                      //�ŵ繦�ʹ��� 4λ
    uint8_t HEAT_POWER;                      //���ȹ��ʹ��� 4λ
    uint8_t CHG_OT;                      //������ 4λ
    uint8_t CHG_UT;                      //������ 4λ
    uint8_t CHG_T_UNBA;                      //����²���� 4λ
    uint8_t DSG_OT;                      //�ŵ���� 4λ
    uint8_t DSG_UT;                      //�ŵ���� 4λ
    uint8_t SG_T_UNBA;                      //�ŵ��²���� 4λ
    uint8_t T_RISE_FAST;                      //��о�������� 4λ
    uint8_t CELL_OR;                      //����������� 4λ
    uint8_t CELL_SHORT;                      //�����· 4λ
    uint8_t CELL_0V;                      //����0V 4λ
    uint8_t CELL_OV_SEVERE;                      //���ع�ѹ 4λ
    uint8_t SOC_LOW;                      //SOC�� 4λ
    uint8_t SOC_HIGH;                      //SOC�� 4λ
    uint8_t SOH_LOW;                      //SOH�� 4λ
    uint8_t FC;                      //���� 4λ
    uint8_t AFE_OV;                      //AFE�жϹ�ѹ 4λ
    uint8_t AFE_UV;                      //AFE�ж�Ƿѹ 4λ
    uint8_t AFE_OT;                      //AFE�жϹ��� 4λ
    uint8_t AFE_UT;                      //AFE�ж�Ƿ�� 4λ
    uint8_t AFE_OI;                      //AFE�жϹ��� 4λ
    uint8_t BAL_OT;                      //������� 4λ
    uint8_t HEATER_OT;                      //����Ĥ���� 4λ
    uint8_t HEAT_SLOW;                      //��о���ȹ��� 4λ
    uint8_t MOS1_OT;                      //MOS1����(���)
    uint8_t MOS2_OT;                      //MOS2����(�ŵ�)
    uint8_t MOS3_OT;                      //MOS3����(Ԥ��)
    uint8_t MOS4_OT;                      //MOS4����
    uint8_t MOS5_OT;                      //MOS5����
    uint8_t MOS6_OT;                      //MOS6����
    uint8_t PRE_OT;                      //Ԥ�����
    uint8_t PRE_OTIME;                      //Ԥ�䳬ʱ
    uint8_t PRE_FAST;                      //Ԥ�����
    uint8_t RESERVE1_OT;                      //Ԥ������
    uint8_t RESERVE2_OT;                      //Ԥ������
    uint8_t RESERVE3_OT;                      //Ԥ������
    uint8_t RESERVE4_OT;                      //Ԥ������
    uint8_t RESERVE5_OT;                      //Ԥ������
    uint8_t RESERVE6_OT;                      //Ԥ������
    uint8_t FUSE_OT;                      //FUSE����
    uint8_t ISO_UR;                      //��Ե����
    uint8_t CHG_OTIME;                      //����ʱ
    uint8_t CHARGER;                      //���������
    uint8_t CHG_VERSION;                      //���汾
    uint8_t CHG_RESERVE;                      //���Ԥ��
    uint8_t VCU_CMD;                      //VCU����
    uint8_t DSG_RESERVE;                      //�ŵ�Ԥ��
    uint8_t COMM_INSTRUMENT;                      //�Ǳ�ͨѶ
    uint8_t COMM_CHG;                      //���ͨѶ
    uint8_t COMM_VCU;                      //VCUͨѶ
    uint8_t COMM_INT;                      //�ڲ�ͨѶ
    uint8_t SHORT;                      //��·
    uint8_t EOL_OVER;                      //EOL������ֹ
    uint8_t SLEEP;                      //����ģʽ
    uint8_t SHUT;                      //�͵����ػ�
    uint8_t RESERVE_0;                      //Ԥ��_0����ֵΪ0
    uint8_t RESERVE_1;                      //Ԥ��_1����ֵΪ0
    uint8_t RESERVE_2;                      //Ԥ��_2����ֵΪ0
    uint8_t RESERVE_3;                      //Ԥ��_3����ֵΪ0
    uint8_t RESERVE_4;                      //Ԥ��_4����ֵΪ0
    uint8_t RESERVE_5;                      //Ԥ��_5����ֵΪ0
    uint8_t RESERVE_6;                      //Ԥ��_6����ֵΪ0
    uint8_t RESERVE_7;                      //Ԥ��_7����ֵΪ0
    uint8_t VCELL_OPEN;                      //�����ѹ�ɼ�����
    uint8_t VCELL_SHORT;                      //�����ѹ�ɼ���·
    uint8_t VCELL_DRIFT;                      //�����ѹƯ��
    uint8_t T_OPEN;                      //�¶ȶ���
    uint8_t T_SHORT;                      //�¶ȶ�·
    uint8_t T_DRIFR;                      //�¶�Ư��
    uint8_t CUTRRENT_OPEN;                      //����������
    uint8_t CUTRRENT_SHORT;                      //��������·
    uint8_t CUTRRENT_DRIFT;                      //����Ư��
    uint8_t VSUM_OPEN;                      //��ѹ��⿪·
    uint8_t VSUM_SHORT;                      //��ѹ����·
    uint8_t VSUM_DRIFT;                      //��ѹƯ��
    uint8_t T_MCU_OPEN;                      //MCU����¶ȿ�·
    uint8_t T_MCU_SHORT;                      //MCU����¶ȶ�·
    uint8_t ISO;                      //��Ե��·�쳣
    uint8_t POSACTOR_DRIVE;                      //����ACTOR��������(MOS����ʧЧ)
    uint8_t NEGACTOR_DRIVE;                      //����ACTOR��������
    uint8_t PREACTOR_DRIVE;                      //����ACTOR��������
    uint8_t HEATACTOR_DRIVE;                      //����ACTOR��������
    uint8_t ACTOR5_DRIVE;                      //ACTOR5��������
    uint8_t ACTOR6_DRIVE;                      //ACTOR6��������
    uint8_t ACTOR7_DRIVE;                      //ACTOR7��������
    uint8_t ACTOR8_DRIVE;                      //ACTOR8��������
    uint8_t ACTOR9_DRIVE;                      //ACTOR9��������
    uint8_t ACTOR10_DRIVE;                      //ACTOR10��������
    uint8_t POSACTOR_ADHESION;                      //����ACTORճ��
    uint8_t NEGACTOR_ADHESION;                      //����ACTORճ��
    uint8_t PREACTOR_ADHESION;                      //Ԥ��ACTORճ��
    uint8_t HEATACTOR_ADHESION;                      //����ACTORճ��
    uint8_t ACTOR5_ADHESION;                      //ACTOR5ճ��
    uint8_t ACTOR6_ADHESION;                      //ACTOR6ճ��
    uint8_t ACTOR7_ADHESION;                      //ACTOR7ճ��
    uint8_t ACTOR8_ADHESION;                      //ACTOR8ճ��
    uint8_t ACTOR9_ADHESION;                      //ACTOR9ճ��
    uint8_t ACTOR10_ADHESION;                      //ACTOR10ճ��
    uint8_t FAST_HEATER;                      //����Ĥ��������
    uint8_t SLOW_HEATER;                      //����Ĥ������
    uint8_t RESERVE_COOL;                      //�������
    uint8_t BAL_SHORT;                      //�����·
    uint8_t BAL_OPEN;                      //�������
    uint8_t EEPROM_COMM;                      //EEPROMͨѶ
    uint8_t EEPROM_FAULT;                      //EEPROM����
    uint8_t RTC_COMM;                      //RTCͨѶ
    uint8_t RTC_FAULT;                      //RTC����
    uint8_t AFE_COMM;                      //AFEͨѶ
    uint8_t AFE_FAULT;                      //AFE����
    uint8_t DOG_COMM;                      //���Ź�ͨѶ
    uint8_t DOG_FAULT;                      //���Ź�����
    uint8_t COMM_CAN;                      //CANͨѶ
    uint8_t COMM_485;                      //485ͨѶ
    uint8_t POWER_BASE_HIGH;                      //��׼��ѹ��
    uint8_t POWER_BASE_LOW;                      //��׼��ѹ��
    uint8_t POWER_12V_HIGH;                      //12V��
    uint8_t POWER_12V_LOW;                      //12V��
    uint8_t POWER_5V_HIGH;                      //5V�ߣ�5V���ϣ������ߵͣ�
    uint8_t POWER_5V_LOW;                      //5V��
    uint8_t POWER_3_3V_HIGH;                      //3.3V��
    uint8_t POWER_3_3V_LOW;                      //3.3V��
    uint8_t FUSE;                      //FUSE
    uint8_t MCU_CORE;                      //MCU�ں�
    uint8_t MCU_RAM;                      //MCU RAM
    uint8_t MCU_FLASH;                      //MCU Flash
    uint8_t MCU_INTERRUPT;                      //MCU�ж�
    uint8_t MCU_POWER;                      //MCU��Դ
    uint8_t MCU_AD;                      //MCU��ѹ
    uint8_t MCU_CLOCK;                      //MCU ʱ��
    uint8_t MCU_GPIO;                      //MCU GPIO
    uint8_t MCU_EEPROM;                      //MCU EEPROM
    uint8_t MCU_BUS;                      //MCU ����
    uint8_t RESERVE_8;                      //Ԥ��_8����ֵΪ0
    uint8_t RESERVE_9;                      //Ԥ��_9����ֵΪ0
    uint8_t RESERVE_10;                      //Ԥ��_10����ֵΪ0
    uint8_t RESERVE_11;                      //Ԥ��_11����ֵΪ0
    uint8_t RESERVE_12;                      //Ԥ��_12����ֵΪ0
    uint8_t RESERVE_13;                      //Ԥ��_13����ֵΪ0
    uint8_t RESERVE_14;                      //Ԥ��_14����ֵΪ0
    uint8_t CRC_CHECK;                      //CRCУ��
}BleResponseCheckCmdErr_t;
//=============================================================================
//�ظ��ŵ�/��̬����/�ŵ���
//=============================================================================
typedef struct 
{
    uint8_t  DataType;                       //��������
    uint8_t  DSG_Result;                      //ִ������
    uint16_t CellVolt1;                      //��о��ѹ1
    uint16_t CellVolt2;                      //��о��ѹ2
    uint16_t CellVolt3;                      //��о��ѹ3
    uint16_t CellVolt4;                      //��о��ѹ4
    uint16_t CellVolt5;                      //��о��ѹ5
    uint16_t CellVolt6;                      //��о��ѹ6
    uint16_t CellVolt7;                      //��о��ѹ7
    uint16_t CellVolt8;                      //��о��ѹ8
    uint16_t CellVolt9;                      //��о��ѹ9
    uint16_t CellVolt10;                      //��о��ѹ10
    uint16_t CellVolt11;                      //��о��ѹ11
    uint16_t CellVolt12;                      //��о��ѹ12
    uint16_t CellVolt13;                      //��о��ѹ13
    uint16_t Current;                      //��·���������Ϊ�����ŵ�Ϊ��
    uint16_t MaxCellVolt;                      //����о��ѹ
    uint16_t MinCellVolt;                      //��С��о��ѹ
    uint16_t MaxCellDeltV;                      //����оѹ��
    uint16_t MaxCellTemp;                      //����о�¶�
    uint16_t MinCellTemp;                      //��С��о�¶�
    uint16_t CellDeltVolt;                      //��о��ѹѹ��
    uint16_t BoardTemp;                      //�����¶�
    uint16_t MosTemp;                      //Mos���¶�
    uint16_t BattSumVolt;                      //�����ѹ(��о��ѹ�ۼ�ֵ)
    uint16_t BattVolt;                      //�����ѹ(���ֵ)
    uint16_t PackVolt;                      //����ѹ
    uint16_t SOC;                      //�����ѹ(���ֵ)
    uint16_t SOH;                      //�����ѹ(���ֵ)
    uint16_t MaxChgCurr;                      //��������
    uint16_t MaxDsgCurr;                      //���ŵ����
    uint8_t FaultCode;                       //������
    uint8_t CRC_CHECK;                      //CRCУ��
}BleResponseErr_t;

//=============================================================================
//BLE��������ṹ��
//=============================================================================
typedef struct 
{
    uint8_t  OPERA_STA; //����ģʽ
    uint16_t DSG_CUR;
    uint32_t DSG_END_VOL;
    uint16_t CH_CUR;
    uint32_t CH_END_VOL;
    uint32_t DataType;
}BleSendErr_t;

//=============================================================================
//BLE�豸���ƽṹ��
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