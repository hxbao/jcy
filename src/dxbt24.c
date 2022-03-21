#include "dxbt24.h"
#include "includes.h"

atc_t atc;

#define PROTOCOL_HEAD_LEN 6
// crc key值
#define CRC_KEY 7

#define CompareValue 10000 //每隔ms发送一次数据
#define BT24_UART_QUEUE_LMT 300
#define BT24_UART_RECV_BUF_LMT 300

//BT24接收透传数据缓存
unsigned char bt24_queue_buf[BT24_UART_QUEUE_LMT];
unsigned char bt24_rx_buf[BT24_UART_RECV_BUF_LMT];
unsigned char bt24_tx_buf[BT24_UART_RECV_BUF_LMT];

unsigned char *queue_in;
unsigned char *queue_out;

uint8_t bat_rx_buf[100];

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
//=============================================================================
//接收数据帧类型
//=============================================================================
#define         APP_CHECK_CMD         		0x80    	//检测校验
#define         DEVICE_PARAM_CMD            0x0A   		//电池状态
#define         START_DC_CMD                0x91       	//开始放电                       	
#define         GET_DC_CMD                 	0x0B       	//得到放电结果
#define         FW_UPDATA_CMD               0x01       	//固件升级
//=============================================================================
//发送数据帧值
//=============================================================================
#define         BT24_TX_FIRST         		0xF5    	
#define         BT24_TX_ADDRH            	0x5A   		//电池高位
#define         BT24_TX_ADDRL             	0x02       	//地址地位                       	
#define         BT24_TX_CMD              	0x80       	//发送指令


static uint32_t ConfigModuleNoBlockCnt;		 //配置函数延时变量
static uint8_t ConfigModuleNoBlockFlage;	 // 1-配置完 0-未配置完
static uint8_t ConfigModuleNoBlockCaseValue; //控制执行哪一条Case 语句
int DataReturnFlage = 0;					 //是否返回了预期的数据
uint32_t RunCnt = 0;						 //记录运行状态发送的次数
/**
 * DXBT24 5005A功耗情况
 * 唤醒模式下:发送数据 700uA  低功耗模式运行 只广播时 2uA , 设备连接情况下 300uA
 *
 */

/**
 * 蓝牙模块初始化
 */
void DXBT24_Init(void)
{

	// mod管脚进入配置模式
	//   DXBT24_Rst();
	// DXBT24_SWITCH_MOD_AT();
	// bsp_DelayMS(300);
	// USART_SendData(&DXBT24_UARTX, "AT+RESET", strlen("AT+RESET"));
	bsp_DelayMS(300);
	//    DXBT24_SWITCH_MOD_TC();
}

void DXBT24_Rst(void)
{
	DXBT24_RST_LOW();
	bsp_DelayMS(300);
	DXBT24_RST_HIGH();
}

void DXBT24_SetSleepMode(void)
{
	DXBT24_WKP_HIGH();
	DXBT24_SWITCH_MOD_TC();
}

void DXBT24_WakeupFromSleepMode(void)
{
	DXBT24_WKP_LOW();
	DXBT24_SWITCH_MOD_AT();
}

uint8_t DXBT24_CheckLinked(void)
{
	uint8_t linked = 0;

	if (DXBT24_GET_LINK_STAT() == 1)
	{
		linked = 1;
	}
	else
	{
		linked = 0;
	}
	return linked;
}

uint8_t DXBT24_GetMacAddr(uint8_t *mac)
{
	uint8_t ret = 0;
	uint8_t waitCount = 0;
	uint8_t macAddr[20];

	/*
		DXBT24_SWITCH_MOD_AT();
		bsp_DelayMS(200);
		USART_SendDataPoll(&DXBT24_UARTX, "AT+MAC?", strlen("AT+MAC?"));
		while (1)
		{
			if (USART_ReceiveData(&DXBT24_UARTX,macAddr))
			{
				if ((macAddr[0] == 0x2b) &&
					(macAddr[1] == 0x4f) &&
					(macAddr[2] == 0x4b) &&
					(macAddr[3] == 0x3d)) //0x2b 0x4f 0x4b 0x3d
				{
					memcpy(mac, macAddr + 4, 6);
					ret = 1;
				}
			}
			else
			{
				bsp_DelayMS(2);
				waitCount++;
				if (waitCount > 100)
				{
					break;
				}
			}
		}*/
	return ret;
}

void DXBT24_WriteModuleName(uint8_t *name)
{
	uint8_t cmd[48];

	memset(cmd, 0, 48);
	// sprintf(cmd,"AT+NAME1=%s",name);
	cmd[0] = 'A';
	cmd[1] = 'T';
	cmd[2] = '+';
	cmd[3] = 'N';
	cmd[4] = 'A';
	cmd[5] = 'M';
	cmd[6] = 'E';
	cmd[7] = '=';
	memcpy((char *)cmd + 8, (char const *)name, strlen((char const *)name));

	DXBT24_SWITCH_MOD_AT();
	// USART_SendDataPoll(&DXBT24_UARTX, "AT+NAME?",8);
	bsp_DelayMS(300);
	// USART_SendDataPoll(&DXBT24_UARTX, cmd, strlen((char *)cmd)+5);
	DXBT24_SWITCH_MOD_TC();
}

uint8_t DXBT24_ReadModuleName(uint8_t *name)
{
	uint8_t ret = 0;
	uint8_t waitCount = 0;
	uint8_t devID[20];

	DXBT24_SWITCH_MOD_AT();
	bsp_DelayMS(200);
	/*
	//USART_SendDataPoll(&DXBT24_UARTX, "AT+NAME?", strlen("AT+NAME?"));
	while (1)
	{
		if ( USART_ReceiveData(&DXBT24_UARTX,devID))
		{
			if ((devID[0] == 0x2b) &&
				(devID[1] == 0x4f) &&
				(devID[2] == 0x4b) &&
				(devID[3] == 0x3d)) //0x2b 0x4f 0x4b 0x3d
			{
				memcpy(name, devID + 4, strlen((char *)devID + 4));
				ret = 1;
			}
		}
		else
		{
			bsp_DelayMS(2);
			waitCount++;
			if (waitCount > 100)
			{
				break;
			}
		}
	}
	DXBT24_SWITCH_MOD_TC();
	*/
	return ret;
}

void DXBT24_SendData(uint8_t *pData, uint16_t length)
{
	// USART_SendDataPoll(&DXBT24_UARTX, pData, length);
}

void DXBT24_ReceiveHandle(uint8_t *pData, uint16_t length)
{
}
/*****************************************************************************
函数名称 :
功能描述 : 协议串口初始化函数
输入参数 : 无
返回参数 : 无
使用说明 : 必须在MCU初始化代码中调用该函数
*****************************************************************************/
void bt24_protocol_init(void)
{
	queue_in = (unsigned char *)bt24_queue_buf;
	queue_out = (unsigned char *)bt24_queue_buf;
}
/*****************************************************************************
函数名称 :
功能描述 : 收数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 :
*****************************************************************************/
void bt24_receive_input(uint8_t value)
{
	//#error "请在串口接收中断中调用uart_receive_input(value),串口数据由MCU_SDK处理,用户请勿再另行处理,完成后删除该行"

	if ((queue_in > queue_out) && ((queue_in - queue_out) >= sizeof(bt24_queue_buf)))
	{
		//数据队列满
	}
	else if ((queue_in < queue_out) && ((queue_out - queue_in) == 0))
	{
		//数据队列满
	}
	else
	{
		//队列不满
		if (queue_in >= (unsigned char *)(bt24_queue_buf + sizeof(bt24_queue_buf)))
		{
			queue_in = (unsigned char *)(bt24_queue_buf);
		}

		*queue_in++ = value;
	}
}
/*****************************************************************************
函数名称 : get_queue_total_data
功能描述 : 读取队列内数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char get_queue_total_data(void)
{
	if (queue_in != queue_out)
		return 1;
	else
		return 0;
}
/*****************************************************************************
函数名称 : Queue_Read_Byte
功能描述 : 读取队列1字节数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char Queue_Read_Byte(void)
{
	unsigned char value;

	if (queue_out != queue_in)
	{
		//有数据
		if (queue_out >= (unsigned char *)(bt24_queue_buf + sizeof(bt24_queue_buf)))
		{
			//数据已经到末尾
			queue_out = (unsigned char *)(bt24_queue_buf);
		}

		value = *queue_out++;
	}

	return value;
}
/**
 * @brief
 * @param[in] {pack} 数据源指针
 * @param[in] {pack_len} 计算校验和长度
 * @return
 */
unsigned char get_check_crc8(uint8_t *pack, uint8_t pack_len, uint8_t key)
{
	uint8_t i;
	uint8_t crc = 0;
	while (pack_len-- != 0)
	{
		for (i = 0x80; i != 0; i /= 2)
		{
			if ((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
				crc *= 2;

			if ((*pack & i) != 0)
				crc ^= key;
		}
		pack++;
	}
	return (crc);
}
/*****************************************************************************
函数名称 : get_app_check_updata
功能描述 : 
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void get_app_check_updata(uint8_t *batRxBuff)
{
	uint8_t bat_type;
	uint8_t fault_code;
	uint8_t check_crc;
	// bat_type=get_device_bat_type();
	bat_type=2;
	fault_code=get_device_fault_code();
	bt24_tx_buf[BT24_FRAME_FIRST]=BT24_TX_FIRST;
	bt24_tx_buf[BT24_FRAME_ADDRH]=BT24_TX_ADDRH;
	bt24_tx_buf[BT24_FRAME_ADDRL]=BT24_TX_ADDRL;
	bt24_tx_buf[BT24_FRAME_CMDTYPE]=BT24_TX_CMD;
	if((bat_type==1)||(bat_type==0))
	{
		if(fault_code!=0)
		{
			bt24_tx_buf[BT24_FRAME_LENGTH]=0x02;
			bt24_tx_buf[BT24_FRAME_DATATYPE]=0;
			bt24_tx_buf[BT24_FRAME_STATE]=0;
			bt24_tx_buf[BT24_FRAME_FAULT]=fault_code;
			check_crc=get_check_crc8(bt24_tx_buf,strlen(bt24_tx_buf),7);
			bt24_tx_buf[BT24_FRAME_FAULT+1]=check_crc;
			atc_transmit(&atc,bt24_tx_buf,9);
			return;
		}
		else
		{
			bt24_tx_buf[BT24_FRAME_LENGTH]=0x04;
			bt24_tx_buf[BT24_FRAME_DATATYPE]=0x01;
			bt24_tx_buf[BT24_FRAME_STATE]=0;
			bt24_tx_buf[BT24_FRAME_FAULT]=0;
			bt24_tx_buf[BT24_FRAME_RESULT]=0;
			check_crc=get_check_crc8(bt24_tx_buf,strlen(bt24_tx_buf),7);
			bt24_tx_buf[BT24_FRAME_RESULT+1]=check_crc;
			atc_transmit(&atc,bt24_tx_buf,10);
			return;
		}
	}
	else
	{
		bmsOneWireHandler(batRxBuff);
		bt24_tx_buf[BT24_FRAME_LENGTH]=0x4F;
		bt24_tx_buf[BT24_FRAME_LENGTH+3]=batRxBuff[31];
		bt24_tx_buf[BT24_FRAME_LENGTH+4]=batRxBuff[31]>>2;
		bt24_tx_buf[BT24_FRAME_LENGTH+4]=batRxBuff[31]>>4;
	}
	atc_transmit(&atc,bt24_tx_buf,bt24_tx_buf[BT24_FRAME_LENGTH]+7);
}
/**
 * @brief  	处理透传数据
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
void bt24_data_handler(uint8_t offset)
{
#ifdef SUPPORT_MCU_FIRM_UPDATE
	unsigned char *firmware_addr;
	static unsigned long firm_length;	   // MCU升级文件长度
	static unsigned char firm_update_flag; // MCU升级标志
	unsigned long dp_len;
#endif
	uint8_t cmd_type = bt24_rx_buf[offset + BT24_FRAME_CMDTYPE];
	switch (cmd_type)
	{
	case APP_CHECK_CMD:		//检测校验
		/* code */
		get_app_check_updata(bat_rx_buf);
		break;
	case DEVICE_PARAM_CMD:	//获取数据
		
		break;
	case START_DC_CMD:		//放电通知

		break;
	case GET_DC_CMD:		//得到放电结构

		break;
#ifdef SUPPORT_MCU_FIRM_UPDATE
	case FW_UPDATA_CMD:		//固件升级

		break;
#endif	
	default:
		break;
	}
}
/**
 * @brief  	接收到透传数据
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
void bt24_recv_service(atc_t *atc)
{
	static unsigned short rx_in = 0;
	uint8_t offset = 0;
	while ((rx_in < sizeof(bt24_rx_buf)) && get_queue_total_data() > 0)
	{
		bt24_rx_buf[rx_in++] = Queue_Read_Byte();
	}
	if (rx_in < PROTOCOL_HEAD_LEN)
		return;
	while (rx_in - offset >= PROTOCOL_HEAD_LEN)
	{
		if (bt24_rx_buf[offset + BT24_FRAME_FIRST] != BT24_RX_FIRST)
		{
			offset++;
			continue;
		}
		if (bt24_rx_buf[offset + BT24_FRAME_ADDRH] != BT24_RX_ADDRH)
		{
			offset++;
			continue;
		}
		if (bt24_rx_buf[offset + BT24_FRAME_ADDRL] != BT24_RX_ADDRL)
		{
			offset += 2;
			continue;
		}
		if (get_check_crc8(bt24_rx_buf, rx_in - 1, CRC_KEY) != bt24_rx_buf[PROTOCOL_HEAD_LEN + offset - 1])
		{
			//校验出错
			offset += 3;
			continue;
		}
		bt24_data_handler(offset);
		offset += rx_in;
	}
	rx_in -= offset;
	if (rx_in > 0)
	{
		memcpy((char *)bt24_rx_buf, (char *)bt24_rx_buf + offset, rx_in);
	}
}
/**
 * @brief  	BT24蓝牙重启
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t DXBT24_Reset()
{
	char echo_buf[10];
	if (atc_command(&atc, "AT+RESET\r\n", 1000, echo_buf, 20, 1, "OK") == 1)
	{
		return 1;
	}
	return 0;
}
/**
 * @brief  	BT24蓝牙恢复出厂设置
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t DXBT24_Set_Factory()
{
	char echo_buf[10];
	if (atc_command(&atc, "AT+DEFAULT\r\n", 1000, echo_buf, 20, 1, "OK") == 1)
	{
		return 1;
	}
	return 0;
}
/**
 * @brief  	BT24蓝牙恢复出厂设置
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t DXBT24_Set_Name(uint8_t *name)
{
	uint8_t cmd[40];
	char echo_buf[40];
	memset(cmd, 0, 40);
	cmd[0] = 'A';
	cmd[1] = 'T';
	cmd[2] = '+';
	cmd[3] = 'N';
	cmd[4] = 'A';
	cmd[5] = 'M';
	cmd[6] = 'E';
	memcpy((char *)cmd + 7, (char const *)name, strlen((char const *)name));
	strcat(cmd, "\r\n");
	if (atc_command(&atc, cmd, 3000, echo_buf, 20, 1, "+NAME="))
	{
		return 1;
	}
	return 0;
}
/**
 * @brief  	BT24蓝牙AT初始化 轮询
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t DXBT24_AT_Init(uint8_t *name)
{
	char echo_buf[20];
	if (GetConfigLoopTime() > CompareValue && ConfigModuleNoBlockFlage == 0)
	{
		GetConfigTimeClear();
		if (DataReturnFlage == 1) //上一条指令是OK的
		{
			RunCnt = 0;
			DataReturnFlage = 0;
			ConfigModuleNoBlockCaseValue++; //执行下一条
		}
		else if (DataReturnFlage == 0)
		{
			RunCnt++;
			if (RunCnt >= 3)
			{
				RunCnt = 0;
				ConfigModuleNoBlockCaseValue = 0;
				ConfigModuleNoBlockFlage = 1;
			}
		}
		switch (ConfigModuleNoBlockCaseValue)
		{
		case 0:
			if (atc_command(&atc, "AT\r\n", 1000, echo_buf, 20, 1, "OK"))
			{
				DataReturnFlage = 1;
			}
			break;
		case 1:
			if (DXBT24_Set_Name(name) == 1)
			{
				DataReturnFlage = 1;
				ConfigModuleNoBlockFlage = 1;
			}
			break;
		}
	}
}

/**
 * @brief  	AT指令解析
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
void atc_found(char *foundStr)
{
	if (strstr(foundStr, "\r\n") != NULL)
	{

	}
}
/**
 * @brief  	BT24蓝牙协议任务
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
void BT24Task(void const *argument)
{
	uint8_t data_buff[200];
	bt24_protocol_init();
	atc_init(&atc, "MY_ATC", USART2, atc_found);
	atc_addSearch(&atc, "\r\n");
	// atc_command(&atc,"AT\r\n",3000,echo_buf,20,1,"OK");
	for (;;)
	{
		DXBT24_AT_Init("BT24");
		atc_loop(&atc);
		bt24_recv_service(&atc);
	}
}