#include "dxbt24.h"
#include "includes.h"

atc_t atc;
BleResponseErr_t BRE;
BleResponseCheckCmdErr_t BRCC;
BleSendErr_t BSE;
BleDeviceNameErr_t	BDN;

#define BLE_ID_SAVE_ADDR	0x0800F000

#define BLE_STA_CLK RCC_APB2Periph_GPIOA
#define BLE_STA_PIN GPIO_PIN_7
#define BLE_STA_PORT GPIOA

#define PROTOCOL_HEAD_LEN 6
// crc key值
#define CRC_KEY 7

#define CompareValue 5000 //每隔ms发送一次数据
#define BT24_UART_QUEUE_LMT 300
#define BT24_UART_RECV_BUF_LMT 300

static char BLE_NAME[]="DX-BT24-";	//定义蓝牙名称

// BT24接收透传数据缓存
unsigned char bt24_queue_buf[BT24_UART_QUEUE_LMT];
unsigned char bt24_rx_buf[BT24_UART_RECV_BUF_LMT];
unsigned char bt24_tx_buf[BT24_UART_RECV_BUF_LMT];

unsigned char *queue_in;
unsigned char *queue_out;

uint8_t bat_rx_buf[100];

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
char __num2hex(uint8_t num)
{
    if (num <= 0x9) {
        return num + '0';
    }

    if ((0xA <= num) && (num <= 0xF)) {
        return num - 0xA + 'A';
    }

    return (char)-1;
}
/**
 * HEX转字符串
 */
void __hex2str(uint8_t *in, char *out, int len)
{
    int i = 0;

    for (i = 0; i < len; ++i) {
        out[(len-i) * 2-2] = __num2hex(in[i] >> 4);
        out[(len-i) * 2-1] = __num2hex(in[i] & 0x0F);
    }
    out[2 * len] = '\0';
}
/**
 * 蓝牙模块P0-11初始化
 */
static void DX_BT24_Init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);   
    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */

    GPIO_InitStructure.GPIO_Mode     = GPIO_Mode_Input;
    GPIO_InitStructure.Pin            = BLE_STA_PIN;
    GPIO_InitStructure.GPIO_Pull      = GPIO_No_Pull;
    GPIO_InitPeripheral(BLE_STA_PORT, &GPIO_InitStructure);  
}
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
	// DXBT24_RST_LOW();
	bsp_DelayMS(300);
	// DXBT24_RST_HIGH();
}

void DXBT24_SetSleepMode(void)
{
	// DXBT24_WKP_HIGH();
	// DXBT24_SWITCH_MOD_TC();
}

void DXBT24_WakeupFromSleepMode(void)
{
	// DXBT24_WKP_LOW();
	// DXBT24_SWITCH_MOD_AT();
}

// uint8_t DXBT24_CheckLinked(void)
// {
// 	uint8_t linked = 0;

// 	if (DXBT24_GET_LINK_STAT() == 1)
// 	{
// 		linked = 1;
// 	}
// 	else
// 	{
// 		linked = 0;
// 	}
// 	return linked;
// }

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

	// DXBT24_SWITCH_MOD_AT();
	// USART_SendDataPoll(&DXBT24_UARTX, "AT+NAME?",8);
	bsp_DelayMS(300);
	// USART_SendDataPoll(&DXBT24_UARTX, cmd, strlen((char *)cmd)+5);
	// DXBT24_SWITCH_MOD_TC();
}

uint8_t DXBT24_ReadModuleName(uint8_t *name)
{
	uint8_t ret = 0;
	uint8_t waitCount = 0;
	uint8_t devID[20];

	// DXBT24_SWITCH_MOD_AT();
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
	SEGGER_RTT_printf(0, "bt24_crc_value %x\n",crc);
	return (crc);
}
/*****************************************************************************
函数名称 : get_app_check_updata
功能描述 :	检测校验
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void get_app_check_updata(uint8_t offset,uint8_t *batRxBuff)
{
	uint8_t bat_type;
	uint8_t fault_code;
	uint8_t check_crc;
	bat_type=get_onebus_bat_type();
	// bat_type = 2;
	fault_code = get_device_fault_code();
	bt24_tx_buf[BT24_FRAME_FIRST] = BT24_TX_FIRST;
	bt24_tx_buf[BT24_FRAME_ADDRH] = BT24_TX_ADDRH;
	bt24_tx_buf[BT24_FRAME_ADDRL] = BT24_TX_ADDRL;
	bt24_tx_buf[BT24_FRAME_CMDTYPE] = BT24_TX_CHECK_CMD;
	if ((bat_type == 1) || (bat_type == 0))
	{
		if (fault_code != 0)
		{
			bt24_tx_buf[BT24_FRAME_LENGTH] = 0x03;
			bt24_tx_buf[BT24_FRAME_DATATYPE] = bat_type;
			bt24_tx_buf[BT24_FRAME_STATE] = 0;
			bt24_tx_buf[BT24_FRAME_FAULT] = fault_code;
			check_crc = get_check_crc8(bt24_tx_buf, (bt24_tx_buf[BT24_FRAME_LENGTH]+5), 7);
			bt24_tx_buf[BT24_FRAME_FAULT + 1] = check_crc;
			atc_transmit(&atc, bt24_tx_buf, bt24_tx_buf[BT24_FRAME_LENGTH]+6);
			return;
		}
		else
		{
			bt24_tx_buf[BT24_FRAME_LENGTH] = 0x04;
			bt24_tx_buf[BT24_FRAME_DATATYPE] = bat_type;
			bt24_tx_buf[BT24_FRAME_STATE] = 0;
			bt24_tx_buf[BT24_FRAME_FAULT] = 0;
			bt24_tx_buf[BT24_FRAME_RESULT] = 0;
			check_crc = get_check_crc8(bt24_tx_buf, (bt24_tx_buf[BT24_FRAME_LENGTH]+5), 7);
			bt24_tx_buf[BT24_FRAME_RESULT + 1] = check_crc;
			atc_transmit(&atc, bt24_tx_buf, bt24_tx_buf[BT24_FRAME_LENGTH]+6);
			return;
		}
	}
	else
	{
		bmsOneBusHandler(batRxBuff);
		bt24_tx_buf[BT24_FRAME_LENGTH] = 0x50;
		bt24_tx_buf[BT24_FRAME_DATATYPE] = bat_type;
		bt24_tx_buf[BT24_FRAME_STATE] = 0; 	
		/*
		读取一线通/485/CAN 
		故障信息
		*/

		check_crc = get_check_crc8(bt24_tx_buf, (bt24_tx_buf[BT24_FRAME_LENGTH]+5), 7);
		bt24_tx_buf[bt24_tx_buf[BT24_FRAME_LENGTH] + 5] = check_crc;
	}
	atc_transmit(&atc, bt24_tx_buf, bt24_tx_buf[BT24_FRAME_LENGTH] + 6);
}
/*****************************************************************************
函数名称 : get_device_param_handler
功能描述 :	获取静态数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void get_device_param_handler(uint8_t offset,uint8_t *batRxBuff)
{
	uint8_t check_crc;
	uint8_t bat_type;
	bat_type=get_onebus_bat_type();
	//  bat_type = 2;
	if(bat_type==1)
	{
		bt24_tx_buf[BT24_FRAME_FIRST] = BT24_TX_FIRST;
		bt24_tx_buf[BT24_FRAME_ADDRH] = BT24_TX_ADDRH;
		bt24_tx_buf[BT24_FRAME_ADDRL] = BT24_TX_ADDRL;
		bt24_tx_buf[BT24_FRAME_CMDTYPE] = BT24_TX_DP_CMD;
		bt24_tx_buf[BT24_FRAME_LENGTH] = 0x2c;
		bt24_tx_buf[BT24_FRAME_DATATYPE] = 0;
		bt24_tx_buf[BT24_FRAME_STATE] = bat_type;
	}
	else if(bat_type==2)
	{
		bt24_tx_buf[BT24_FRAME_FIRST] = BT24_TX_FIRST;
		bt24_tx_buf[BT24_FRAME_ADDRH] = BT24_TX_ADDRH;
		bt24_tx_buf[BT24_FRAME_ADDRL] = BT24_TX_ADDRL;
		bt24_tx_buf[BT24_FRAME_CMDTYPE] = BT24_TX_DP_CMD;
		bt24_tx_buf[BT24_FRAME_LENGTH] = 0x3a;
		bt24_tx_buf[BT24_FRAME_DATATYPE] = 0;
		bt24_tx_buf[BT24_FRAME_STATE] = bat_type;
	}
	bmsOneBusHandler(batRxBuff);
	/*
	读取一线通/485/CAN 或者检测仪数据
	*/
	//回路电流
	bt24_tx_buf[32]=get_device_cur_value()>>8;	
	bt24_tx_buf[33]=get_device_cur_value();
	//最大电芯
	//最小电芯
	//压差
	//最大温度
	//最小温度
	//电池总压
	//电池总压(检测值)
	//外总压
	//SOC
	//SOH
	//最大充电电流
	//最大放电电流
	//故障码
	//从主机得到
	

	bt24_tx_buf[48]=get_device_vol_value();
	check_crc = get_check_crc8(bt24_tx_buf, (bt24_tx_buf[BT24_FRAME_LENGTH]+5), 7);
	bt24_tx_buf[bt24_tx_buf[BT24_FRAME_LENGTH] + 5] = check_crc;
	atc_transmit(&atc, bt24_tx_buf, bt24_tx_buf[BT24_FRAME_LENGTH] + 6);
}
/*****************************************************************************
函数名称 : device_start_dc_handler
功能描述 :	 放电通知
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void device_start_dc_handler(uint8_t offset,uint8_t *batRxBuff)
{
	uint8_t check_crc;
	uint8_t ch_over;
	bt24_tx_buf[BT24_FRAME_FIRST] = BT24_TX_FIRST;
	bt24_tx_buf[BT24_FRAME_ADDRH] = BT24_TX_ADDRH;
	bt24_tx_buf[BT24_FRAME_ADDRL] = BT24_TX_ADDRL;
	bt24_tx_buf[BT24_FRAME_CMDTYPE] = BT24_TX_DC_CMD;
	bt24_tx_buf[BT24_FRAME_LENGTH] = 0x02;
	bt24_tx_buf[BT24_FRAME_DATATYPE] = 0;
	bt24_tx_buf[BT24_FRAME_STATE] = 0x01;
	check_crc = get_check_crc8(bt24_tx_buf, (bt24_tx_buf[BT24_FRAME_LENGTH]+5), 7);
	bt24_tx_buf[bt24_tx_buf[BT24_FRAME_LENGTH] + 5] = check_crc;

	BSE.OPERA_STA=bt24_rx_buf[BT24_SET_OPERA_STA];
	BSE.DSG_CUR=(bt24_rx_buf[offset+BT24_SET_DSG_CUR]<<8)|bt24_rx_buf[offset+BT24_SET_DSG_CUR+1];
	BSE.DSG_END_VOL=(bt24_rx_buf[offset+BT24_SET_DSG_END_VOL]<<24)|(bt24_rx_buf[offset+BT24_SET_DSG_END_VOL+1]<<16)
	|(bt24_rx_buf[offset+BT24_SET_DSG_END_VOL+2]<<8)|(bt24_rx_buf[offset+BT24_SET_DSG_END_VOL+3]);
	BSE.CH_CUR=(bt24_rx_buf[offset+BT24_SET_CH_CUR]<<8)|bt24_rx_buf[offset+BT24_SET_CH_CUR+1];
	BSE.CH_END_VOL=(bt24_rx_buf[offset+BT24_SET_CH_END_VOL]<<24)|(bt24_rx_buf[offset+BT24_SET_CH_END_VOL+1]<<16)
	|(bt24_rx_buf[offset+BT24_SET_CH_END_VOL+2]<<8)|(bt24_rx_buf[offset+BT24_SET_CH_END_VOL+3]);
	BSE.DataType=bt24_rx_buf[offset+BT24_SET_DataType];

	atc_transmit(&atc, bt24_tx_buf, bt24_tx_buf[BT24_FRAME_LENGTH] + 6);
}
/*****************************************************************************
函数名称 : device_get_dc_handler
功能描述 :	询问放电结果
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void device_get_dc_handler(uint8_t offset,uint8_t *batRxBuff)
{
	uint8_t check_crc;
	uint8_t bat_type;
	bat_type=get_onebus_bat_type();
	//  bat_type = 2;
	if(bat_type==1)
	{
		bt24_tx_buf[BT24_FRAME_FIRST] = BT24_TX_FIRST;
		bt24_tx_buf[BT24_FRAME_ADDRH] = BT24_TX_ADDRH;
		bt24_tx_buf[BT24_FRAME_ADDRL] = BT24_TX_ADDRL;
		bt24_tx_buf[BT24_FRAME_CMDTYPE] = BT24_TX_GET_CMD;
		bt24_tx_buf[BT24_FRAME_LENGTH] = 0x2d;
		bt24_tx_buf[BT24_FRAME_DATATYPE] = 0;
		bt24_tx_buf[BT24_FRAME_STATE] = bat_type;
	}
	else if(bat_type==2)
	{
		bt24_tx_buf[BT24_FRAME_FIRST] = BT24_TX_FIRST;
		bt24_tx_buf[BT24_FRAME_ADDRH] = BT24_TX_ADDRH;
		bt24_tx_buf[BT24_FRAME_ADDRL] = BT24_TX_ADDRL;
		bt24_tx_buf[BT24_FRAME_CMDTYPE] = BT24_TX_GET_CMD;
		bt24_tx_buf[BT24_FRAME_LENGTH] = 0x3b;
		bt24_tx_buf[BT24_FRAME_DATATYPE] = 0;
		bt24_tx_buf[BT24_FRAME_STATE] = bat_type;
	}
	bmsOneBusHandler(batRxBuff);
	/*
	读取一线通/485/CAN 或者检测仪数据
	*/

	check_crc = get_check_crc8(bt24_tx_buf, (bt24_tx_buf[BT24_FRAME_LENGTH]+5), 7);
	bt24_tx_buf[bt24_tx_buf[BT24_FRAME_LENGTH] + 5] = check_crc;

	atc_transmit(&atc, bt24_tx_buf, bt24_tx_buf[BT24_FRAME_LENGTH] + 6);
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
	case APP_CHECK_CMD: //检测校验
		/* code */
		get_app_check_updata(offset,bat_rx_buf);
		break;
	case DEVICE_PARAM_CMD: //获取数据
		get_device_param_handler(offset,bat_rx_buf);
		break;
	case START_DC_CMD: //放电通知
		device_start_dc_handler(offset,bat_rx_buf);
		break;
	case GET_DC_CMD: //得到放电结果
		device_get_dc_handler(offset,bat_rx_buf);
		break;
		// APP扫码电池后，传输电池充放电信息
#ifdef SUPPORT_MCU_FIRM_UPDATE
	case GET_FW_CMD: //固件升级

		break;
	case FW_UPDATA_CHECK:

		break;
	case FW_UPDATA_CONTENT:

		break;
	case FW_UPDATA_OVER:

		break;
	case FW_UPDATA_RESULT:

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
	uint8_t rx_value_len = 0;             //数据帧长度
	uint8_t crc;
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
		rx_value_len=bt24_rx_buf[offset + BT24_FRAME_LENGTH];
		if((rx_in - offset) < (rx_value_len+PROTOCOL_HEAD_LEN))
		{
			break;
		}
		if (get_check_crc8(bt24_rx_buf, rx_in - 1, CRC_KEY) != bt24_rx_buf[rx_value_len+PROTOCOL_HEAD_LEN + offset - 1])
		{
			//校验出错
			offset += 3;
			crc=get_check_crc8(bt24_rx_buf, rx_in - 1, CRC_KEY);
			SEGGER_RTT_printf(0, "bt24_crc_check_error!\n");
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
 * @brief  	从APP得到电池状态
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_status(void)
{

}
/**
 * @brief  	从APP得到电芯体系
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_core(void)
{
}
/**
 * @brief  	从APP得到电池规格
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_spec(void)
{
}
/**
 * @brief  	从APP得到电池容量
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_cap(void)
{

}
/**
 * @brief  	从APP得到电池SN
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_sn(void)
{

}
/**
 * @brief  	下发电池工作状态
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_opera_status(void)
{
	if(GPIO_ReadInputDataBit(BLE_STA_PORT,BLE_STA_PIN))
	{
		return BSE.OPERA_STA;
	}
	return 0xAA;	
}
/**
 * @brief  	设置检测模式
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_det_mode(void)
{

}
/**
 * @brief  	设置放电电流
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint16_t bt24_get_bat_disch_cur(void) 
{
	return BSE.DSG_CUR;
} 
/**
 * @brief  	设置放电终止电压
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint32_t bt24_get_bat_disch_end_vol(void)
{
	return BSE.DSG_END_VOL;
}
/**
 * @brief  	设置充电电流
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint16_t bt24_get_bat_ch_cur(void)
{
	return BSE.CH_CUR;
}
/**
 * @brief  	设置充电终止电压
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint32_t bt24_get_bat_ch_end_vol(void)
{
	return BSE.CH_END_VOL;
}
/**
 * @brief  	设置充电终止电流
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_ch_end_cur(void)
{
}
/**
 * @brief  	设置工作模式
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t bt24_get_bat_set_mode(void)
{
	return	BSE.DataType;
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
 * @brief  	蓝牙名称FLASH查询
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t DXBT24_Device_ID_Read(uint16_t BLE_ID)
{
	Flash_Read(BLE_ID_SAVE_ADDR,BDN.BLE_ID_BUFF,sizeof(BDN.BLE_ID_BUFF));
	if(((BDN.BDB.BLE_ID_H<<8)|BDN.BDB.BLE_ID_L)!=BLE_ID) 
	{
		BDN.BDB.BLE_MARK=0x55;
		BDN.BDB.BLE_ID_H=BLE_ID>>8;
		BDN.BDB.BLE_ID_L=BLE_ID;
		Flash_Write(BLE_ID_SAVE_ADDR,BDN.BLE_ID_BUFF,sizeof(BDN.BLE_ID_BUFF));
		SEGGER_RTT_printf(0,"BLE_ID_RELOAD %x%x!\r\n",BDN.BDB.BLE_ID_H,BDN.BDB.BLE_ID_H);
		SEGGER_RTT_printf(0,"BLE_RESET!\r\n");
		return 1;
	}
	SEGGER_RTT_printf(0,"BLE_ID %x!\r\n",BLE_ID);
	return 0;
}
/**
 * @brief  	BT24蓝牙AT初始化 轮询
 * @param	500ms轮询
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
uint8_t DXBT24_AT_Init(uint8_t *name,uint8_t name_len)
{
	char echo_buf[20]; 
	uint16_t id;
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
				// ConfigModuleNoBlockFlage = 1;
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
			id=get_device_send_ble_id();
			if(DXBT24_Device_ID_Read(id))	//蓝牙名称需要重写并重启
			{
				for (int i = 0; i < name_len; i++)
				{
					echo_buf[i]=name[i];			
				}					
				__hex2str((uint8_t*)&id,(char*)(echo_buf+8),2);
				if (DXBT24_Set_Name(echo_buf) == 1)	//写入蓝牙名称
				{
					DataReturnFlage = 1;				
				}
			}
			else
			{
				DataReturnFlage = 1;
				ConfigModuleNoBlockFlage = 1;
			}		
			// else
			// {
			// 	SEGGER_RTT_printf(0,"BLE disconnected from device,please check!\r\n");
			// }		
			break;
		case 2:	//蓝牙重启
			if (atc_command(&atc, "AT+RESET\r\n", 1000, echo_buf, 20, 1, "OK"))
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
	DX_BT24_Init();	
	bt24_protocol_init();
	atc_init(&atc, "MY_ATC", 2, atc_found);
	atc_addSearch(&atc, "\r\n");
	// atc_command(&atc,"AT\r\n",3000,echo_buf,20,1,"OK");
	for (;;)
	{
		DXBT24_AT_Init(BLE_NAME,sizeof(BLE_NAME));
		atc_loop(&atc);
		bt24_recv_service(&atc);
	}
}
