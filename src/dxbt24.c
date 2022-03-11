#include "dxbt24.h"
#include "includes.h"

#define DXBT24_GET_LINK_STAT() GPIO_ReadInputDataBit(TN_BLE_STATUS_PORT,TN_BLE_STATUS_PIN)

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

	//mod管脚进入配置模式
    DXBT24_Rst();
	//DXBT24_SWITCH_MOD_AT();
	//bsp_DelayMS(300);
	//USART_SendData(&DXBT24_UARTX, "AT+RESET", strlen("AT+RESET"));
	bsp_DelayMS(300);
    DXBT24_SWITCH_MOD_TC();
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
        
        memset(cmd,0,48);
	//sprintf(cmd,"AT+NAME1=%s",name);
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
        //USART_SendDataPoll(&DXBT24_UARTX, "AT+NAME?",8);
	bsp_DelayMS(300);
	//USART_SendDataPoll(&DXBT24_UARTX, cmd, strlen((char *)cmd)+5);
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
	//USART_SendDataPoll(&DXBT24_UARTX, pData, length);
}

void DXBT24_ReceiveHandle(uint8_t *recvData, uint16_t length)
{

}