/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"

typedef volatile uint32_t vu32;

void main(void)
{
	Flash_Read(APP_CONFIG_AREA_ADDR, (uint8_t *)&appBin, sizeof(AppBinHandle_t));
	while (1)
	{

		if (appBin.flag == 0xaa) //need updata appbin data
		{
			iap_write_appbin(IAP_APP_BIN_ADDR, appBin.srcFlashAddr, appBin.appBinByteSize);
			appBin.flag = 0x55;
			Flash_Write(APP_CONFIG_AREA_ADDR, (uint8_t *)&appBin, sizeof(AppBinHandle_t));
		}
		//default run to APP1
		if (((*(vu32 *)(IAP_APP_BIN_ADDR + 4)) & 0xFF000000) == FLASH_BASE_ADDR) //
		{
			iap_load_app(((uint32_t)IAP_APP_BIN_ADDR)); //jump to app
		}
	}
}