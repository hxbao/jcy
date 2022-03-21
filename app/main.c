/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"

static void Bsp_Init(void)
{
	GPIOInit();
	bsp_InitTimer();
}


void main(void)
{
	uint16_t newCrc;
	uint8_t count = 5;
	//中断地址偏移设置
//	MCU_SetNVOffset();
	Clk_Config();
	SysTick_Config(SystemCoreClock / 1000);

#if(MCU_LIB_SELECT ==1)

#elif(MCU_LIB_SELECT ==2)
	DBG_ConfigPeriph(DBG_STOP|DBG_IWDG_STOP, ENABLE);	
	//DBG_ConfigPeriph(DBG_STOP|DBG_IWDG_STOP, DISABLE);	
#endif
	Flash_Read(APP_CONFIG_CRC_ADDR, (uint8_t *)&newCrc, 2);	
	SEGGER_RTT_Init();
	//new fw version crc check
	SEGGER_RTT_printf(0, "new crc:%x\n", newCrc);
	SEGGER_RTT_printf(0, "SystemCoreClockFreq:%d\n", SystemCoreClock);
	SEGGER_RTT_printf(0, "version:%d\n", FW_VERSION);
	SEGGER_RTT_printf(0, "complie time:%s\n", COMPLIE_TIME);

	Bsp_Init();

	// bmsOneWireInit();
	// TIM_Configuration(47,99);	//(47+1)*(99+1)/48000000

	// atc_init(&atc, "MY_ATC", USART2,atc_found);
	// atc_addSearch(&atc, "\r\n+CMD:");

	// while(1)
	// {
	// 	atc_loop(&atc);
	// }
	CreateTask();
	osKernelStart();
}