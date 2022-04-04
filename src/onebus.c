/******************************************************************************
  * @file           : onebus.c
  * @version        : v1.0
  * @author         : Azreal
  * @creat          : 2021 0804
******************************************************************************/

#include "includes.h"
#include "onebus.h"

void _smart_bms_init(bms_info *bms);

bms_info bms;
OneBusStaticData_t OBS;
TIM_TimeBaseInitType TIM_TimeBaseStructure;

#define PA_RX_LEN	51		//PA一线通接收数据包字节

#define bms_data		MCU_GPIO_GetBit(ONE_RXD1_PORT,ONE_RXD1_PIN)
#define TEST_PIN_HIGH()	MCU_GPIO_SetBit(TEST_IO_PORT,TEST_IO_PIN)
#define TEST_PIN_LOW()	MCU_GPIO_ClrBit(TEST_IO_PORT,TEST_IO_PIN)

#define ACC_ENABLE()	MCU_GPIO_SetBit(ACC_IO_PORT,ACC_IO_PIN)
#define ACC_DISABLE()	MCU_GPIO_ClrBit(ACC_IO_PORT,ACC_IO_PIN)

#define ONE_TXD1_HIGH()	MCU_GPIO_SetBit(ONE_TXD1_PORT,ONE_TXD1_PIN)
#define ONE_TXD1_LOW()	MCU_GPIO_ClrBit(ONE_TXD1_PORT,ONE_TXD1_PIN)

#define ONE_TXD2_HIGH()	MCU_GPIO_SetBit(ONE_TXD2_PORT,ONE_TXD2_PIN)
#define ONE_TXD2_LOW()	MCU_GPIO_ClrBit(ONE_TXD2_PORT,ONE_TXD2_PIN)

uint8_t bms_rx_buf[100];		//BMS接收协议保存区

uint16_t PrescalerValue = 0;

uint8_t bmstype;

uint32_t config_cnt;
/** 
* @brief  	bms一线通初始化
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void bmsOneBusInit(void)
{
    GPIO_InitType GPIO_InitStructure;

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

	GPIO_InitStruct(&GPIO_InitStructure);
    /* GPIOC Configuration:Pin6, 7, 8 and 9 as alternate function push-pull */
    GPIO_InitStructure.Pin        = ONE_RXD1_PIN | ONE_RXD2_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Input;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = GPIO_PIN_6|ONE_TXD1_PIN|ONE_TXD2_PIN;
	GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
	GPIO_InitStructure.GPIO_Pull    = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = GPIO_PIN_8;	//ACC_CTL
	GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
	GPIO_InitStructure.GPIO_Pull    = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;

    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

	ONE_TXD1_HIGH();
	ONE_TXD2_HIGH();
	// ACC_ENABLE();	//短接ACC和BAT+
	bmsOneBusParamInit(&bms);
}
/**
 * @brief  Configures tim1 clocks.
 */
void TIM_Configuration(uint16_t arr,uint16_t psc)
{
	/* PCLK1 = HCLK/4 */
    RCC_ConfigPclk2(RCC_HCLK_DIV1);

    /* TIM1 clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_TIM1, ENABLE);

    /* Compute the prescaler value */
    PrescalerValue = psc; //(uint16_t) (SystemCoreClock / 12000000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.Period    = arr;
    TIM_TimeBaseStructure.Prescaler = psc;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(TIM1, &TIM_TimeBaseStructure);

    /* Prescaler configuration */
    TIM_ConfigPrescaler(TIM1, PrescalerValue, TIM_PSC_RELOAD_MODE_IMMEDIATE);

    /* TIM1 enable update irq */
    TIM_ConfigInt(TIM1, TIM_INT_UPDATE, ENABLE);

    /* TIM1 enable counter */
    TIM_Enable(TIM1, ENABLE);

	NVIC_InitType NVIC_InitStructure;

    /* Enable the TIM1 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}
/** 
* @brief  	bms参数初始化
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void bmsOneBusParamInit(bms_info *bms)
{
	int i=0;
	bms->Receflag=0;
	bms->Bytecount=0;
	bms->Bitcount=0;  
	bms->Syncflage=0;
	bms->Sflage=0;
	bms->bmstate=0;
	bms->Synch=0;
	bms->Syncl=0;
	bms->buff0=0;
	bms->buff1=0;
	bms->buff2=0;
	bms->buff3=0; 
	for(i=0;i<100;i++)
	{
		bms->pb_list[i]=0;
	}
}
/** 
* @brief  	bms一线通协议解析
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void bmsOneBusCheckPA(bms_info *bms,uint8_t *bmsRxBuff)
{
	int i;
	switch(bms->bmstate)  
	{
		case bms_h:
			if(!bms_data)
			{
				bms->bmstate=bms_l;
				bms->pb_list[bms->Bytecount]>>=1;
				if(bms->bmscount<0)
				{ 
					bms->pb_list[bms->Bytecount]+=0x80;
				}
				if(bms->bmscount==0)
				{
					bms->bmstate=bmsreset;
				}
				bms->bmscount=0;
				if((++bms->Bitcount&7)==0)
				{
					bms->Bytecount++;
				}
				if(bms->Bitcount>=PA_RX_LEN*8)
				{
					for (int i = 0; i < PA_RX_LEN; i++)
					{
						/* code */
						bms_rx_buf[i]=bms->pb_list[i];
					}
					bms->Receflag=1;
					bms->bmstype=2;
					bms->bmscount=bmsreset;
				}
			}	
			else
			{
				bms->bmscount--;
				if(bms->bmscount<(-20))
				{
					bms->bmscount=bmsreset;
				}				
			}
			break;
		case bms_l:		
			if(!bms_data)
			{
				bms->bmscount++;
				if(bms->bmscount>20) 
				{
					bms->bmstate=bmsreset;
				}
			}
			else
			{
				bms->bmstate=bms_h;					
			}	
			break;
		case sync1:		
			if(!bms_data)
			{
				bms->Syncl++;
				bms->Syncflage=1;
			}
			if(bms->Syncflage)
			{
				if(bms_data)
				{
					bms->Sflage=1;
					if((bms->Syncl<pa_syncl_short)||(bms->Syncl>pa_syncl_long))
					{
						bms->bmstate=bmsreset;
					}
					bms->Synch++;
				}
			}
			if(bms->Sflage)
			{
				if(!bms_data)
				{					
					if((bms->Synch<pa_synch_short)||(bms->Synch>pa_synch_long))
					{
						bms->bmstate=bmsreset;
					}
					else
					{
						bms->bmstate=bms_l;												
					}
				}
			}
			break;
		case bmsreset:
			bms->bmstate=sync1; 
			bms->Synch=0;              
			bms->Syncl=0;              
			bms->Syncflage=0;          
			bms->Sflage=0;
			bms->bmscount=0;            
			bms->Bitcount=0;   
			bms->Bytecount=0; 	
		break;
	}
}
/** 
* @brief  	bms上报平台数据
* @param  	p_buf 数据上报缓存
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void bmsOneBusParamUpdata(bms_info *bms,uint8_t *bmsRxBuff)
{
	switch(bms->Receflag)
	{
		case 0:

		break;
		case 1:	//PA一线通
			if(bms->Receflag==1)
			{
				bms->Receflag=0;
				memcpy(bmsRxBuff,bms->pb_list,PA_RX_LEN);	
			}
		break;
	}
}
/** 
* @brief  	bms上报平台数据
* @param  	p_buf 数据上报缓存
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void bmsOneBusHandler(uint8_t *bmsRxBuff)
{
	bmsOneBusParamUpdata(&bms,bmsRxBuff);
}
/** 
* @brief  	bms上报电池状态
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_onebus_bat_sta(void)
{
	return bms.pb_list[2]>>4;
	// return OBS.BAT_STATUS;
}
/** 
* @brief  	bms上报电池总压
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_onebus_bat_max_vol(void)
{
	return bms.pb_list[14]<<8|bms.pb_list[15];
	// return OBS.BAT_CH_MAX_VOL;
}
/** 
* @brief  	bms上报电池类型
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_onebus_bat_type(void)
{
	return bms.bmstype;
}
/** 
* @brief  	bms上报电池最大充电电压
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_onebus_bat_max_ch_vol(void)
{
	return bms.pb_list[4]<<8|bms.pb_list[5];
}
/** 
* @brief  	bms上报电池最大充电电流
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_onebus_bat_max_ch_cur(void)
{
	return bms.pb_list[6]<<8|bms.pb_list[7];
}
/** 
* @brief  	bms上报电池最大放大电流
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_onebus_bat_max_dsg_cur(void)
{
	return bms.pb_list[10]<<8|bms.pb_list[11];
}
/** 
* @brief  	bms上报电池最高单体温度
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_onebus_bat_max_temp(void)
{
	return bms.pb_list[24];
}
/** 
* @brief  	bms上报电池最低单体温度
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint16_t get_onebus_bat_min_temp(void)
{
	return bms.pb_list[25];
}
/** 
* @brief  	bms上报电池SOC
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_onebus_bat_soc(void)
{
	return bms.pb_list[3];
}
/** 
* @brief  	bms上报电池SOH
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_onebus_bat_soh(void)
{
	return bms.pb_list[23];
}
/** 
* @brief  	配置时间清零
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint32_t GetConfigClear()
{

}
/** 
* @brief  	配置时间清零
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void GetConfigTimeClear()
{
	config_cnt=0;
}
/** 
* @brief  	得到配置时间
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint32_t GetConfigLoopTime()
{
	return config_cnt;
}
/**
 * @brief  This function handles TIM1 update interrupt request.
 * TIM1定时器100us
 */
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetIntStatus(TIM1, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM1, TIM_INT_UPDATE);
		// GPIO_ToggleBits(TEST_IO_PORT,TEST_IO_PIN); 
		config_cnt++;
		bmsOneBusCheckPA(&bms,bms_rx_buf); 
	}
}
/** 
* @brief  	IO口翻转测试定时器
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void GPIO_ToggleBits(GPIO_Module* GPIOx, uint16_t GPIO_Pin)
{
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));

  GPIOx->POD^= GPIO_Pin;
} 