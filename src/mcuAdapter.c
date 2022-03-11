#include "includes.h"


static uint8_t FlagRtcWakeUp = 0;


#define VREF_VAL (2048)
float ADC_Ratio = (3300.0) / 4095;

void MCU_SetNVOffset()
{
    if (VECT_OFFSET != 0 && __VTOR_PRESENT == 1)
    { //需要设置中断地址的偏移
        SCB->VTOR = 0x8000000 | VECT_OFFSET;
    }
}

void GPIOInit(void)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

    //所有管脚都初始化为浮动输入
    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Input;
 
    //输出管脚配置
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

#if(PROJECT_ID == 2)    
    /// BKEN
    GPIO_InitStructure.Pin = TN_BK_EN_PIN;
    GPIO_InitPeripheral(TN_BK_EN_PORT, &GPIO_InitStructure);
  
    GPIO_InitStructure.Pin = TN_LED2_PIN;
    GPIO_InitPeripheral(TN_LED2_PORT, &GPIO_InitStructure);

#endif


 
    GPIO_InitStructure.Pin = TN_VPRO_CON_PIN;
    GPIO_InitPeripheral(TN_VPRO_CON_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_485_TXEN_PIN;
    GPIO_InitPeripheral(TN_485_TXEN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_SHSHIP_PIN;
    GPIO_InitPeripheral(TN_SHSHIP_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_NTC0_POWER_PIN;
    GPIO_InitPeripheral(TN_NTC0_POWER_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_NTC1_POWER_PIN;
    GPIO_InitPeripheral(TN_NTC1_POWER_PORT, &GPIO_InitStructure);

    

    GPIO_InitStructure.Pin = TN_ONE_TX_PIN;
    GPIO_InitPeripheral(TN_ONE_TX_PORT, &GPIO_InitStructure);

  
    GPIO_ResetBits(TN_VPRO_CON_PORT, TN_VPRO_CON_PIN);
    GPIO_ResetBits(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
    GPIO_ResetBits(TN_NTC1_POWER_PORT, TN_NTC1_POWER_PIN);
    GPIO_SetBits(TN_ONE_TX_PORT,TN_ONE_TX_PIN);
    


    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_InitStructure.Pin = TN_LED_PIN;
    GPIO_InitPeripheral(TN_LED_PORT, &GPIO_InitStructure);
    GPIO_SetBits(TN_LED_PORT,TN_LED_PIN);
  

    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    //GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_I2C2;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_High;

    GPIO_InitStructure.Pin = TN_I2C1_SCL_PIN|TN_I2C1_SDA_PIN;
    GPIO_InitPeripheral(TN_I2C1_SCL_PORT, &GPIO_InitStructure);

    //GPIO_InitStructure.Pin = TN_I2C1_SDA_PIN;
    //GPIO_InitPeripheral(TN_I2C1_SDA_PORT, &GPIO_InitStructure);

#if(USE_485_IF == 1)

    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_InitStructure.Pin = TN_485_TXEN_PIN;
    GPIO_InitPeripheral(TN_485_TXEN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_485_COMM_ON_PIN;
    GPIO_InitPeripheral(TN_485_COMM_ON_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_COMM_ON2_PIN;
    GPIO_InitPeripheral(TN_COMM_ON2_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(TN_485_TXEN_PORT, TN_485_TXEN_PIN);
    GPIO_SetBits(TN_485_COMM_ON_PORT, TN_485_COMM_ON_PIN);
    
    //485 中断
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Input;
    GPIO_InitStructure.Pin = TN_485_INT_PIN;
    GPIO_InitPeripheral(TN_485_INT_PORT, &GPIO_InitStructure);



#endif

}

/**
 * @brief  Selects PLL clock as System clock source and configure HCLK, PCLK2
 *         and PCLK1 prescalers.
 */
void SetSysClockToPLL(uint32_t freq, uint32_t RCC_SYSCLKSource)
{
    ErrorStatus HSEStartUpStatus;
    ErrorStatus HSIStartUpStatus;

    uint32_t pllsrcclk;
    uint32_t pllsrc;
    uint32_t pllmul;
    uint32_t plldiv = RCC_PLLDIVCLK_DISABLE;
    uint32_t latency;
    uint32_t pclk1div, pclk2div;
    uint32_t msi_ready_flag = RESET;
    uint8_t src = SYSCLK_PLLSRC_HSI;
    if (HSE_VALUE != 8000000)
    {
        /* HSE_VALUE == 8000000 is needed in this project! */
        while (1);
    }

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration
     * -----------------------------*/

    if ((src == SYSCLK_PLLSRC_HSI)         || (src == SYSCLK_PLLSRC_HSIDIV2) 
     || (src == SYSCLK_PLLSRC_HSI_PLLDIV2) || (src == SYSCLK_PLLSRC_HSIDIV2_PLLDIV2))
    {
        /* Enable HSI */
        RCC_ConfigHsi(RCC_HSI_ENABLE);

        /* Wait till HSI is ready */
        HSIStartUpStatus = RCC_WaitHsiStable();

        if (HSIStartUpStatus != SUCCESS)
        {
            /* If HSI fails to start-up, the application will have wrong clock
               configuration. User can add here some code to deal with this
               error */

            /* Go to infinite loop */
            while (1);
        }

        if ((src == SYSCLK_PLLSRC_HSIDIV2) || (src == SYSCLK_PLLSRC_HSIDIV2_PLLDIV2))
        {
            pllsrc = RCC_PLL_HSI_PRE_DIV2;
            pllsrcclk = HSI_VALUE/2;

            if(src == SYSCLK_PLLSRC_HSIDIV2_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSI_VALUE/4;
            }
        } else if ((src == SYSCLK_PLLSRC_HSI) || (src == SYSCLK_PLLSRC_HSI_PLLDIV2))
        {
            pllsrc = RCC_PLL_HSI_PRE_DIV1;
            pllsrcclk = HSI_VALUE;

            if(src == SYSCLK_PLLSRC_HSI_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSI_VALUE/2;
            }
        }

    } else if ((src == SYSCLK_PLLSRC_HSE)         || (src == SYSCLK_PLLSRC_HSEDIV2) 
            || (src == SYSCLK_PLLSRC_HSE_PLLDIV2) || (src == SYSCLK_PLLSRC_HSEDIV2_PLLDIV2))
    {
        /* Enable HSE */
        RCC_ConfigHse(RCC_HSE_ENABLE);

        /* Wait till HSE is ready */
        HSEStartUpStatus = RCC_WaitHseStable();

        if (HSEStartUpStatus != SUCCESS)
        {
            /* If HSE fails to start-up, the application will have wrong clock
               configuration. User can add here some code to deal with this
               error */

            /* Go to infinite loop */
            while (1);
        }

        if ((src == SYSCLK_PLLSRC_HSEDIV2) || (src == SYSCLK_PLLSRC_HSEDIV2_PLLDIV2))
        {
            pllsrc = RCC_PLL_SRC_HSE_DIV2;
            pllsrcclk = HSE_VALUE/2;

            if(src == SYSCLK_PLLSRC_HSEDIV2_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSE_VALUE/4;
            }
        } else if ((src == SYSCLK_PLLSRC_HSE) || (src == SYSCLK_PLLSRC_HSE_PLLDIV2))
        {
            pllsrc = RCC_PLL_SRC_HSE_DIV1;
            pllsrcclk = HSE_VALUE;

            if(src == SYSCLK_PLLSRC_HSE_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSE_VALUE/2;
            }
        }
    }

    latency = (freq/32000000);
    
    if(freq > 54000000)
    {
        pclk1div = RCC_HCLK_DIV4;
        pclk2div = RCC_HCLK_DIV2;
    }
    else
    {
        if(freq > 27000000)
        {
            pclk1div = RCC_HCLK_DIV2;
            pclk2div = RCC_HCLK_DIV1;
        }
        else
        {
            pclk1div = RCC_HCLK_DIV1;
            pclk2div = RCC_HCLK_DIV1;
        }
    }
    
    if(((freq % pllsrcclk) == 0) && ((freq / pllsrcclk) >= 2) && ((freq / pllsrcclk) <= 32))
    {
        pllmul = (freq / pllsrcclk);
        if(pllmul <= 16)
        {
            pllmul = ((pllmul - 2) << 18);
        }
        else
        {
            pllmul = (((pllmul - 17) << 18) | (1 << 27));
        }
    }
    else
    {

        while(1);
    }

    /* Cheak if MSI is Ready */
    if(RESET == RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_MSIRD))
    {
        /* Enable MSI and Config Clock */
        RCC_ConfigMsi(RCC_MSI_ENABLE, RCC_MSI_RANGE_4M);
        /* Waits for MSI start-up */
        while(SUCCESS != RCC_WaitMsiStable());

        msi_ready_flag = SET;
    }

    /* Select MSI as system clock source */
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_MSI);

    FLASH_SetLatency(latency);

    /* HCLK = SYSCLK */
    RCC_ConfigHclk(RCC_SYSCLKSource);//RCC_SYSCLK_DIV1

    /* PCLK2 = HCLK */
    RCC_ConfigPclk2(pclk2div);

    /* PCLK1 = HCLK */
    RCC_ConfigPclk1(pclk1div);

    /* Disable PLL */
    RCC_EnablePll(DISABLE);

    RCC_ConfigPll(pllsrc, pllmul, plldiv);

    /* Enable PLL */
    RCC_EnablePll(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_CTRL_FLAG_PLLRDF) == RESET);

    /* Select PLL as system clock source */
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSysclkSrc() != 0x0C);

    if(msi_ready_flag == SET)
    {
        /* MSI oscillator OFF */
        RCC_ConfigMsi(RCC_MSI_DISABLE, RCC_MSI_RANGE_4M);
    }
}

void Clk_Config(void)
{
    //n32l406x 时钟配置
    //SetSysClockToPLL(64000000, RCC_SYSCLK_DIV1);
}

void Iwdt_Config(void)
{
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteConfig(IWDG_WRITE_ENABLE);

    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV256); //IWDG_PRESCALER_DIV256

    /* Set counter reload value to obtain 250ms IWDG TimeOut.
       Counter Reload Value = 1000
                            f = 40k/256 = 156.25
                            period = 3000*1/156.25 = 4096*0.0064 = 19.2s                            
     */
    IWDG_CntReload(3000); //LSI = 40K
    /* Reload IWDG counter */
    IWDG_ReloadKey();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
}

void Iwdt_Start(void)
{
    IWDG_Enable();
}

void Iwdt_Feed(void)
{
    IWDG_ReloadKey();
}

//-----------------------------RTC----------------------------------------------
void RTC_ConfigInit(void)
{
    RTC_DateAndTimeDefaultVale();
    RTC_CLKSourceConfig(3, 0, 1);
    RTC_PrescalerConfig();

    RTC_DateRegulate();
    RTC_TimeRegulate();

    WakeUpClockSelect(1);
    /* wake up timer value */
    //RTC_SetWakeUpCounter(25000);

    EXTI20_RTCWKUP_Configuration(ENABLE);
    /* Enable the RTC Wakeup Interrupt */
    RTC_ConfigInt(RTC_INT_WUT, ENABLE);
    RTC_EnableWakeUp(ENABLE);
    RTC_SetWakeUpCounter(25000);
}

uint16_t rtcCount = 0;
void RTC_WKUP_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_INT_WUT) != RESET)
    {
        RTC_ClrIntPendingBit(RTC_INT_WUT);
        EXTI_ClrITPendBit(EXTI_LINE20);
        
        // //RTC 唤醒之后，需要根据进入中断的类型
        // if(flagIntEnterType == 0x02)
        // {
        //     //深度休眠
        //     //只清楚WDT，然后继续休眠
        //     Iwdt_Feed();
        //     SCB->SCR |= 0x02;
        // }else
        // if(flagIntEnterType == 0x01)
        // {
        //     //唤醒之后退出中断，继续跑
        //     //SCB->SCR &= ~0x02;
        // }

        // FlagRtcWakeUp = 0x01;
        // rtcCount++;
    }
}

uint8_t Rtc_GetWakeUpFlag(void)
{
    return FlagRtcWakeUp;
}

void Rtc_ClrWakeUpFlag(void)
{
    FlagRtcWakeUp = 0;
}

uint16_t Rtc_GetCount(){
    return rtcCount;
}

void Rtc_SetCount(uint16_t val)
{
    rtcCount = 0;
}

void RTC_AlarmITEnable(void)
{
    RTC_ConfigInt(RTC_INT_WUT,ENABLE);
}

void RTC_AlarmITDisable(void)
{
    RTC_ConfigInt(RTC_INT_WUT,DISABLE);
}

//-----------------------------END RTC----------------------------------------------



///< ADC 采样端口初始化
void ADC_PortInit(void)
{
}

void ADC_DeConfig(void)
{
    ADC_DeInit(ADC);
    ADC_Enable(ADC, DISABLE);
}

uint16_t ADC_GetChannleVal(uint8_t ADC_Channel)
{
    uint16_t dat;

    ADC_ConfigRegularChannel(ADC, ADC_Channel, 1, ADC_SAMP_TIME_55CYCLES5);
    /* Start ADC Software Conversion */
    ADC_EnableSoftwareStartConv(ADC, ENABLE);
    while (ADC_GetFlagStatus(ADC, ADC_FLAG_ENDC) == 0)
    {
    }
    ADC_ClearFlag(ADC, ADC_FLAG_ENDC);
    ADC_ClearFlag(ADC, ADC_FLAG_STR);
    dat = ADC_GetDat(ADC);
    return dat;
}

void ADC_Config(void)
{
    ADC_InitType ADC_InitStructure;
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    /* RCC_ADCHCLK_DIV16*/
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    //RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8); //selsect HSE as RCC ADC1M CLK Source

    /* ADC configuration ------------------------------------------------------*/
    ADC_InitStructure.MultiChEn = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = 1;
    ADC_Init(ADC, &ADC_InitStructure);
    /* Enable ADC */
    ADC_Enable(ADC, ENABLE);
    /* Check ADC Ready */
    while (ADC_GetFlagStatusNew(ADC, ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC);
    /* Check the end of ADC1 calibration */
    while (ADC_GetCalibrationStatus(ADC))
        ;
}

void ADC_GetMcuAdcInfo(uint16_t *buf, uint16_t *mosTemp, uint16_t *balTemp, uint16_t *preDsgCur)
{

    //开启温探power
    GPIO_SetBits(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
    bsp_DelayMS(10);

    *buf = ADC_GetChannleVal(ADC_CH_0);             //PMATCH,PB12 AdcExInputCH19
    *(buf + 1) = ADC_GetChannleVal(ADC_CH_4);       //balance temp ,PA07 ，NTC1
    *(buf + 2) = ADC_GetChannleVal(ADC_CH_5);       //mos temp ,PB00 ，NTC
    //*(buf + 3) = ADC_GetChannleVal(ADC_CH_VREFINT); //2.048V
    *(buf + 4) = ADC_GetChannleVal(ADC_CH_10);      //PB1 ,pre_Isen_p
    *(buf + 5) = 0;                                 // ADC_GetChannleVal(ADC_CH_4);   //PB1 ,pre_Isen_n

    //use vrefint calibrate the VDD as vref
    //ADC_Ratio = (float)VREF_VAL / *(buf + 3);
    *mosTemp = CalcuTemp((uint16_t)(*(buf + 2) * ADC_Ratio));
    *balTemp = CalcuTemp((uint16_t)(*(buf + 1) * ADC_Ratio));
    
    *preDsgCur = (uint16_t)(*(buf + 4) * ADC_Ratio); 
    //关闭 temperature power
    GPIO_ResetBits(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
}

/**
 * @brief  Initializes peripherals
 * @param  None
 * @retval None
 */
void TWI_Init(void)
{
/*    I2C_InitType i2c1_master;
    GPIO_InitType i2c1_gpio;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C2, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    I2C_DeInit(TWI_I2C);
    i2c1_master.BusMode = I2C_BUSMODE_I2C;
    i2c1_master.FmDutyCycle = I2C_FMDUTYCYCLE_2;
    i2c1_master.OwnAddr1 = 0x55;
    i2c1_master.AckEnable = I2C_ACKEN;
    i2c1_master.AddrMode = I2C_ADDR_MODE_7BIT;
    i2c1_master.ClkSpeed = I2C_SPEED; // 100K

    I2C_Init(TWI_I2C, &i2c1_master);
    I2C_Enable(TWI_I2C, ENABLE);
    */
   //use io simulate
}

//------------------------------------------------------------------
//模块内部函数开始
//------------------------------------------------------------------
#define SCK_H() MCU_GPIO_SetBit(TN_I2C1_SCL_PORT, TN_I2C1_SCL_PIN)
#define SCK_L() MCU_GPIO_ClrBit(TN_I2C1_SCL_PORT, TN_I2C1_SCL_PIN)

#define SDA_H() MCU_GPIO_SetBit(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN)
#define SDA_L() MCU_GPIO_ClrBit(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN)


static uint8_t READ_SDA(void)
{
    uint8_t bit = 0;
    
    //gpio_init(GPIOE,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_5);
    //Hal_DelayUS(2);
    
    //bsp_DelayUS(5);
    bit = MCU_GPIO_GetBit(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN);
    //gpio_init(GPIOE, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_5);
    return bit;
}

static void TWI_IIC_Start(void)
{
    
    // SCK_H();
    // SDA_H();
    // bsp_DelayUS(30);
    // SDA_L();
    // bsp_DelayUS(30);
    // SCK_L();
    // SCK_H();
    // SDA_L();
    // bsp_DelayUS(10);
    SDA_H();
    SCK_H();    
    bsp_DelayUS(100);
    SDA_L();
    bsp_DelayUS(100);
    SCK_L();
    // bsp_DelayUS(10);
}

static void TWI_IIC_Stop(void)
{
    SDA_L();
    SCK_H();
    bsp_DelayUS(10);
    SDA_H();
}

static uint8_t TWI_IIC_Ack(void)
{
    uint8_t ack = 0;
    //check ack
    //SWITCH_SDA_MODE(1);
    SCK_L();
    bsp_DelayUS(10);
    SCK_H();    
    bsp_DelayUS(10);
    ack = READ_SDA();
    //SWITCH_SDA_MODE(0);
    bsp_DelayUS(5);
    SCK_L();
    bsp_DelayUS(10);
    return ack;
}

static void TWI_IIC_SendAck(uint8_t isAck)
{
    if(isAck)
    {
        SDA_L();
    }else
    {
        SDA_H();
    }
    bsp_DelayUS(10);  
    SCK_H();     
    bsp_DelayUS(10);
    SCK_L();
    bsp_DelayUS(1);
    SDA_H();
}

static void TWI_IIC_WriteByte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if ((byte & 0x80) == 0x80)
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        bsp_DelayUS(10);
        SCK_H();
        bsp_DelayUS(10);
        SCK_L();
        bsp_DelayUS(10);
        byte <<= 1;
    }
    SDA_H();
    bsp_DelayUS(5);
}

static uint8_t TWI_IIC_ReadByte()
{
    uint8_t i;
    uint8_t byte;

    for (i = 0; i < 8; i++)
    {
        byte <<= 1;
        SCK_L();
        bsp_DelayUS(10);
        SCK_H();
        bsp_DelayUS(10);
        if (READ_SDA() == SET)
        {
            byte |= 0x01;
        }
        bsp_DelayUS(10);
    }
    SCK_L();
    SDA_H();
    return byte;
}

static uint8_t TWI_WriteRegByte(uint8_t addr, uint8_t byte)
{
    uint8_t retCode = 0;
    TWI_IIC_Start();
    TWI_IIC_WriteByte(addr);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
    }
    TWI_IIC_WriteByte(byte);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
    }
    TWI_IIC_Stop();
    return retCode;
}

/**
 ******************************************************************************
 ** \brief  主机接收函数
 **
 ** \param u8Addr从机内存地址，pu8Data读数据存放缓存，u32Len读数据长度
 **
 ** \retval 读数据是否成功
 **
 ******************************************************************************/
uint8_t I2C_MasterReadData(uint8_t SlaveID, uint8_t u8Addr, uint8_t *pu8Data, uint32_t u32Len)
{
    uint8_t retCode = 0;
    uint8_t i;
    TWI_IIC_Start();
    TWI_IIC_WriteByte(SlaveID);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //SEGGER_RTT_printf(0,"ACK ERROR1\n");
    }
    TWI_IIC_WriteByte(u8Addr);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //SEGGER_RTT_printf(0,"ACK ERROR2\n");
    }
#if(AFE_CHIP_SELECT == 3)
#else
    TWI_IIC_WriteByte((uint8_t)u32Len);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //SEGGER_RTT_printf(0,"ACK ERROR3\n");
    }
#endif
    TWI_IIC_Start();
    //read
    TWI_IIC_WriteByte((SlaveID|0x01));//(SlaveID|0x01)
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //SEGGER_RTT_printf(0,"ACK ERROR4\n");
    }

    for(i = 0;i<u32Len;i++)
    {
        *(pu8Data+i) = TWI_IIC_ReadByte();
        //SEGGER_RTT_printf(0,"%02x ",*(pu8Data+i));
        //send ack
        TWI_IIC_SendAck(1);

    }
    //SEGGER_RTT_printf(0,"\n");
    //read crc
    *(pu8Data+i) = TWI_IIC_ReadByte();
    //send nack
    TWI_IIC_SendAck(0);

    TWI_IIC_Stop();
    return retCode;
}
/**
 ******************************************************************************
 ** \brief  主机发送函数
 **
 ** \param u8Addr从机内存地址，pu8Data写数据，u32Len写数据长度
 **
 ** \retval 写数据是否成功
 **
 ******************************************************************************/
uint8_t I2C_MasterWriteData(uint8_t SlaveID, uint8_t u8Addr, uint8_t *pu8Data, uint32_t u32Len)
{
    uint8_t retCode = 0;
    uint8_t i;
    TWI_IIC_Start();
    TWI_IIC_WriteByte(SlaveID);
    if (TWI_IIC_Ack())
    {
        //SEGGER_RTT_printf(0,"I2C_MasterWriteData ACK ERROR1\n");
        retCode = 1;
    }
    TWI_IIC_WriteByte(u8Addr);
    if (TWI_IIC_Ack())
    {
        //SEGGER_RTT_printf(0,"I2C_MasterWriteData ACK ERROR2\n");
        retCode = 1;
    }

    for(i = 0;i<u32Len;i++)
    {
        TWI_IIC_WriteByte(*(pu8Data+i));
        if (TWI_IIC_Ack())
        {
            //SEGGER_RTT_printf(0,"I2C_MasterWriteData ACK ERROR3\n");
            retCode = 1;
            break;
        }
    }
    TWI_IIC_Stop();
    return retCode;
}
