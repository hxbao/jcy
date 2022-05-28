/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"


uint8_t Uart0FlagUartInited = 0;

static uint8_t Atl485Flag;   //硬件485接口
static uint32_t Atl485BaudRate;
static uint8_t Atl485CheckFlag;

pf_RxCallback RxCallback;


//注册一个接收回调处理函数
#if(MCU_LIB_SELECT == 1)

static void Uart0PortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    DDL_ZERO_STRUCT(stcGpioCfg);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
 
    //<RX
    stcGpioCfg.enDir =  GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin10,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin10,GpioAf1); //配置PA01为LPUart0_RX
}

static void HandleRecvData(uint8_t data)
{
    RxCallback(data);
}

void Uart0Init(pf_RxCallback callback)
{
	stc_uart_cfg_t  stcCfg;

    DDL_ZERO_STRUCT(stcCfg);
    Uart0PortCfg();
    ///<外设模块时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);

   ///<LPUART 初始化
    stcCfg.enRunMode        = UartMskMode3;          ///<模式3
    stcCfg.enStopBit        = UartMsk1bit;           ///<1bit停止位
    stcCfg.enMmdorCk        = UartMskEven;           ///<检验
    stcCfg.stcBaud.u32Baud  = 9600;                  ///<波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       ///<通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); ///<获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART0, &stcCfg);                   ///<串口初始化
    
    Uart_ClrStatus(M0P_UART0,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART0,UartTC);                ///<清接收请求
    Uart_EnableIrq(M0P_UART0,UartRxIrq);             ///<使能串口接收中断
    EnableNvic(UART0_2_IRQn, IrqLevel0, TRUE);       ///<系统中断使能

    //接收回调函数
    RxCallback = callback;
}

#if(PROJECT_ID == 2)
void Uart0Init4TY(pf_RxCallback callback)
{
	stc_uart_cfg_t  stcCfg;

    DDL_ZERO_STRUCT(stcCfg);
    Uart0PortCfg();
    ///<外设模块时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);

   ///<LPUART 初始化
    stcCfg.enRunMode        = UartMskMode1;          ///<模式1
    stcCfg.enStopBit        = UartMsk1bit;           ///<1bit停止位
    stcCfg.enMmdorCk        = UartMskEven;           ///<检验
    stcCfg.stcBaud.u32Baud  = 9600;                  ///<波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       ///<通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); ///<获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART0, &stcCfg);                   ///<串口初始化
    
    Uart_ClrStatus(M0P_UART0,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART0,UartTC);                ///<清接收请求
    Uart_EnableIrq(M0P_UART0,UartRxIrq);             ///<使能串口接收中断
    EnableNvic(UART0_2_IRQn, IrqLevel0, TRUE);       ///<系统中断使能

    //接收回调函数
    RxCallback = callback;
}
#elif (PROJECT_ID == 4)
void Uart0Init4TY(pf_RxCallback callback)
{
	stc_uart_cfg_t  stcCfg;

    DDL_ZERO_STRUCT(stcCfg);
    Uart0PortCfg();
    ///<外设模块时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);

   ///<LPUART 初始化
    stcCfg.enRunMode        = UartMskMode1;          ///<模式1
    stcCfg.enStopBit        = UartMsk1bit;           ///<1bit停止位
    stcCfg.enMmdorCk        = UartMskEven;           ///<检验
    stcCfg.stcBaud.u32Baud  = 9600;                  ///<波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;       ///<通道采样分频配置
    stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq(); ///<获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART0, &stcCfg);                   ///<串口初始化
    
    Uart_ClrStatus(M0P_UART0,UartRC);                ///<清接收请求
    Uart_ClrStatus(M0P_UART0,UartTC);                ///<清接收请求
    Uart_EnableIrq(M0P_UART0,UartRxIrq);             ///<使能串口接收中断
    EnableNvic(UART0_2_IRQn, IrqLevel0, TRUE);       ///<系统中断使能

    //接收回调函数
    RxCallback = callback;
}
#endif

void UartReInit(void)
{
	// if (Uart0FlagUartInited == 1)
	// {
	//     Sysctrl_SetPeripheralGate(SysctrlPeripheralLpUart0,FALSE);
	//     Gpio_SetAfMode(GpioPortA,GpioPin9,GpioAf0); //配置PA00为LPUart0_TX
	//     Gpio_SetAfMode(GpioPortA,GpioPin10,GpioAf0); //配置PA00为LPUart0_RX
    //     Uart0FlagUartInited = 0 ;
	// }
    
}

void Uart0SendData(uint8_t *pData, uint16_t len)
{
	uint16_t i;

    Gpio_SetAfMode(GpioPortA,GpioPin9,GpioAf1); //配置PA00为LPUart0_TX
    Uart_DisableIrq(M0P_UART0,UartRxIrq); 
#if (USE_485_IF == 1)
    ENABLE_485_TX_PIN();
#endif
	for(i = 0;i<len;i++)
	{
        Uart_SendDataPoll(M0P_UART0, *(pData+i));
	}
    bsp_DelayUS(100); //由于需要在中断中调用，不使用bsp_DelayMS()函数
#if (USE_485_IF == 1)
    DISABLE_485_TX_PIN();
#endif
    Uart_ClrStatus(M0P_UART0,UartRC);
    Uart_EnableIrq(M0P_UART0,UartRxIrq); 
    Gpio_SetAfMode(GpioPortA,GpioPin9,GpioAf0);
}

///<Uart0 中断服务函数
void Uart0_IRQHandler(void)
{
    uint8_t data;
   
    if(Uart_GetStatus(M0P_UART0, UartRC))    ///接收数据
    {
        Uart_ClrStatus(M0P_UART0, UartRC);   ///<清接收中断请求
        data = Uart_ReceiveData(M0P_UART0);///读取数据
        HandleRecvData(data);  
    }
}
#elif(MCU_LIB_SELECT == 2) 

static void Uart1PortCfg(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);   

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_9;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode     = GPIO_Mode_Input;
    GPIO_InitStructure.Pin            = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Pull      = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);  

    GPIO_InitStructure.Pin            = TN_485_TXEN_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_No_Pull;
    GPIO_InitPeripheral(TN_485_TXEN_PORT, &GPIO_InitStructure);

}

static void UartSetTxMode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_9;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
}

static void UartSetPPMode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_9;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_PIN_9);
}

static void HandleRecvData(uint8_t data)
{
    RxCallback(data);
}

static void NVIC_UartConfiguration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}


void Uart1Init(pf_RxCallback callback)
{
    USART_InitType USART_InitStructure;
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE); 
    Uart1PortCfg();
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    USART_Init(USART1, &USART_InitStructure);
    //enable recive interrupt
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
    USART_Enable(USART1, ENABLE);
    NVIC_UartConfiguration();
    //接收回调函数
    RxCallback = callback;
}
/** 
* @brief  	设置串口波特率
* @param  	不同总线系统波特率不一致
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void Uart1SetBaudRate(void)
{
    if(bsp_CheckTimer(TMR_UART_BAUDRATE_SET)) 
    {
        static uint8_t temp=0;
        if(Atl485CheckFlag==0)  //未返回协议数据
        {
            switch (temp)
            {
            case 0:
                /* code */
                Atl485BaudRate=115200;  //硬件485 115200
                ACC_ENABLE();
                SEGGER_RTT_printf(0,"ATL485_HARDWARE_MODE_USART_BAUDRATE_SWITCH ---- %d\r\n",Atl485BaudRate);
                break;
            case 1:
                /* code */
                Atl485BaudRate=9600;  //硬件485 9600
                ACC_ENABLE();
                SEGGER_RTT_printf(0,"ATL485_HARDWARE_MODE_USART_BAUDRATE_SWITCH ---- %d\r\n",Atl485BaudRate);
                break;
            case 2:
                /* code */
                Atl485BaudRate=57600;   //一线通转485 57600
                ACC_DISABLE();
                SEGGER_RTT_printf(0,"ONEBUS_CHANGE_485_USART_BAUDRATE_SWITCH ---- %d\r\n",Atl485BaudRate);
                break;
            }
            USART_InitType USART_InitStructure;
            USART_StructInit(&USART_InitStructure);
            USART_InitStructure.BaudRate            = Atl485BaudRate;   //485硬件总线波特率	
            USART_Init(USART1, &USART_InitStructure);
        }
        else    //找到返回协议数据
        {
            if(Atl485CheckFlag>0)
            {
                Atl485CheckFlag--;
            }
        }
        if(++temp>2)
        {
            temp=0;
        }
    }
}

#if(PROJECT_ID == 2)
void Uart0Init4TY(pf_RxCallback callback)
{
   USART_InitType USART_InitStructure;
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE); 
    Uart0PortCfg();
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = 9600;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    USART_Init(USART1, &USART_InitStructure);
    //enable recive interrupt
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
    USART_Enable(USART1, ENABLE);
    NVIC_UartConfiguration();
    //接收回调函数
    RxCallback = callback;
}
#endif

void Uart1SendData(uint8_t *pData, uint16_t len)
{
    uint16_t i;
    //IO 设置到Uart发送模式
    UartSetTxMode();
    USART_ConfigInt(USART1,USART_INT_RXDNE,DISABLE);
#if (USE_485_IF == 1)
    ENABLE_485_TX_PIN();
#endif
	for(i = 0;i<len;i++)
	{
        USART_SendData(USART1, *(pData+i));        
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXDE) == RESET)
        {
        }
	}
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXC)==RESET);
    bsp_DelayUS(100); //由于需要在中断中调用，不使用bsp_DelayMS()函数
#if (USE_485_IF == 1)
    DISABLE_485_TX_PIN();
#endif
     USART_ClrIntPendingBit(USART1,USART_INT_RXDNE);
     USART_ConfigInt(USART1,USART_INT_RXDNE,ENABLE);
     UartSetPPMode();
}

///<Uart1 中断服务函数
void USART1_IRQHandler(void)
{
    uint8_t data;
   
    if(USART_GetIntStatus(USART1, USART_INT_RXDNE) != RESET)    ///接收数据
    {
        data = USART_ReceiveData(USART1);///读取数据
        HandleRecvData(data); 
        Atl485Flag=0x02;
        Atl485CheckFlag=5;  //5秒超时
        //USART_ClrIntPendingBit(USART1, USART_INT_RXDNE);   ///<清接收中断请求 
    }else
    {
        data = USART1->STS;
        //SEGGER_RTT_printf(0,"UART ERROR:%X\n",USART1->STS);
        USART_ReceiveData(USART1);
    }
}

/** 
* @brief  	得到硬件485模式标志位
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_atl485_flag()
{
    return Atl485Flag;
}

/** 
* @brief  	清楚硬件485模式标志位
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_atl485_clear()
{
    Atl485Flag=0;
    return Atl485Flag;
}

#endif