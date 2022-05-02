/******************************************************************************
  * @file           : haluart2.c
  * @version        : v1.0
  * @author         : Azreal
  * @creat          : 2021 0804
******************************************************************************/

#include "includes.h"

typedef void (*pf_Rx2Callback)(uint8_t rData);

pf_Rx2Callback Rx2Callback; 

uint8_t OneBusAtl485Flag;   //一线通485接口
/** 
* @brief  	bms一线通设置为串口2
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
static void OneBusSetUart2Mode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = ONE_TXD1_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART2;
    GPIO_InitPeripheral(ONE_TXD1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin            = ONE_RXD1_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Input;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART2;
    GPIO_InitPeripheral(ONE_RXD1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = GPIO_PIN_8;	//ACC_CTL
	GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
	GPIO_InitStructure.GPIO_Pull    = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

	ACC_ENABLE();	//短接ACC和BAT+
}

static void Uart2SetTxMode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = ONE_TXD1_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART2;
    GPIO_InitPeripheral(ONE_TXD1_PORT, &GPIO_InitStructure);
}

static void Uart2SetPPMode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = ONE_TXD1_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(ONE_TXD1_PORT, &GPIO_InitStructure);
    GPIO_SetBits(ONE_TXD1_PORT,ONE_TXD1_PIN);
}

static void HandleRecvData(uint8_t data)
{
    Rx2Callback(data);
}

static void NVIC_UartConfiguration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
/** 
* @brief  	USART2 初始化
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void Uart2Init(pf_Rx2Callback callback)
{
    USART_InitType USART_InitStructure;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2, ENABLE); 
    OneBusSetUart2Mode();
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = 57600;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    USART_Init(USART2, &USART_InitStructure);
    //enable recive interrupt
    USART_ConfigInt(USART2, USART_INT_RXDNE, ENABLE);

	USART_Enable(USART2, ENABLE);
	// USART_EnableHalfDuplex(USART2, ENABLE);	//单线半双工初始化
    NVIC_UartConfiguration();
    //接收回调函数
    Rx2Callback = callback;
}

void Uart2SendData(uint8_t *pData, uint16_t len)
{
    uint16_t i;
    //IO 设置到Uart发送模式
    OneBusAtl485Flag=0;
    Uart2SetTxMode();
	// RX_DISABLE();
    USART_ConfigInt(USART2,USART_INT_RXDNE,DISABLE);
	for(i = 0;i<len;i++)
	{
        USART_SendData(USART2, *(pData+i));        
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXDE) == RESET)
        {
        }
	}
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXC)==RESET );
    bsp_DelayUS(100); //由于需要在中断中调用，不使用bsp_DelayMS()函数
    USART_ClrIntPendingBit(USART2,USART_INT_RXDNE);
	USART_ConfigInt(USART2,USART_INT_RXDNE,ENABLE);
	// RX_ENABLE();
    // Uart2SetPPMode();
}

///<Uart2 中断服务函数
void USART2_IRQHandler(void)
{
    uint8_t data;
    if(USART_GetIntStatus(USART2, USART_INT_RXDNE) != RESET)    ///接收数据
    {
        // USART_ClrIntPendingBit(USART2, USART_INT_RXDNE);   ///<清接收中断请求 
        data = USART_ReceiveData(USART2);///读取数据
		// RX_BUF[rx_cnt++]=data;
        HandleRecvData(data);  
        OneBusAtl485Flag=1;     
    }else
    {
        data = USART2->STS;
        // // SEGGER_RTT_printf(0,"UART ERROR:%X\n",USART2->STS);
        USART_ReceiveData(USART2);
    }
}

/** 
* @brief  	得到一线通485兼容电池模式
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_onebus_flag()
{
    return OneBusAtl485Flag;
}
/** 
* @brief  	清除一线通485兼容电池模式
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t get_onebus_clear()
{
    OneBusAtl485Flag=0;
    return OneBusAtl485Flag;
}