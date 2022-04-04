#include "includes.h"

pf_Rx3Callback Rx3Callback;      

static void Uart3PortCfg(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);   
    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_10;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF0_USART3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode     = GPIO_Mode_Input;
    GPIO_InitStructure.Pin            = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Pull      = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_USART3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);  

}
static void UartSetTxMode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_10;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF0_USART3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

static void UartSetPPMode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_10;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF0_USART3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_PIN_10);
}

static void HandleRecvData(uint8_t data)
{
    Rx3Callback(data);
}

static void NVIC_UartConfiguration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void Uart3Init(pf_Rx3Callback callback)
{
    USART_InitType USART_InitStructure;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART3, ENABLE); 
    Uart3PortCfg();
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    USART_Init(USART3, &USART_InitStructure);
    //enable recive interrupt
    USART_ConfigInt(USART3, USART_INT_RXDNE, ENABLE);

    USART_Enable(USART3, ENABLE);
    NVIC_UartConfiguration();
    //接收回调函数
    Rx3Callback = callback;
}

void Uart3SendData(uint8_t *pData, uint16_t len)
{
    uint16_t i;
    //IO 设置到Uart发送模式
    UartSetTxMode();
    USART_ConfigInt(USART3,USART_INT_RXDNE,DISABLE);
	for(i = 0;i<len;i++)
	{
        USART_SendData(USART3, *(pData+i));        
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXDE) == RESET)
        {
        }
	}
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXC)==RESET);
    bsp_DelayUS(100); //由于需要在中断中调用，不使用bsp_DelayMS()函数
    USART_ClrIntPendingBit(USART3,USART_INT_RXDNE);
    USART_ConfigInt(USART3,USART_INT_RXDNE,ENABLE);
    UartSetPPMode();
}

///<Uart3 中断服务函数
void USART3_IRQHandler(void)
{
    uint8_t data;
   
    if(USART_GetIntStatus(USART3, USART_INT_RXDNE) != RESET)    ///接收数据
    {
        
        data = USART_ReceiveData(USART3);///读取数据
        HandleRecvData(data); 
        //USART_ClrIntPendingBit(USART1, USART_INT_RXDNE);   ///<清接收中断请求 
    }else
    {
        data = USART3->STS;
        //SEGGER_RTT_printf(0,"UART ERROR:%X\n",USART1->STS);
        USART_ReceiveData(USART3);
    }
}