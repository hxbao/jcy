/******************************************************************************
  * @file           : haluart2.c
  * @version        : v1.0
  * @author         : Azreal
  * @creat          : 2021 0804
******************************************************************************/

#include "includes.h"

typedef void (*pf_Rx2Callback)(uint8_t rData);

pf_Rx2Callback Rx2Callback; 

uint8_t OneBusAtl485Flag;   //һ��ͨ485�ӿ�
/** 
* @brief  	bmsһ��ͨ����Ϊ����2
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void OneBusSetUart2Mode(void)
{
    GPIO_InitType GPIO_InitStructure;
     /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB,ENABLE);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = ONE_TXD1_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART2;
    GPIO_InitPeripheral(ONE_TXD1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin            = ONE_RXD1_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Input;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART2;
    GPIO_InitPeripheral(ONE_RXD1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = ONEWIre_485_PIN;	//ACC_CTL
	GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
	GPIO_InitStructure.GPIO_Pull    = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(ONEWIre_485_PORT, &GPIO_InitStructure);

	ONEWIre_485_ENABLE();	//�̽�ACC��BAT+
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
* @brief  	USART2 ��ʼ��
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
	// USART_EnableHalfDuplex(USART2, ENABLE);	//���߰�˫����ʼ��
    NVIC_UartConfiguration();
    //���ջص�����
    Rx2Callback = callback;
}

void Uart2SendData(uint8_t *pData, uint16_t len)
{
    uint16_t i;
    //IO ���õ�Uart����ģʽ
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
    bsp_DelayUS(100); //������Ҫ���ж��е��ã���ʹ��bsp_DelayMS()����
    USART_ClrIntPendingBit(USART2,USART_INT_RXDNE);
	USART_ConfigInt(USART2,USART_INT_RXDNE,ENABLE);
	// RX_ENABLE();
    // Uart2SetPPMode();
}

///<Uart2 �жϷ�����
void USART2_IRQHandler(void)
{
    uint8_t data;
    if(USART_GetIntStatus(USART2, USART_INT_RXDNE) != RESET)    ///��������
    {
        // USART_ClrIntPendingBit(USART2, USART_INT_RXDNE);   ///<������ж����� 
        data = USART_ReceiveData(USART2);///��ȡ����
		// RX_BUF[rx_cnt++]=data;
        HandleRecvData(data);  
        OneBusAtl485Flag=2;     
    }else
    {
        data = USART2->STS;
        // // SEGGER_RTT_printf(0,"UART ERROR:%X\n",USART2->STS);
        USART_ReceiveData(USART2);
    }
}

/** 
* @brief  	�õ�һ��ͨ485���ݵ��ģʽ
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
* @brief  	���һ��ͨ485���ݵ��ģʽ
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