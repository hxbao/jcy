/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"


pf_RxCallback LpUartRxCallback;

static void LpUartPortCfg(void)
{
    GPIO_InitType GPIO_InitStructure;
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);   
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure LPUART Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_LPUART;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = GPIO_PIN_0;  
    GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;  
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_LPUART;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

}

static void HandleLpuartRecvData(uint8_t data)
{
    LpUartRxCallback(data);
}

static void NVIC_UartConfiguration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = LPUART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}


void LpUartInit(pf_RxCallback callback)
{
    LPUART_InitType LPUART_InitStructure;

    RCC_ConfigLPUARTClk(RCC_LPUARTCLK_SRC_HSI);
    RCC_EnableRETPeriphClk(RCC_RET_PERIPH_LPUART, ENABLE);
    LpUartPortCfg();
    LPUART_StructInit(&LPUART_InitStructure);
    LPUART_InitStructure.BaudRate            = 9600;
    LPUART_InitStructure.Parity              = LPUART_PE_NO;
    LPUART_InitStructure.RtsThreshold        = LPUART_RTSTH_FIFOFU;
    LPUART_InitStructure.HardwareFlowControl = LPUART_HFCTRL_NONE;
    LPUART_InitStructure.Mode                = LPUART_MODE_RX | LPUART_MODE_TX;

    LPUART_Init(&LPUART_InitStructure);
    LPUART_ConfigInt(LPUART_INT_FIFO_NE, ENABLE);
    // LPUART_ConfigInt(LPUART_INT_TXC, ENABLE);
    
    NVIC_UartConfiguration();
    //接收回调函数
    LpUartRxCallback = callback;
}

void LpUartReset()
{
    LPUART_InitType LPUART_InitStructure;
    LPUART_StructInit(&LPUART_InitStructure);
    LPUART_InitStructure.BaudRate            = 115200;
    LPUART_Init(&LPUART_InitStructure);
}

void LpUartDeInit(void)
{

}

void LpUartSendData(uint8_t *pData, uint16_t len)
{
    uint16_t i;
    //IO 设置到Uart发送模式
	for(i = 0;i<len;i++)
	{ 
        LPUART_SendData(pData[i]);
        while (LPUART_GetFlagStatus(LPUART_FLAG_TXC) == RESET);
        LPUART_ClrFlag(LPUART_FLAG_TXC);
	}
}

void LpUartSendDataLoop()
{

}

///<Uart1 中断服务函数
void LPUART_IRQHandler(void)
{
    uint8_t data;

    if (LPUART_GetIntStatus(LPUART_INT_FIFO_NE) != RESET)
    {
        /* Read one byte from the receive data register */
        data = LPUART_ReceiveData();
        bt24_receive_input(data);
        HandleLpuartRecvData(data); 
    }else
    {
    //    LPUART_ClrIntPendingBit(LPUART_INT_FIFO_NE);
    }
}
