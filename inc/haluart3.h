#ifndef  __HALUART3_H
#define  __HALUART3_H

#include "includes.h"

typedef void (*pf_Rx3Callback)(uint8_t rData);
 
extern uint8_t Uart3_RxBuf[100];
extern uint8_t FlagUart0Inited;
extern uint16_t Uart0RxCount;

void Uart3Init(pf_Rx3Callback callback);
void Uart3DeInit(void);
void Uart3SendData(uint8_t *pData, uint16_t len);
uint16_t Uart3ReceiveData(uint8_t *pData);

#if(MCU_LIB_SELECT == 1)
#define DISABLE_UART_RXINT() Uart_DisableIrq(M0P_UART0,UartRxIrq)
#define ENABLE_UART_RXINT()  Uart_EnableIrq(M0P_UART0,UartRxIrq)
#elif(MCU_LIB_SELECT == 2)
#define DISABLE_UART_RXINT() USART_ConfigInt(USART1,USART_INT_RXDNE,DISABLE)
#define ENABLE_UART_RXINT()  USART_ConfigInt(USART1,USART_INT_RXDNE,ENABLE)
#endif

#endif
