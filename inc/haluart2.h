#ifndef  __HALUART2_H
#define  __HALUART2_H

#include "includes.h"

typedef void (*pf_Rx2Callback)(uint8_t rData);
 
extern uint8_t Uart2_RxBuf[100];

void Uart2Init(pf_Rx2Callback callback);
void Uart2DeInit(void);
void Uart2SendData(uint8_t *pData, uint16_t len);
uint16_t Uart2ReceiveData(uint8_t *pData);
uint8_t get_onebus_clear(void);
uint8_t get_onebus_flag(void);
#endif
