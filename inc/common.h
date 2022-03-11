#ifndef COMMON_H
#define COMMON_H

#include "includes.h"



#if(MCU_LIB_SELECT == 2)
#define TN_LED_PORT       GPIOB
#define TN_LED_PIN        GPIO_PIN_10

#define TN_PF1_PORT      GPIOF                                        
#define TN_PF1_PIN       GPIO_PIN_1


///< I2C SH309
#define TN_I2C1_SCL_PORT    GPIOB
#define TN_I2C1_SCL_PIN     GPIO_PIN_13
#define TN_I2C1_SDA_PORT    GPIOB
#define TN_I2C1_SDA_PIN     GPIO_PIN_14

///< CHG_DET                
#define TN_CHG_DET_PORT      GPIOA
#define TN_CHG_DET_PIN       GPIO_PIN_0

///< WEAK_UP
#define TN_WAKE_UP_PORT      GPIOA
#define TN_WAKE_UP_PIN       GPIO_PIN_1

///< ACC
#define TN_ACC_PORT      GPIOA
#define TN_ACC_PIN       GPIO_PIN_2

///<SH_INT
#define TN_SHINT_PORT      GPIOA
#define TN_SHINT_PIN       GPIO_PIN_10


///< TXD0
#define TN_UART0_TXD_PORT      GPIOA
#define TN_UART0_TXD_PIN       GPIO_PIN_9

///< RXD0
#define TN_UART0_RXD_PORT      GPIOA
#define TN_UART0_RXD_PIN      GPIO_PIN_10

///< VPRO_CON
#define TN_VPRO_CON_PORT      GPIOB
#define TN_VPRO_CON_PIN       GPIO_PIN_5

///< SH_SHIP
#define TN_SHSHIP_PORT       GPIOA
#define TN_SHSHIP_PIN       GPIO_PIN_6

///<NTC0 POWER
#define TN_NTC0_POWER_PORT      GPIOB
#define TN_NTC0_POWER_PIN       GPIO_PIN_7

///<NTC1 POWER
#define TN_NTC1_POWER_PORT      GPIOB
#define TN_NTC1_POWER_PIN       GPIO_PIN_7

///NTC1 ADC Input
#define TN_NTC1_ADIN_PORT    GPIOA
#define TN_NTC1_ADIN_PIN     GPIO_PIN_3
///NTC0 ADC Input
#define TN_NTC0_ADIN_PORT    GPIOA
#define TN_NTC0_ADIN_PIN     GPIO_PIN_4


///<ONE_TX
#define TN_ONE_TX_PORT      GPIOA
#define TN_ONE_TX_PIN       GPIO_PIN_9

///<ONE_RX
#define TN_ONE_RX_PORT      GPIOA
#define TN_ONE_RX_PIN       GPIO_PIN_10

///<AlARM
#define TN_ALARM_PORT      GPIOB
#define TN_ALARM_PIN       GPIO_PIN_12


///PRE_ISEN_P
#define TN_PRE_ISEN_P_PORT      GPIOB
#define TN_PRE_ISEN_P_PIN       GPIO_PIN_1

///PRE_ISEN_N
#define TN_PRE_ISEN_N_PORT      GPIOB
#define TN_PRE_ISEN_N_PIN       GPIO_PIN_2

#define TN_BLE_STATUS_PORT      GPIOB
#define TN_BLE_STATUS_PIN      GPIO_PIN_2

#define TN_SPI_CS_PIN   GPIO_PIN_12
#define TN_SPI_CS_PORT  GPIOB

#define TN_SPI_CLK_PIN   GPIO_PIN_13
#define TN_SPI_CLK_PORT  GPIOB

#define TN_SPI_MISO_PIN   GPIO_PIN_14
#define TN_SPI_MISO_PORT  GPIOB

#define TN_SPI_MOSI_PIN   GPIO_PIN_15
#define TN_SPI_MOSI_PORT  GPIOB

#define TN_485_COMM_ON_PORT GPIOA
#define TN_485_COMM_ON_PIN GPIO_PIN_11

#define TN_COMM_ON2_PORT GPIOA
#define TN_COMM_ON2_PIN GPIO_PIN_12

#define TN_485_TXEN_PORT  GPIOB
#define TN_485_TXEN_PIN   GPIO_PIN_3

#define TN_485_INT_PORT GPIOB
#define TN_485_INT_PIN GPIO_PIN_6


#if(MCU_LIB_SELECT ==1)
extern uint8_t CommonRam[512];
extern uint8_t CommonRam2[256];
#elif(MCU_LIB_SELECT ==2)
extern uint8_t CommonRam[2048];
extern uint8_t CommonRam2[256];
#endif


extern const float NTC103AT[151];


uint16_t CalcuTemp(uint16_t Vntc);
uint16_t CalcuTemp1(float Rntc);
float CalaRntcFromTemp(uint16_t temp);
void MyMemcpy(uint8_t *dst,uint8_t *src,uint16_t len);
#endif

#endif