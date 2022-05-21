/*
 * iap.h
 *
 *  Created on: 
 *      Author: hxbao
 */

#ifndef INCLUDE_IAP_H_
#define INCLUDE_IAP_H_

#include "includes.h"
#define APP_CONFIG_CRC_ADDR  (FLASH_BASE_ADDR+0x01FFF0)
#define APP_CONFIG_AREA_ADDR (FLASH_BASE_ADDR+0x01FF7F)
#define FLASH_START_ADDR_APP2 (FLASH_BASE_ADDR+0x0010000)
#define IAP_APP_BIN_ADDR ((uint32_t)(FLASH_BASE_ADDR+VECT_OFFSET))  //保留4K空间用于存放Bootloader

typedef void (*iapfun)(void);   

typedef enum
{
	BIN_CRC_ERROR,
	BLOCK_CRC_ERROR,
	BIN_TOTAL_SIZE_ERROR,
	BIN_FILE_DOWN_OK,
	BIN_BLOCK_DOWN_OK
} Iap_ErrorCode;

//define appHandle struct
// typedef  struct
// {
//   uint8_t  flag;
//   uint8_t  resv[3];
//   int appBinByteSize;
//   uint32_t srcFlashAddr;
//   int appBinCrc;
//   uint16_t appBinPackNum; //升级包数量
//   char appbinVersion[8];

// }AppBinHandle_t;

typedef  struct
{
  uint8_t  flag;
  uint8_t  otaInSwVer[4];	//软件内部版本号
  uint8_t  otaExSwVer[2];	//软件内部次版本号
  uint8_t  proCode[12];		//项目编号
  uint8_t  otaHwVerMaj;		//硬件主版本号
  uint8_t  otaHwVerMin;		//硬件次版本号
  int appBinByteSize;
  uint16_t appBinPackNum; //升级包数量
  uint32_t srcFlashAddr;
  int appBinCrc;
  char appbinVersion[8];

}AppBinHandle_t;

extern AppBinHandle_t appBin;

//Iap_ErrorCode iap_temporaryStore_appbin(void);
void iap_temporaryStore_appbin(uint8_t *data,uint32_t offsetAddr, uint16_t len);

void iap_start(uint32_t appSize,uint16_t appCrc);
uint8_t IsIapBusy(void);
Iap_ErrorCode iap_verify(void);
Iap_ErrorCode iap_write_bin2flash(void);

uint8_t IapHanle(uint8_t cmd,uint8_t len,uint8_t *inBuf,uint8_t *outBuf);
void save_iap_configration(void);
void iap_load_app(uint32_t appxaddr);
void iap_write_appbin(uint32_t appxaddr, uint32_t appbufaddr, uint32_t appsize);
uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint32_t usDataLen, uint16_t wCRCin);
void IAP_TEST(void);

#endif /* INCLUDE_IAP_H_ */
