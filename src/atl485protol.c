#include "includes.h"
#include "atl485protol.h"


//ATL BMS 485设备地址
#define ATL_MODBUS_DEV 0x01


//命令码,主机发送的命令码
#define ATL_MODBUS_CMD_READ_ID        0x04
#define ATL_MODBUS_CMD_READ           0x03
#define ATL_MODBUS_CMD_WRITE          0x10
#define ATL_MODBUS_CMD_WRITE_ID       0x11

//读电压单体
#define CMD_ID_READ_CELLVID            0x01
//读取电池数据
#define CMD_ID_READ_BATD               0x03
//读取电池故障报警信息
#define CMD_ID_READ_BATSTA             0x04         
//读取项目信息
#define CMD_ID_READ_PROJECT            0x07





//回应命令错误码
typedef enum {
    CS_ERR = 0x80,
    ADDR_ERR = 0x40,
    UNKONWN_CMD_ERR = 0x20,
    NO_DATA_ERR = 0x10,
    PWD_ERR = 0x11
}ACK_ERR_en;

typedef struct{
    uint8_t funcode;
    uint8_t cmdId;
    uint16_t reqaddr;
    uint16_t reqCount;
}AltMdCmdReq_t;


uint8_t ATLMdOrgInBuf[256];

Atl485_Cellv_t atl485cellv;
Atl485_Cellt_t atl485cellt;
Atl485_batd_t atl485batd;
Atl485_BatState_t atl485batsta;
Atl485_ProjectInfo_t atl485prjInfo;
Atl485_ProjectRunthresd_t atl485runthred;
Atl485_BatErrHis_t atl485baterrhis;
Atl485_ChgHis_t atl485chghis;
Atl485_F099HIS_t atl485f099his;

AltMdCmdReq_t atlcmdreq;


uint8_t rxIndex = 0;
uint8_t ATLMdPollFlg = 0;


static uint16_t ATL_Calc_CRC(uint8_t *pDesBuf,uint8_t len)
{
    uint16_t TY_CRC = 0xFFFF;
    uint16_t i;
    uint16_t count;

    for(count = 0; count < len; count++)
    {
        int i;
        TY_CRC = TY_CRC^*(pDesBuf + count);
        for(i = 0; i < 8; i++)
        {
            if(TY_CRC&1)
            {
                TY_CRC>>=1;
                TY_CRC^=0xA001;
            }
            else{
                TY_CRC>>=1;
            }
        }
    }
    return TY_CRC;
}

static void CommdSendFrame(AltMdCmdReq_t req)
{  
    uint8_t data[10];

    uint16_t crc;

    data[0] = 0x01;
    data[1] = req.funcode;
    data[2] = (uint8_t)req.reqaddr;
    data[3] = (uint8_t)(req.reqaddr>>8);
    data[4] = (uint8_t)(req.reqCount);
    data[5] = (uint8_t)(req.reqCount>>8);

    crc = ATL_Calc_CRC(data,6);
    data[6] = (uint8_t)crc;
    data[7] = (uint8_t)(crc>>8);
    ATl_ModbusSend(data,8);
}

static void ATLModbusParse(uint8_t *pMdInput,uint8_t len)
{
    uint8_t i;
    uint16_t byCount;
    uint16_t mdoffset;

    uint16_t TY_CRCO;
    uint16_t TY_CRC;
    uint16_t length2;

    //TyModbusTY_CRCCheck
    TY_CRCO = pMdInput[len-2] + (((uint16_t)pMdInput[len-1])<<8);
    TY_CRC = ATL_Calc_CRC(pMdInput,len-2);

    if(TY_CRCO == TY_CRC)
    {
        //回复数据长度
        byCount = ((uint16_t)pMdInput[2]<<8) +pMdInput[3];
        

        //根据发送的指令和地址，把接收到的数据，填充到相应的数据结构里
        // #define CMD_ID_READ_CELLVID            0x01
        // //读取电池数据
        // #define CMD_ID_READ_BATD               0x03
        // //读取电池故障报警信息
        // #define CMD_ID_READ_BATSTA             0x04         
        // //读取项目信息
        // #define CMD_ID_READ_PROJECT            0x07
        
        if(atlcmdreq.cmdId == CMD_ID_READ_CELLVID)//读数据请求
        {
            if((atlcmdreq.reqaddr >= ATL485_BASE_ADDR_CELLV) && (atlcmdreq.reqaddr < ATL485_BASE_ADDR_CELLV+1000))
            {
                //地址偏移
                mdoffset = atlcmdreq.reqaddr - ATL485_BASE_ADDR_CELLV;
                memcpy((uint8_t*)&atl485cellv+mdoffset,pMdInput+5,2*byCount);
            }

        }else
        if(atlcmdreq.cmdId == CMD_ID_READ_BATD)
        {
            if((atlcmdreq.reqaddr >= ATL485_BASE_ADDR_BATD) && (atlcmdreq.reqaddr < ATL485_BASE_ADDR_BATD+1000))
            {
                 //地址偏移
                mdoffset = atlcmdreq.reqaddr - ATL485_BASE_ADDR_BATD;
                memcpy((uint8_t*)&atl485batd+mdoffset,pMdInput+5,2*byCount);
            }
        }else
        if(atlcmdreq.cmdId == CMD_ID_READ_BATSTA)
        {
            if((atlcmdreq.reqaddr >= ATL485_BASE_ADDR_BATSTA) && (atlcmdreq.reqaddr < ATL485_BASE_ADDR_BATSTA+1000))
            {
                  //地址偏移
                mdoffset = atlcmdreq.reqaddr - ATL485_BASE_ADDR_BATSTA;
                memcpy((uint8_t*)&atl485batsta+mdoffset,pMdInput+5,2*byCount);
            }
        }else
        if(atlcmdreq.cmdId == CMD_ID_READ_PROJECT)
        {
            if((atlcmdreq.reqaddr >= ATL485_BASE_ADDR_PROJECTINTO) && (atlcmdreq.reqaddr < ATL485_BASE_ADDR_PROJECTINTO+1000))
            {
                mdoffset = atlcmdreq.reqaddr - ATL485_BASE_ADDR_PROJECTINTO;
                memcpy((uint8_t*)&atl485prjInfo+mdoffset,pMdInput+5,2*byCount);
            }
        }

    }else
    {
        SEGGER_RTT_printf(0,"recv crc error\n");
    }
}



void ATL_ModbusRecvHandle(uint8_t rdata)
{
    uint16_t mdLen = 0;
    if(rdata == ATL_MODBUS_DEV && rxIndex == 0)
    {
        ATLMdOrgInBuf[rxIndex++] = rdata;
        return;
    }else
    if(rxIndex > 0)
    {           
        ATLMdOrgInBuf[rxIndex++] = rdata;
        if(rxIndex > 254)
        {
            SEGGER_RTT_printf(0,"recv:%02X ",rdata);
            return ;
        }
        SEGGER_RTT_printf(0,"%02X ",rdata);
        if(rxIndex ==4)
        {
            mdLen = ATLMdOrgInBuf[3] + (uint16_t)ATLMdOrgInBuf[4]<<8;
        }
        //请求读数据帧
        if(rxIndex == mdLen*2 + 6)
        {   
            //返回读数据帧         
            if(ATL_MODBUS_CMD_READ == ATLMdOrgInBuf[1])
            {
                SEGGER_RTT_printf(0,"\n");
                ATLModbusParse(ATLMdOrgInBuf,rxIndex);
                rxIndex = 0;
            }else{
                rxIndex = 0;
            }

            //返回写数据帧
            if(ATL_MODBUS_CMD_WRITE == ATLMdOrgInBuf[1])
            {
                SEGGER_RTT_printf(0,"\n");
                //ATLModbusParse(ATLMdOrgInBuf);
                rxIndex = 0;
            }else{
                rxIndex = 0;
            }
        }
    }
}

void ATLModbusPoll(void)
{
    static uint8_t index = 0;
    //循环发送取数据指令
    if(bsp_CheckTimer(TMR_ATL485))  
    {
        // GPIO_ToggleBits(TEST_IO_PORT,TEST_IO_PIN);
        switch(index++)
        {
            case 0:
            atlcmdreq.cmdId = CMD_ID_READ_CELLVID;
            atlcmdreq.funcode = 0x03;
            atlcmdreq.reqaddr = ATL485_BASE_ADDR_CELLV;
            atlcmdreq.reqCount = sizeof(Atl485_Cellv_t);
            CommdSendFrame(atlcmdreq);
            break;

            case 1:
            atlcmdreq.cmdId = CMD_ID_READ_BATD;
            atlcmdreq.funcode = 0x03;
            atlcmdreq.reqaddr = ATL485_BASE_ADDR_BATD;
            atlcmdreq.reqCount = sizeof(Atl485_batd_t);
            CommdSendFrame(atlcmdreq);
            break;

            case 2:
            atlcmdreq.cmdId = CMD_ID_READ_BATSTA;
            atlcmdreq.funcode = 0x03;
            atlcmdreq.reqaddr = ATL485_BASE_ADDR_BATSTA;
            atlcmdreq.reqCount = sizeof(Atl485_BatState_t);
            CommdSendFrame(atlcmdreq);
            break;

            case 3:
            atlcmdreq.cmdId = CMD_ID_READ_PROJECT;
            atlcmdreq.funcode = 0x03;
            atlcmdreq.reqaddr = ATL485_BASE_ADDR_PROJECTINTO;
            atlcmdreq.reqCount = sizeof(Atl485_ProjectInfo_t);
            CommdSendFrame(atlcmdreq);
            break;
        }
    } 

    if(index == 4)
    {
        index = 0;
    }
}



