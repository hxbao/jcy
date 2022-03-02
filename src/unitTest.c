#include "includes.h"
#include "SEGGER_RTT.h"

#ifdef UNIT_TEST_EN


void MCU_GpioSHIntInit(void);

//返回的数据
//68 31 CE 68 82 A0 53 64 34 34 88 88 88 88 34 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 33 33 33 33 33 33 33 33 22 22 22 22 22 22 33 63 33 33 33 33 34 ED 33 33 33 33 33 34 33 32 37 37 4B 33 38 41 F1 40 59 3F 24 40 3A 40 CE 40 3F 40 55 41 B0 40 36 3F 2F 3F 1E 40 44 3F FC 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 79 63 77 64 63 89 63 64 89 64 33 33 33 3A 41 3C 34 79 40 41 F1 3F FC 33 3F 34 28 3E 0A 3E 09 33 34 34 28 52 33 97 48 87 48 87 33 33 33 33 33 33 33 33 33 33 1D 16 


// function
//     local s = ...
//     local str = s:toHex()
  
//     local t = {}
//     local m = {}

//     t.Msg = "niuprotocol"
//     t.originData = str
//     m.niuPayload = t
//     return json.encode(m)

// end



// function
//     local s = ...
//     local str = s:toHex()
//     local addr = str:sub(1,10)
//     local strlen = str:len()/2 -2
//     local cs = 0
//     local t = {}
//     local m = {}

//     --地址命令匹配判断
//     if(addr ~= "6831ce") then 
//         t.errMsg = "Invalid addr"
//         t.originData = str
//         m.err = t
//         return json.encode(m)
//     end
//      --校验数据完整性
//     for i =1,strlen do
//        cs = cs + tonumber(str:sub(2*(i-1)+1,2*(i-1)+2),16)
//     end
//     cs = cs%0x100
//     if(cs == tonumber(str:sub(-4,-3),16)) then       
//         --解码数据包
//         local cmd = tonumber(str:sub(9,10),16)
//         local datlen = tonumber(str:sub(11,12),16)
//         --payload 数据需要减0x33，才能解码到原始数据，payload 数据就是表地址0开始到最后的数据
//         local payload = str:sub(13,-5) 
//         --读表数据回应
       
//           t.fun = 0x82
//           t.datlen = 0x6
//           t.payload = payload        
//           m.niuprotocol = t
//           return json.encode(m)

//     else
//         t.errMsg = "checksum error"
//         t.originData = str
//         m.err = t
//         return json.encode(m)
//     end
// end



uint8_t inited = 0;
uint8_t abDataOut[512];
uint64_t timeTick = 0;
//RTT接收字符数据
uint8_t rttKeyInArray[11]={0x68,0x31,0xce,0x68,0x02,0x02,0x33,0xd3,0xd9,0x16};//read whole table
uint8_t rttRxIndex = 0;

static void PrintOutNiuCommdTable(void);
static void PrintOutConfigMacro(void);
static void PrintOutAlgEnginerInfo(void);
static void  RttLogRecord(void);

void UnitTestProcess(void)
{
    //stc_gpio_cfg_t pstcGpioCfg;
    int rttKeyinChar;

    rttKeyinChar = SEGGER_RTT_GetKey();

    if(inited == 0)
    {
        inited = 1;
        SEGGER_RTT_ConfigUpBuffer(1, "DataOut", &abDataOut[0], 512,
        SEGGER_RTT_MODE_NO_BLOCK_SKIP);
        bsp_StartAutoTimer(TMR_UINT_TEST,1000);
    }

    if(bsp_CheckTimer(TMR_UINT_TEST))
    {
        //data record
        //RttLogRecord();
    }
    
    if(rttKeyinChar>0)
    {

           
        if((char)rttKeyinChar=='1')
        {
            
           
            SEGGER_RTT_printf(0,"DelayMS ONETX 500ms Test...\n");
            MCU_GPIO_SetBit(TN_ONE_TX_PORT,TN_ONE_TX_PIN);
            bsp_DelayMS(500);
            MCU_GPIO_ClrBit(TN_ONE_TX_PORT,TN_ONE_TX_PIN);
           
             //使能时钟输出，测量
        }else
        if((char)rttKeyinChar=='2')
        {
           /*   ///< 端口方向配置->输出
            pstcGpioCfg.enDir = GpioDirOut;
            ///< 端口驱动能力配置->高驱动能力
            pstcGpioCfg.enDrv = GpioDrvH;
            ///< 端口上下拉配置->无上下拉
            pstcGpioCfg.enPu = GpioPuDisable;
            pstcGpioCfg.enPd = GpioPdDisable;
            ///< 端口开漏输出配置->开漏输出关闭
            pstcGpioCfg.enOD = GpioOdDisable;    
            ///< GPIO IO PB00初始化
            //Gpio_Init(GpioPortA, GpioPin14, &pstcGpioCfg);
            Gpio_Init(GpioPortA, GpioPin1, &pstcGpioCfg);

            Gpio_SfHClkOutputCfg(GpioSfHclkOutEnable, GpioSfHclkOutDiv1);
            Gpio_SetAfMode(GpioPortA, GpioPin1, GpioAf6);*/
        }else
        if((char)rttKeyinChar=='3')//afeInfo view
        {


        }else
        if((char)rttKeyinChar=='4')//开关充放电MOS，采集MOS反馈信号
        {

        }else
        if((char)rttKeyinChar=='5')//预放电开关信号测试
        {
 
        }else
        if((char)rttKeyinChar=='6')//串口一线通电路测试
        {

 
        }else
        if((char)rttKeyinChar=='7')//充电唤醒电路测试
        {
            // SWITCH_PRED_ON();
            // bsp_DelayMS(500);
            // SWITCH_PRED_OFF();
            // SEGGER_RTT_printf(0,"PREDsg Driver test,Please Check Oscilloscope Signal\n");
        }else
        if((char)rttKeyinChar=='8')//系统唤醒和ACC唤醒电路测试
        {
 
        }else
        if((char)rttKeyinChar=='9')//三端保险丝自毁电路驱动测试
        {
 
        }else
        if((char)rttKeyinChar=='a')//低功耗测试
        {
 
        }else
        if((char)rttKeyinChar=='b')//获取时钟频率
        {
           //uint32_t fval;
           // fval = Sysctrl_GetHClkFreq();
           // SEGGER_RTT_printf(0,"HclkFreq->%d\n",fval);
        }
        else
        if((char)rttKeyinChar=='c')//打印niu modbus 数据表
        {

        }else
        if((char)rttKeyinChar=='d')//打印flash write 测试
        {
 
        }else
        if((char)rttKeyinChar=='f')//打印config文件
        {

        }else
        if((char)rttKeyinChar=='s')
        {

        }
    }
}

static void  RttLogRecord(void)
{
 
}

static void PrintOutNiuCommdTable(void)
{
 
}

static void PrintOutConfigMacro(void)
{
 


}

static void PrintOutAlgEnginerInfo(void)
{
 
}

#endif