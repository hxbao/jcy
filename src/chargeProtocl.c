#include "includes.h"
#include "chargeProtocol.h"

#define ch_len 0x0D //数据长度

unsigned char data_process_buf[DATA_PROCESS_LMT]; //串口数据处理缓存
unsigned char uart_rx_buf[UART_RECV_BUF_LMT];     //串口接收缓存
unsigned char uart_tx_buf[UART_SEND_BUF_LMT];     //串口发送缓存
//
unsigned char *rx_buf_in;
unsigned char *rx_buf_out;

unsigned char dev_recv_flag; // ENABLE:停止一切数据上传  DISABLE:恢复一切数据上传

DeviceResponseCmdErr_t DRC;
/**
 * @brief  计算校验和
 * @param[in] {pack} 数据源指针
 * @param[in] {pack_len} 计算校验和长度
 * @return 校验和
 */
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len,uint8_t frame_type)
{
    unsigned short i;
    unsigned char check_sum = 0;

    for (i = 0; i < pack_len; i++)
    {
        check_sum += *pack++;
    }
    if(frame_type==0)
    {
        check_sum+=DEV_FRAME_FIRST;
    }
    else
    {
        check_sum+=MOD_FRAME_FIRST;
    }
    return check_sum;
}

/**
 * @brief  协议串口初始化函数
 * @param  Null
 * @return Null
 * @note   在MCU初始化代码中调用该函数
 */
void device_protocol_init(void)
{
    rx_buf_in = (unsigned char *)uart_rx_buf;
    rx_buf_out = (unsigned char *)uart_rx_buf;

    dev_recv_flag = DISABLE;
}
/**
 * @brief  串口接收数据暂存处理
 * @param[in] {value} 串口收到的1字节数据
 * @return Null
 * @note   在MCU串口处理函数中调用该函数,并将接收到的数据作为参数传入
 */
void device_uart_receive_input(unsigned char value)
{
    if (1 == rx_buf_out - rx_buf_in)
    {
        //串口接收缓存已满
    }
    else if ((rx_buf_in > rx_buf_out) && ((rx_buf_in - rx_buf_out) >= sizeof(uart_rx_buf)))
    {
        //串口接收缓存已满
    }
    else
    {
        //串口接收缓存未满
        if (rx_buf_in >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf)))
        {
            rx_buf_in = (unsigned char *)(uart_rx_buf);
        }

        *rx_buf_in++ = value;
    }
}

/**
 * @brief  判断串口接收缓存中是否有数据
 * @param  Null
 * @return 是否有数据
 */
unsigned char with_data_rxbuff(void)
{
    if (rx_buf_in != rx_buf_out)
        return 1;
    else
        return 0;
}

/**
 * @brief  读取队列1字节数据
 * @param  Null
 * @return Read the data
 */
unsigned char take_byte_rxbuff(void)
{
    unsigned char value;

    if (rx_buf_out != rx_buf_in)
    {
        //有数据
        if (rx_buf_out >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf)))
        {
            //数据已经到末尾
            rx_buf_out = (unsigned char *)(uart_rx_buf);
        }

        value = *rx_buf_out++;
    }

    return value;
}
/*****************************************************************************
函数名称 : device_data_handle
功能描述 : 数据帧处理
输入参数 : offset:数据起始位
返回参数 : 无
*****************************************************************************/
void device_data_handle(unsigned short offset,DeviceResponseCmdErr_t *DRC)
{
    DRC->DEV_STATUS = data_process_buf[offset + 3];
    DRC->DEV_GET_VOL= (data_process_buf[offset + 4]<<24)|(data_process_buf[offset + 5]<<16)
        |(data_process_buf[offset + 6]<<8)|data_process_buf[offset + 7];
    DRC->DEV_GET_CUR=(data_process_buf[offset + 8]<<8)|data_process_buf[offset + 9];
    DRC->DEV_TIME_H=data_process_buf[offset + 10];
    DRC->DEV_TIME_M=data_process_buf[offset + 11];
    DRC->DEV_TIME_S=data_process_buf[offset + 12];
    DRC->DEV_GET_SPE=(data_process_buf[offset + 13]<<8)|data_process_buf[offset + 14];
    DRC->DEV_SET_DISCH_CUR=(data_process_buf[offset + 15]<<8)|data_process_buf[offset + 16];
    DRC->DEV_SET_DISCH_END_VOL= (data_process_buf[offset + 17]<<24)|(data_process_buf[offset + 18]<<16)
        |(data_process_buf[offset + 19]<<8)|data_process_buf[offset + 20];
    DRC->DEV_SET_CH_CUR=(data_process_buf[offset + 21]<<8)|data_process_buf[offset + 22];
    DRC->DEV_SET_CH_END_VOL= (data_process_buf[offset + 23]<<24)|(data_process_buf[offset + 24]<<16)
        |(data_process_buf[offset + 25]<<8)|data_process_buf[offset + 26];
    DRC->BAT_CH_END_CUR=(data_process_buf[offset + 27]<<8)|data_process_buf[offset + 28];
    DRC->DEV_BAT_TYPE=data_process_buf[offset + 29];
    DRC->DEV_FAULT_CODE=data_process_buf[offset + 30];
    DRC->DEV_BLE_ID=(data_process_buf[offset + 31]<<8)|data_process_buf[offset + 32];
}

/**
 * @brief  串口数据处理服务
 * @param  Null
 * @return Null
 * @note   在MCU主函数while循环中调用该函数
 */
void device_uart_service(void)
{
    static short rx_in = 0;
    unsigned short offset = 0;
    unsigned short rx_value_len = 0;

    while ((rx_in < sizeof(data_process_buf)) && with_data_rxbuff() > 0)
    {
        data_process_buf[rx_in++] = take_byte_rxbuff();
    }

    if (rx_in < DEV_PROTOCOL_HEAD)
        return;

    while ((rx_in - offset) >= DEV_PROTOCOL_HEAD)
    {
        if (data_process_buf[offset + HEAD_FIRST] != DEV_FRAME_FIRST)
        {
            offset++;
            continue;
        }
        if (data_process_buf[offset + DEV_ADDR] != DEV_FRAME_SECOND)
        {
            offset++;
            continue;
        }
        rx_value_len = data_process_buf[offset + DATA_LEN];
        if (rx_value_len > sizeof(data_process_buf) + DEV_PROTOCOL_HEAD)
        {
            offset += 2;
            continue;
        }

        if ((rx_in - offset) < rx_value_len)
        {
            break;
        }

        //数据接收完成
        if (get_check_sum((unsigned char *)(data_process_buf+3) + offset, rx_value_len,DEV2MOD) != data_process_buf[offset + rx_value_len + 3])
        {
            //校验出错
            SEGGER_RTT_printf(0,"sum_check_error!");
            offset += 4;
            continue;
        }
        device_data_handle(offset,&DRC);
        dev_recv_flag=1;
        offset += rx_value_len + 4;
    } // end while
    if(rx_in>=offset)
    {
        rx_in -= offset;
    }
    if (rx_in > 0)
    {
        memcpy((char *)data_process_buf, (const char *)data_process_buf + offset, rx_in);
    }
    return;
}

/*****************************************************************************
函数名称 : uart_transmit_output
功能描述 : 发数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 : 请将MCU串口发送函数填入该函数内,并将接收到的数据作为参数传入串口发送函数
*****************************************************************************/
void device_uart_transmit_output(unsigned char value)
{
    Uart3SendData(&value,1);
}

/*****************************************************************************
函数名称 : uart_write_data
功能描述 : 向uart写入连续数据
输入参数 : in:发送缓存指针
           len:数据发送长度
返回参数 : 无
*****************************************************************************/
static void uart_write_data(unsigned char *in, unsigned short len)
{
    if ((NULL == in) || (0 == len))
    {
        return;
    }

    while (len--)
    {
        device_uart_transmit_output(*in);
        in++;
    }
}

/**
 * @brief  向串口发送一帧数据
 * @param[in] {fr_type} 帧类型
 * @param[in] {fr_ver} 帧版本
 * @param[in] {len} 数据长度
 * @return Null
 */
void device_uart_write_frame(void)
{
    static uint8_t index = 0;
    uint8_t check_sum = 0;
    uint8_t len;
    //循环发送取数据指令
    if (bsp_CheckTimer(TMR_DEVICE_LOOP))
    {
        uart_tx_buf[HEAD_FIRST] = MOD_FRAME_FIRST;
        uart_tx_buf[DEV_ADDR] = MOD_FRAME_SECOND;
        uart_tx_buf[DATA_LEN] = MOD_FRAME_TATOL_LEN;
        uart_tx_buf[BAT_STATE_FRAME] = get_onebus_bat_sta();    //从一线通得到电池状态
        uart_tx_buf[BAT_CORE_FRAME]= bt24_get_bat_type();       //得到电池类型
        uart_tx_buf[BAT_SPE_FRAME]= 0;
        uart_tx_buf[BAT_CAP_FRAME]= 0;
        uart_tx_buf[BAT_SN_FRAME]= 0;
        uart_tx_buf[BAT_SN_FRAME+1]= 0;
        uart_tx_buf[BAT_SN_FRAME+2]= 0;
        uart_tx_buf[BAT_SN_FRAME+3]= 0;
        uart_tx_buf[BAT_OPERA_FRAME]= bt24_get_bat_opera_status();
        uart_tx_buf[BAT_DISCH_CUR_FRAME]= bt24_get_bat_disch_cur()>>8;
        uart_tx_buf[BAT_DISCH_CUR_FRAME+1]= bt24_get_bat_disch_cur();
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME]= bt24_get_bat_disch_end_vol()>>24;
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME+1]= bt24_get_bat_disch_end_vol()>>16;
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME+2]= bt24_get_bat_disch_end_vol()>>8;
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME+3]= bt24_get_bat_disch_end_vol();
        uart_tx_buf[BAT_CH_CUR_FRAME]= bt24_get_bat_ch_cur()>>8;
        uart_tx_buf[BAT_CH_CUR_FRAME+1]= bt24_get_bat_ch_cur();
        uart_tx_buf[BAT_CH_END_VOL_FRAME]= bt24_get_bat_ch_end_vol()>>24;
        uart_tx_buf[BAT_CH_END_VOL_FRAME+1]= bt24_get_bat_ch_end_vol()>>16;
        uart_tx_buf[BAT_CH_END_VOL_FRAME+2]= bt24_get_bat_ch_end_vol()>>8;
        uart_tx_buf[BAT_CH_END_VOL_FRAME+3]= bt24_get_bat_ch_end_vol();
        uart_tx_buf[BAT_CH_END_CUR_FRAME]= 0;
        uart_tx_buf[BAT_SET_MODE_FRAME]= bt24_get_bat_set_mode();
        check_sum=get_check_sum((unsigned char *)(uart_tx_buf+3), MOD_FRAME_TATOL_LEN,MOD2DEV);
        uart_tx_buf[BAT_TO_DEV_CRC]= check_sum;
    }
    Uart3SendData((unsigned char *)uart_tx_buf, (MOD_FRAME_TATOL_LEN+4));
}

/*****************************************************************************
函数名称 : get_device_work_station
功能描述 : 查询检测仪工作状态
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_work_station(void)
{
    if((DRC.DEV_STATUS==0x01)||(DRC.DEV_STATUS==0x02))  
    {
        DRC.DEV_STATUS=0x01;
        return DRC.DEV_STATUS;
    }
    if(DRC.DEV_STATUS==0x03)
    {
        DRC.DEV_STATUS=0x03;
        return DRC.DEV_STATUS;
    }
    if(DRC.DEV_STATUS==0x05)
    {
        DRC.DEV_STATUS=0x05;
        return DRC.DEV_STATUS;
    }
    if(DRC.DEV_STATUS==0)
    {
        DRC.DEV_STATUS=0;
        return DRC.DEV_STATUS;
    }
    DRC.DEV_STATUS=DRC.DEV_FAULT_CODE;
    if((DRC.DEV_STATUS==1)||(DRC.DEV_STATUS==2))
    {
        DRC.DEV_STATUS=0x05;
        return DRC.DEV_STATUS; 
    }
    return DRC.DEV_STATUS; 
}

/*****************************************************************************
函数名称 : get_device_vol_value
功能描述 : 查询设备电压
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint32_t get_device_vol_value(void)
{
    return DRC.DEV_GET_VOL;
}

/*****************************************************************************
函数名称 : get_device_cur_value
功能描述 : 查询检测仪工作电流
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint16_t get_device_cur_value(void)
{
    return DRC.DEV_GET_CUR;
}
/*****************************************************************************
函数名称 : get_device_bat_type
功能描述 : 查询电池类型
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_bat_type(void)
{
    return DRC.DEV_BAT_TYPE;
}
/*****************************************************************************
函数名称 : get_device_fault_code
功能描述 : 电池故障
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint32_t get_device_fault_code(void)
{
    uint32_t temp;  
    if(DRC.DEV_FAULT_CODE==0x01)
    {
        temp= 0x0400;    
    }   
    else if(DRC.DEV_FAULT_CODE==0x02)
    {    
        temp= 0x0800; 
    }
    return temp;
}
/*****************************************************************************
函数名称 : get_device_send_ble_id
功能描述 : 得到BLE ID
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint16_t get_device_send_ble_id(void)
{
    // return 0x1234;
    return DRC.DEV_BLE_ID;
}
/*****************************************************************************
函数名称 : get_device_send_ble_id
功能描述 : 得到BLE ID
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_recv_flag(void)
{
    return dev_recv_flag;
}
/*****************************************************************************
函数名称 : device_drc_init
功能描述 : 初始化DRC参数
输入参数 : 无
返回参数 : 无
*****************************************************************************/
void device_drc_init(void)
{
    DRC.DEV_FAULT_CODE=0;
}
/*****************************************************************************
函数名称 : get_device_time_h
功能描述 : 查询小时
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_time_h(void)
{
    return DRC.DEV_TIME_H;
}
/*****************************************************************************
函数名称 : get_device_time_m
功能描述 : 查询分钟
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_time_m(void)
{
    return DRC.DEV_TIME_M;
}
/*****************************************************************************
函数名称 : get_device_time_s
功能描述 : 查询秒
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_time_s(void)
{
    return DRC.DEV_TIME_S;
}
/*****************************************************************************
函数名称 : get_device_sum_cap
功能描述 : 累计容量
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint16_t get_device_sum_cap(void)
{
    return DRC.DEV_GET_SPE;
}