#include "includes.h"
#include "chargeProtocol.h"
 
#define ch_len	0x0D	//数据长度

uint8_t device_bat_param_array[14]={0};

ch_param cp; 

/**
 * @brief  内存拷贝
 * @param[out] {dest} 目标地址
 * @param[in] {src} 源地址
 * @param[in] {count} 拷贝数据个数
 * @return 数据处理完后的源地址
 */
void *my_memcpy(void *dest, const void *src, unsigned short count)  
{  
    unsigned char *pdest = (unsigned char *)dest;  
    const unsigned char *psrc  = (const unsigned char *)src;  
    unsigned short i;
    
    if(dest == NULL || src == NULL) { 
        return NULL;
    }
    
    if((pdest <= psrc) || (pdest > psrc + count)) {  
        for(i = 0; i < count; i ++) {  
            pdest[i] = psrc[i];  
        }  
    }else {
        for(i = count; i > 0; i --) {  
            pdest[i - 1] = psrc[i - 1];  
        }  
    }  
    
    return dest;  
}
/**
 * @brief  计算校验和
 * @param[in] {pack} 数据源指针
 * @param[in] {pack_len} 计算校验和长度
 * @return 校验和
 */
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len)
{
    unsigned short i;
    unsigned char check_sum = 0;
    
    for(i = 0; i < pack_len; i ++) {
        check_sum += *pack ++;
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
    
    stop_update_flag = DISABLE; 

}
/**
 * @brief  串口接收数据暂存处理
 * @param[in] {value} 串口收到的1字节数据
 * @return Null
 * @note   在MCU串口处理函数中调用该函数,并将接收到的数据作为参数传入
 */
void uart_receive_input(unsigned char value)
{
    if(1 == rx_buf_out - rx_buf_in) { 
        //串口接收缓存已满
    }else if((rx_buf_in > rx_buf_out) && ((rx_buf_in - rx_buf_out) >= sizeof(uart_rx_buf))) {
        //串口接收缓存已满
    }else {
        //串口接收缓存未满
        if(rx_buf_in >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf))) {
            rx_buf_in = (unsigned char *)(uart_rx_buf);
        }
        
        *rx_buf_in ++ = value;
    }
}

/**
 * @brief  判断串口接收缓存中是否有数据
 * @param  Null
 * @return 是否有数据
 */
unsigned char with_data_rxbuff(void)
{
    if(rx_buf_in != rx_buf_out)
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
    
    if(rx_buf_out != rx_buf_in) {
        //有数据
        if(rx_buf_out >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf))) {
            //数据已经到末尾
            rx_buf_out = (unsigned char *)(uart_rx_buf);
        }
        
        value = *rx_buf_out ++;   
    }
    
    return value;
}
/*****************************************************************************
函数名称 : data_handle
功能描述 : 数据帧处理
输入参数 : offset:数据起始位
返回参数 : 无
*****************************************************************************/
void data_handle(unsigned short offset)
{
	device_bat_param_array[2]=data_process_buf[offset+2];
	device_bat_param_array[3]=data_process_buf[offset+3];
	device_bat_param_array[4]=data_process_buf[offset+4];
	device_bat_param_array[5]=data_process_buf[offset+5];
	device_bat_param_array[6]=data_process_buf[offset+6];
}

/**
 * @brief  串口数据处理服务
 * @param  Null
 * @return Null
 * @note   在MCU主函数while循环中调用该函数
 */
void device_uart_service(void)
{
   static unsigned short rx_in = 0;
	unsigned short offset = 0;
	unsigned short rx_value_len = 0;
	
	while((rx_in < sizeof(data_process_buf)) && with_data_rxbuff() > 0) {
			data_process_buf[rx_in ++] = take_byte_rxbuff();
	}
	
	if(rx_in < PROTOCOL_HEAD)
			return;
	
	while((rx_in - offset) >= PROTOCOL_HEAD) {
			if(data_process_buf[offset + HEAD_FIRST] != FRAME_FIRST) {
					offset ++; 
					continue;
			}        
			
			rx_value_len = data_process_buf[offset + DATA_LEN];
			if(rx_value_len > sizeof(data_process_buf) + PROTOCOL_HEAD) {
					offset += 1;
					continue;
			}
			
			if((rx_in - offset) < rx_value_len) {
					break;
			}
			
			//数据接收完成
			if(get_check_sum((unsigned char *)(data_process_buf+1) + offset,rx_value_len +1) != data_process_buf[offset + rx_value_len+2]) {
					//校验出错
					//printf("crc error (crc:0x%X  but data:0x%X)\r\n",get_check_sum((unsigned char *)wifi_data_process_buf + offset,rx_value_len - 1),wifi_data_process_buf[offset + rx_value_len - 1]);
					offset += 2;
					continue;
			}
			
      data_handle(offset);
			offset += rx_value_len+4;
    }//end while
    rx_in -= offset;
    if(rx_in > 0) {  
        my_memcpy((char *)data_process_buf, (const char *)data_process_buf + offset, rx_in);
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
void uart_transmit_output(unsigned char value)
{
//   USART_SendData(USART3, value);
// 	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){} 
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
  if((NULL == in) || (0 == len))
  {
    return;
  }
  
  while(len --)
  {
    uart_transmit_output(*in);
    in ++;
  }
}

/**
 * @brief  向串口发送一帧数据
 * @param[in] {fr_type} 帧类型
 * @param[in] {fr_ver} 帧版本
 * @param[in] {len} 数据长度
 * @return Null
 */
void device_uart_write_frame(float *ch_p,unsigned char *ch_s,unsigned short len)
{
    unsigned char check_sum = 0;
    
    // uart_tx_buf[HEAD_FIRST] = 0x5A;
    // uart_tx_buf[DATA_LEN] = len;
    // uart_tx_buf[FUNC_TYPE]=ch_s[0];
    // uart_tx_buf[BAT_DIS_CUR_H]=(int)(ch_p[0]*100)/100;
    // uart_tx_buf[BAT_DIS_CUR_L]=(int)(ch_p[0]*100)%100;
    // uart_tx_buf[BAT_DIS_VOL_H]=(int)(ch_p[1]*100)/100;
    // uart_tx_buf[BAT_DIS_VOL_L]=(int)(ch_p[1]*100)%100;
    // uart_tx_buf[BAT_CH_CUR_H]=(int)(ch_p[2]*100)/100;
    // uart_tx_buf[BAT_CH_CUR_L]=(int)(ch_p[2]*100)%100;
    // uart_tx_buf[BAT_CH_VOL_H]=(int)(ch_p[3]*100)/100;
    // uart_tx_buf[BAT_CH_VOL_L]=(int)(ch_p[3]*100)%100;
    // uart_tx_buf[BAT_CH_CUR_POINT_H]=(int)(ch_p[4]*100)/100;
    // uart_tx_buf[BAT_CH_CUR_POINT_L]=(int)(ch_p[4]*100)%100;
    
    // check_sum = get_check_sum((unsigned char *)(uart_tx_buf+1), len+1);
	// 	uart_tx_buf[len+2] = check_sum;
	// 	uart_tx_buf[len+3]=0x5B;
    uart_write_data((unsigned char *)uart_tx_buf, (len+4));
}

/*****************************************************************************
函数名称 : get_device_work_station
功能描述 : 查询检测仪工作状态
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_work_station(void)
{
	return device_bat_param_array[2];
}

/*****************************************************************************
函数名称 : get_device_work_station
功能描述 : 查询检测仪工作状态
输入参数 : 无
返回参数 : 无
*****************************************************************************/
float get_device_vol_value(void)
{
	return device_bat_param_array[3]+(((float)(device_bat_param_array[4]))/100);
}

/*****************************************************************************
函数名称 : get_device_work_station
功能描述 : 查询检测仪工作状态
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint16_t get_device_cur_value(void)
{
	return (device_bat_param_array[5]<<8)|device_bat_param_array[6];
}
/*****************************************************************************
函数名称 : get_device_bat_type
功能描述 : 查询电池类型
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_bat_type(void)
{
	return device_bat_param_array[2];
}
/*****************************************************************************
函数名称 : get_device_fault_code
功能描述 : 查询电池类型
输入参数 : 无
返回参数 : 无
*****************************************************************************/
uint8_t get_device_fault_code(void)
{
	return device_bat_param_array[2];
}