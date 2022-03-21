#include "includes.h"
#include "chargeProtocol.h"
 
#define ch_len	0x0D	//���ݳ���

uint8_t device_bat_param_array[14]={0};

ch_param cp; 

/**
 * @brief  �ڴ濽��
 * @param[out] {dest} Ŀ���ַ
 * @param[in] {src} Դ��ַ
 * @param[in] {count} �������ݸ���
 * @return ���ݴ�������Դ��ַ
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
 * @brief  ����У���
 * @param[in] {pack} ����Դָ��
 * @param[in] {pack_len} ����У��ͳ���
 * @return У���
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
 * @brief  Э�鴮�ڳ�ʼ������
 * @param  Null
 * @return Null
 * @note   ��MCU��ʼ�������е��øú���
 */
void device_protocol_init(void)
{
    rx_buf_in = (unsigned char *)uart_rx_buf;
    rx_buf_out = (unsigned char *)uart_rx_buf;
    
    stop_update_flag = DISABLE; 

}
/**
 * @brief  ���ڽ��������ݴ洦��
 * @param[in] {value} �����յ���1�ֽ�����
 * @return Null
 * @note   ��MCU���ڴ������е��øú���,�������յ���������Ϊ��������
 */
void uart_receive_input(unsigned char value)
{
    if(1 == rx_buf_out - rx_buf_in) { 
        //���ڽ��ջ�������
    }else if((rx_buf_in > rx_buf_out) && ((rx_buf_in - rx_buf_out) >= sizeof(uart_rx_buf))) {
        //���ڽ��ջ�������
    }else {
        //���ڽ��ջ���δ��
        if(rx_buf_in >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf))) {
            rx_buf_in = (unsigned char *)(uart_rx_buf);
        }
        
        *rx_buf_in ++ = value;
    }
}

/**
 * @brief  �жϴ��ڽ��ջ������Ƿ�������
 * @param  Null
 * @return �Ƿ�������
 */
unsigned char with_data_rxbuff(void)
{
    if(rx_buf_in != rx_buf_out)
        return 1;
    else
        return 0;
}

/**
 * @brief  ��ȡ����1�ֽ�����
 * @param  Null
 * @return Read the data
 */
unsigned char take_byte_rxbuff(void)
{
    unsigned char value;
    
    if(rx_buf_out != rx_buf_in) {
        //������
        if(rx_buf_out >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf))) {
            //�����Ѿ���ĩβ
            rx_buf_out = (unsigned char *)(uart_rx_buf);
        }
        
        value = *rx_buf_out ++;   
    }
    
    return value;
}
/*****************************************************************************
�������� : data_handle
�������� : ����֡����
������� : offset:������ʼλ
���ز��� : ��
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
 * @brief  �������ݴ������
 * @param  Null
 * @return Null
 * @note   ��MCU������whileѭ���е��øú���
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
			
			//���ݽ������
			if(get_check_sum((unsigned char *)(data_process_buf+1) + offset,rx_value_len +1) != data_process_buf[offset + rx_value_len+2]) {
					//У�����
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
�������� : uart_transmit_output
�������� : �����ݴ���
������� : value:�����յ��ֽ�����
���ز��� : ��
ʹ��˵�� : �뽫MCU���ڷ��ͺ�������ú�����,�������յ���������Ϊ�������봮�ڷ��ͺ���
*****************************************************************************/
void uart_transmit_output(unsigned char value)
{
//   USART_SendData(USART3, value);
// 	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){} 
}

/*****************************************************************************
�������� : uart_write_data
�������� : ��uartд����������
������� : in:���ͻ���ָ��
           len:���ݷ��ͳ���
���ز��� : ��
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
 * @brief  �򴮿ڷ���һ֡����
 * @param[in] {fr_type} ֡����
 * @param[in] {fr_ver} ֡�汾
 * @param[in] {len} ���ݳ���
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
�������� : get_device_work_station
�������� : ��ѯ����ǹ���״̬
������� : ��
���ز��� : ��
*****************************************************************************/
uint8_t get_device_work_station(void)
{
	return device_bat_param_array[2];
}

/*****************************************************************************
�������� : get_device_work_station
�������� : ��ѯ����ǹ���״̬
������� : ��
���ز��� : ��
*****************************************************************************/
float get_device_vol_value(void)
{
	return device_bat_param_array[3]+(((float)(device_bat_param_array[4]))/100);
}

/*****************************************************************************
�������� : get_device_work_station
�������� : ��ѯ����ǹ���״̬
������� : ��
���ز��� : ��
*****************************************************************************/
uint16_t get_device_cur_value(void)
{
	return (device_bat_param_array[5]<<8)|device_bat_param_array[6];
}
/*****************************************************************************
�������� : get_device_bat_type
�������� : ��ѯ�������
������� : ��
���ز��� : ��
*****************************************************************************/
uint8_t get_device_bat_type(void)
{
	return device_bat_param_array[2];
}
/*****************************************************************************
�������� : get_device_fault_code
�������� : ��ѯ�������
������� : ��
���ز��� : ��
*****************************************************************************/
uint8_t get_device_fault_code(void)
{
	return device_bat_param_array[2];
}