#include "includes.h"
#include "chargeProtocol.h"

#define ch_len 0x0D //���ݳ���

unsigned char data_process_buf[DATA_PROCESS_LMT]; //�������ݴ�������
unsigned char uart_rx_buf[UART_RECV_BUF_LMT];     //���ڽ��ջ���
unsigned char uart_tx_buf[UART_SEND_BUF_LMT];     //���ڷ��ͻ���
//
unsigned char *rx_buf_in;
unsigned char *rx_buf_out;

unsigned char stop_update_flag; // ENABLE:ֹͣһ�������ϴ�  DISABLE:�ָ�һ�������ϴ�

DeviceResponseCmdErr_t DRC;
/**
 * @brief  ����У���
 * @param[in] {pack} ����Դָ��
 * @param[in] {pack_len} ����У��ͳ���
 * @return У���
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
 * @note   ��MCU���ڴ��������е��øú���,�������յ���������Ϊ��������
 */
void device_uart_receive_input(unsigned char value)
{
    if (1 == rx_buf_out - rx_buf_in)
    {
        //���ڽ��ջ�������
    }
    else if ((rx_buf_in > rx_buf_out) && ((rx_buf_in - rx_buf_out) >= sizeof(uart_rx_buf)))
    {
        //���ڽ��ջ�������
    }
    else
    {
        //���ڽ��ջ���δ��
        if (rx_buf_in >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf)))
        {
            rx_buf_in = (unsigned char *)(uart_rx_buf);
        }

        *rx_buf_in++ = value;
    }
}

/**
 * @brief  �жϴ��ڽ��ջ������Ƿ�������
 * @param  Null
 * @return �Ƿ�������
 */
unsigned char with_data_rxbuff(void)
{
    if (rx_buf_in != rx_buf_out)
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

    if (rx_buf_out != rx_buf_in)
    {
        //������
        if (rx_buf_out >= (unsigned char *)(uart_rx_buf + sizeof(uart_rx_buf)))
        {
            //�����Ѿ���ĩβ
            rx_buf_out = (unsigned char *)(uart_rx_buf);
        }

        value = *rx_buf_out++;
    }

    return value;
}
/*****************************************************************************
�������� : device_data_handle
�������� : ����֡����
������� : offset:������ʼλ
���ز��� : ��
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
    DRC->DEV_GET_SPE=(data_process_buf[offset + 27]<<8)|data_process_buf[offset + 28];
}

/**
 * @brief  �������ݴ�������
 * @param  Null
 * @return Null
 * @note   ��MCU������whileѭ���е��øú���
 */
void device_uart_service(void)
{
    static unsigned short rx_in = 0;
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

        //���ݽ������
        if (get_check_sum((unsigned char *)(data_process_buf+3) + offset, rx_value_len,DEV2MOD) != data_process_buf[offset + rx_value_len + 3])
        {
            //У�����
            SEGGER_RTT_printf(0,"sum_check_error!");
            offset += 4;
            continue;
        }

        device_data_handle(offset,&DRC);
        offset += rx_value_len + 4;
    } // end while
    rx_in -= offset;
    if (rx_in > 0)
    {
        memcpy((char *)data_process_buf, (const char *)data_process_buf + offset, rx_in);
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
void device_uart_transmit_output(unsigned char value)
{
    Uart3SendData(&value,1);
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
 * @brief  �򴮿ڷ���һ֡����
 * @param[in] {fr_type} ֡����
 * @param[in] {fr_ver} ֡�汾
 * @param[in] {len} ���ݳ���
 * @return Null
 */
void device_uart_write_frame(void)
{
    static uint8_t index = 0;
    uint8_t check_sum = 0;
    uint8_t len;
    //ѭ������ȡ����ָ��
    if (bsp_CheckTimer(TMR_DEVICE_LOOP))
    {
        uart_tx_buf[HEAD_FIRST] = MOD_FRAME_FIRST;
        uart_tx_buf[DEV_ADDR] = MOD_FRAME_SECOND;
        uart_tx_buf[DATA_LEN] = MOD_FRAME_TATOL_LEN;
        uart_tx_buf[BAT_STATE_FRAME] = get_onebus_bat_sta();    //��һ��ͨ�õ����״̬
        uart_tx_buf[BAT_CORE_FRAME]= 0;
        uart_tx_buf[BAT_SPE_FRAME]= 0;
        uart_tx_buf[BAT_CAP_FRAME]= 0;
        uart_tx_buf[BAT_SN_FRAME]= 0;
        uart_tx_buf[BAT_SN_FRAME+1]= 0;
        uart_tx_buf[BAT_SN_FRAME+2]= 0;
        uart_tx_buf[BAT_SN_FRAME+3]= 0;
        uart_tx_buf[BAT_OPERA_FRAME]= 0;
        uart_tx_buf[BAT_DISCH_CUR_FRAME]= 0;
        uart_tx_buf[BAT_DISCH_CUR_FRAME+1]= 0;
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME]= 0;
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME+1]= 0;
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME+2]= 0;
        uart_tx_buf[BAT_DISCH_END_VOL_FRAME+3]= 0;
        uart_tx_buf[BAT_CH_CUR_FRAME]= 0;
        uart_tx_buf[BAT_CH_CUR_FRAME+1]= 0;
        uart_tx_buf[BAT_CH_END_VOL_FRAME]= 0;
        uart_tx_buf[BAT_CH_END_VOL_FRAME+0]= 0;
        uart_tx_buf[BAT_CH_END_VOL_FRAME+1]= 0;
        uart_tx_buf[BAT_CH_END_VOL_FRAME+2]= 0;
        uart_tx_buf[BAT_CH_END_CUR_FRAME+3]= 0;
        uart_tx_buf[BAT_SET_MODE_FRAME]= 0;
        check_sum=get_check_sum((unsigned char *)(uart_tx_buf+3), MOD_FRAME_TATOL_LEN+1,MOD2DEV);
        uart_tx_buf[BAT_TO_DEV_CRC]= check_sum;
    }
    Uart3SendData((unsigned char *)uart_tx_buf, (MOD_FRAME_TATOL_LEN+4));
}

/*****************************************************************************
�������� : get_device_work_station
�������� : ��ѯ����ǹ���״̬
������� : ��
���ز��� : ��
*****************************************************************************/
uint8_t get_device_work_station(void)
{
    
}

/*****************************************************************************
�������� : get_device_work_station
�������� : ��ѯ����ǹ���״̬
������� : ��
���ز��� : ��
*****************************************************************************/
float get_device_vol_value(void)
{
    
}

/*****************************************************************************
�������� : get_device_work_station
�������� : ��ѯ����ǹ���״̬
������� : ��
���ز��� : ��
*****************************************************************************/
uint16_t get_device_cur_value(void)
{
    
}
/*****************************************************************************
�������� : get_device_bat_type
�������� : ��ѯ�������
������� : ��
���ز��� : ��
*****************************************************************************/
uint8_t get_device_bat_type(void)
{
    
}
/*****************************************************************************
�������� : get_device_fault_code
�������� : ��ѯ�������
������� : ��
���ز��� : ��
*****************************************************************************/
uint8_t get_device_fault_code(void)
{
    
}