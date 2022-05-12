#include "includes.h"

osThreadId defaultTaskHandle;
osThreadId msgTaskHandle;
osThreadId onebusTaskHandle;
osThreadId BT24TaskHandle;
osThreadId ATL485TaskHandle;
osThreadId MFDeviceTaskHandle;
osThreadId ATLCanBusHandle;

osThreadId defaultTaskHandle;
osMessageQId QueueTx;
osSemaphoreId CommandSemHandle;

void StartDefaultTask(void const * argument);
void MsgTask(void const * argument);
void OneBusTask(void const * argument);
void BT24Task(void const * argument);
void Atl485Task(void const * argument);
void MFDeviceTask(void const * argument);
void ATLCANParseTask(void const * argument);

atc_t atc;

void CreateTask()
{
	/* Create the semaphores(s) */
	/* definition and creation of CommandSem */
	osSemaphoreDef(CommandSem);
	CommandSemHandle = osSemaphoreCreate(osSemaphore(CommandSem), 10);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of QueueTx */
	osMessageQDef(QueueTx, 80, uint32_t);
	QueueTx = osMessageCreate(osMessageQ(QueueTx), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of usbTask */
	osThreadDef(MsgTask, MsgTask, osPriorityNormal, 0, 256);  
	msgTaskHandle = osThreadCreate(osThread(MsgTask), NULL);

		/* definition and creation of usbTask */
	osThreadDef(OneBusTask, OneBusTask, osPriorityNormal, 0, 256);
	onebusTaskHandle = osThreadCreate(osThread(OneBusTask), NULL);

			/* definition and creation of usbTask */
	osThreadDef(BT24Task, BT24Task, osPriorityNormal, 0, 256);
	BT24TaskHandle = osThreadCreate(osThread(BT24Task), NULL);

	osThreadDef(Atl485Task, Atl485Task, osPriorityNormal, 0, 256);
	ATL485TaskHandle = osThreadCreate(osThread(Atl485Task), NULL);

	osThreadDef(MFDeviceTask, MFDeviceTask, osPriorityNormal, 0, 256);
	MFDeviceTaskHandle = osThreadCreate(osThread(MFDeviceTask), NULL);

	osThreadDef(ATLCANParseTask, ATLCANParseTask, osPriorityNormal, 0, 256);
	ATLCanBusHandle = osThreadCreate(osThread(ATLCANParseTask), NULL);
}

void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */

  /* USER CODE BEGIN 5 */
	

	//sdram_test();
	//clock select	

  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
  }
  /* USER CODE END 5 */ 
}
/** 
* @brief  	得到接收标记位
* @param  	1:一线通兼容485 2:485协议 3:CAN协议
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
uint8_t msgLoop(void)
{
	static uint8_t msgID;
	if(bsp_CheckTimer(TMR_MAIN))  
	{
		msgID=get_onebus_flag()|get_atl485_flag();
		if(msgID!=0)	//清除接收标志位
		{
			get_atl485_clear();
			get_onebus_clear();
		}
	}
	return msgID;
}
/** 
* @brief  	消息队列任务
* @param  	1:一线通兼容485 2:485协议 3:CAN协议
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void MsgTask(void const * argument)
{
/* USER CODE BEGIN UsbTask */
	uint16_t dataSize = 0;
//	uint16_t i,j,k;
//	uint8_t a = 0;
	uint32_t pbufAddr;
	osEvent evt;
	uint8_t result;
	bsp_StartCallBackTimer(TMR_MAIN,(void *)msgLoop,2000);
  /* Infinite loop */
	for(;;)
	{
		result=msgLoop(); 
		evt.status=osMessagePut(QueueTx,(uint32_t)result,osWaitForever); 
		// SEGGER_RTT_printf(0,"osMessagePut ID %d\r\n",result);
		osDelay(300);
	}
  /* USER CODE END UsbTask */
}
/** 
* @brief  	bms一线通 485兼容协议任务
* @param  	
* @param  	   
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void OneBusTask(void const * argument)
{	
	osEvent  evt;
	Uart2Init(ATL_ModbusRecvHandle);
	TIM_Configuration(47,99);	//(47+1)*(99+1)/48000000
	ATLModbusSendSlient();
	bsp_StartCallBackTimer(TMR_ONEBUS_CHECK,ATLOneBusModbusPoll,500);
	// OneBusSetPPMode();	//设置一线通模式
	for(;;)
	{
		evt=osMessageGet(QueueTx,20);
		if(evt.value.v==1)
		{
			// SEGGER_RTT_printf(0,"OneBus_Task_Running mode-%d\r\n",evt.value.v);	
		}
		osDelay(500);
	}
}

/** 
* @brief  	485协议任务
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void Atl485Task(void const * argument)
{
	osEvent  evt;
	Uart0Init(ATL_ModbusRecvHandle);
	bsp_StartCallBackTimer(TMR_ATL485,ATLModbusPoll,250);	//100ms
	for(;;)
	{
		evt=osMessageGet(QueueTx,20);
		if(evt.value.v==2)
		{
			// SEGGER_RTT_printf(0,"ATL485_Task_Running mode-%d\r\n",evt.value.v);	
		}
		osDelay(500);
	}
}

/** 
* @brief  	ATL CAN Parser
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void ATLCANParseTask(void const * argument)
{
	RYCAN_Init();
	for(;;)
	{
		RYCAN_TxProcess();
		osDelay(500);
	}
}

/** 
* @brief  	主机通讯协议任务
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void MFDeviceTask(void const * argument)
{
	device_protocol_init();
	device_drc_init();
	Uart3Init(device_uart_receive_input); 
	bsp_StartCallBackTimer(TMR_DEVICE_LOOP,device_uart_write_frame,1000);
	for(;;)
	{
		device_uart_service();
		osDelay(10);
	}
}
/**
 * @brief  	AT指令解析
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
void atc_found(char *foundStr)
{
	if (strstr(foundStr, "\r\n") != NULL)
	{
	}
}
/**
 * @brief  	BT24蓝牙协议任务
 * @param
 * @param
 * @param
 * @retval  	None
 * @warning 	None
 * @example
 **/
void BT24Task(void const *argument)
{
	DX_BT24_Init();	
	bt24_protocol_init();
	iap_config_init();	//初始化IAP版本信息
	atc_init(&atc, "MY_ATC", 2, atc_found);
	atc_addSearch(&atc, "\r\n");
	// atc_command(&atc,"AT\r\n",3000,echo_buf,20,1,"OK");
	for (;;)
	{
		// DXBT24_AT_Init(BLE_NAME,sizeof(BLE_NAME));
		atc_loop(&atc);
		bt24_recv_service();
		osDelay(1);
	}
}
/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}   