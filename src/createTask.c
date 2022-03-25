#include "includes.h"

osThreadId defaultTaskHandle;
osThreadId espTaskHandle;
osThreadId onebusTaskHandle;
osThreadId BT24TaskHandle;
osThreadId ATL485TaskHandle;
osThreadId MFDeviceTaskHandle;

osThreadId defaultTaskHandle;
osMessageQId QueueTxHandle;
osSemaphoreId CommandSemHandle;

void StartDefaultTask(void const * argument);
void EspTask(void const * argument);
void OneBusTask(void const * argument);
void BT24Task(void const * argument);
void Atl485Task(void const * argument);
void MFDeviceTask(void const * argument);

void CreateTask()
{
	/* Create the semaphores(s) */
	/* definition and creation of CommandSem */
	osSemaphoreDef(CommandSem);
	CommandSemHandle = osSemaphoreCreate(osSemaphore(CommandSem), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of QueueTx */
	osMessageQDef(QueueTx, 80, uint32_t);
	QueueTxHandle = osMessageCreate(osMessageQ(QueueTx), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of usbTask */
	osThreadDef(EspTask, EspTask, osPriorityNormal, 0, 256);  
	espTaskHandle = osThreadCreate(osThread(EspTask), NULL);

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
* @brief  	ESP32协议
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void EspTask(void const * argument)
{
  /* USER CODE BEGIN UsbTask */
	uint16_t dataSize = 0;
//	uint16_t i,j,k;
//	uint8_t a = 0;
	uint32_t pbufAddr;
	osEvent event;
	uint8_t result;

	
//	uint16_t ledFlashCount = 0;


  /* Infinite loop */
  for(;;)
  {
		event = osMessageGet(QueueTxHandle, 0xffffffff);
		
		if(event.status == osEventMessage)

		{
		}
  }
  /* USER CODE END UsbTask */
}
/** 
* @brief  	bms一线通协议任务
* @param  	
* @param  	
* @param   
* @retval  	None
* @warning 	None
* @example
**/
void OneBusTask(void const * argument)
{	
	bmsOneBusInit();
	TIM_Configuration(47,99);	//(47+1)*(99+1)/48000000	
	for(;;)
	{
		osDelay(1000);
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
	Uart0Init(ATL_ModbusRecvHandle);
	bsp_StartCallBackTimer(TMR_ATL485,ATLModbusPoll,100);	//100ms
	for(;;)
	{
		osDelay(10);
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
	Uart3Init(device_uart_receive_input); 
	bsp_StartCallBackTimer(TMR_DEVICE_LOOP,device_uart_write_frame,1000);
	for(;;)
	{
		device_uart_service();
		osDelay(10);
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