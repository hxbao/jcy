#include "includes.h"

osThreadId defaultTaskHandle;
osThreadId espTaskHandle;
osMessageQId QueueTxHandle;
osSemaphoreId CommandSemHandle;

void StartDefaultTask(void const * argument);
void EspTask(void const * argument);

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
	osThreadDef(EspTask, EspTask, osPriorityIdle, 0, 256);
	espTaskHandle = osThreadCreate(osThread(EspTask), NULL);
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

  }
  /* USER CODE END 5 */ 
}

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