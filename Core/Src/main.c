/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_UART_MUTEX  1   // set to 0 to observe skipped sequence numbers without the mutex
#define BUTTON_SYNC 0	// 0: binary sem, 1: counting sem, 2: CMSIS thread flags, 3: native task notification
#define STACK_OVERFLOW_TEST  0   // 1: blinkTask allocates a large local array to overflow its stack
#define USE_GATEKEEPER  1   // 0: tasks print directly (blocking UART + mutex), 1: log queue + gatekeeper task with UART DMA
#define LOG_MSG_LEN     64
#define FLAG_TX_DONE    0x01
#define WDG_FAULT_TEST  0   // 1: sensorTask hangs after 10 readings
#define ALIVE_BLINK   0x01U
#define ALIVE_SENSOR  0x02U
#define ALIVE_PRINT   0x04U
#define ALIVE_ALL     (ALIVE_BLINK | ALIVE_SENSOR | ALIVE_PRINT)

#if (BUTTON_SYNC < 0) || (BUTTON_SYNC > 3)
#error "BUTTON_SYNC must be 0..3"
#endif

#if USE_UART_MUTEX
#define UART_LOCK()    osMutexAcquire(uartMutexHandle, osWaitForever)
#define UART_UNLOCK()  osMutexRelease(uartMutexHandle)
#else
#define UART_LOCK()
#define UART_UNLOCK()
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for blinkTask */
osThreadId_t blinkTaskHandle;
const osThreadAttr_t blinkTask_attributes = {
  .name = "blinkTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for printTask */
osThreadId_t printTaskHandle;
const osThreadAttr_t printTask_attributes = {
  .name = "printTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for wdgTask */
osThreadId_t wdgTaskHandle;
const osThreadAttr_t wdgTask_attributes = {
  .name = "wdgTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for tempQueue */
osMessageQueueId_t tempQueueHandle;
const osMessageQueueAttr_t tempQueue_attributes = {
  .name = "tempQueue"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* Definitions for buttonSem */
osSemaphoreId_t buttonSemHandle;
const osSemaphoreAttr_t buttonSem_attributes = {
  .name = "buttonSem"
};
/* Definitions for buttonCountSem */
osSemaphoreId_t buttonCountSemHandle;
const osSemaphoreAttr_t buttonCountSem_attributes = {
  .name = "buttonCountSem"
};
/* USER CODE BEGIN PV */
osMessageQueueId_t logQueueHandle;
osEventFlagsId_t aliveFlagsHandle;
volatile uint32_t log_dropped = 0;
volatile uint32_t blink_log_max_us = 0;
__attribute__((section(".noinit"))) volatile uint32_t wdg_magic;
__attribute__((section(".noinit"))) volatile uint32_t wdg_missing;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
void StartBlinkTask(void *argument);
void StartSensorTask(void *argument);
void StartPrintTask(void *argument);
void StartButtonTask(void *argument);
void StartLogTask(void *argument);
void StartWdgTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "task.h"

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);
	return len;
}

static void log_vprintf(uint32_t timeout, const char *fmt, va_list ap)
{
  char msg[LOG_MSG_LEN];
  vsnprintf(msg, sizeof(msg), fmt, ap);
  if (osMessageQueuePut(logQueueHandle, msg, 0, timeout) != osOK)
  {
    log_dropped++;
  }
}

void log_printf(const char *fmt, ...)            /* drop if full */
{
  va_list ap;
  va_start(ap, fmt);
  log_vprintf(0, fmt, ap);
  va_end(ap);
}

void log_printf_wait(uint32_t timeout, const char *fmt, ...)  /* wait up to timeout */
{
  va_list ap;
  va_start(ap, fmt);
  log_vprintf(timeout, fmt, ap);
  va_end(ap);
}

#if USE_GATEKEEPER
#define LOG(...)            log_printf(__VA_ARGS__)
#define LOG_IMPORTANT(...)  log_printf_wait(100, __VA_ARGS__)
#else
#define LOG(...)            do { UART_LOCK(); printf(__VA_ARGS__); UART_UNLOCK(); } while (0)
#define LOG_IMPORTANT(...)  LOG(__VA_ARGS__)
#endif

#define WDG_INFO_MAGIC  0xDEAD0000U

static void wdg_info_save(uint32_t missing)
{
  /* 1) RTC backup register */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP0R = WDG_INFO_MAGIC | (missing & 0xFFFFU);

  /* 2) .noinit RAM */
  wdg_magic   = WDG_INFO_MAGIC;
  wdg_missing = missing;
}

static uint32_t wdg_info_load(void)
{
  return RTC->BKP0R;
}

static void wdg_info_clear(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP0R = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
  {
    printf("\r\n*** Reset cause: IWDG ***\r\n");

    uint32_t bkp = RTC->BKP0R;
    if ((bkp & 0xFFFF0000U) == WDG_INFO_MAGIC)
      printf("*** [BKP]    last missing 0x%02lx ***\r\n", bkp & 0xFFFFU);
    else
      printf("*** [BKP]    no valid info ***\r\n");

    if (wdg_magic == WDG_INFO_MAGIC)
      printf("*** [noinit] last missing 0x%02lx ***\r\n", wdg_missing);
    else
      printf("*** [noinit] no valid info ***\r\n");
  }
  else
  {
    printf("\r\n*** Reset cause: power-on / pin ***\r\n");
  }

  /* clear both */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP0R = 0;
  wdg_magic = 0;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  __HAL_DBGMCU_FREEZE_IWDG();   // stop IWDG while the core is halted by the debugger
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of buttonSem */
  buttonSemHandle = osSemaphoreNew(1, 1, &buttonSem_attributes);

  /* creation of buttonCountSem */
  buttonCountSemHandle = osSemaphoreNew(10, 0, &buttonCountSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of tempQueue */
  tempQueueHandle = osMessageQueueNew (8, sizeof(int32_t), &tempQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  logQueueHandle = osMessageQueueNew(16, LOG_MSG_LEN, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blinkTask */
  blinkTaskHandle = osThreadNew(StartBlinkTask, NULL, &blinkTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

  /* creation of printTask */
  printTaskHandle = osThreadNew(StartPrintTask, NULL, &printTask_attributes);

  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);

  /* creation of logTask */
  logTaskHandle = osThreadNew(StartLogTask, NULL, &logTask_attributes);

  /* creation of wdgTask */
  wdgTaskHandle = osThreadNew(StartWdgTask, NULL, &wdgTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  aliveFlagsHandle = osEventFlagsNew(NULL);
  if (aliveFlagsHandle == NULL) { Error_Handler(); }
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    osThreadFlagsSet(logTaskHandle, FLAG_TX_DONE);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin != USER_BUTTON_PIN)
		return;

#if BUTTON_SYNC == 0
	osSemaphoreRelease(buttonSemHandle);
#elif BUTTON_SYNC == 1
	osSemaphoreRelease(buttonCountSemHandle);
#elif BUTTON_SYNC == 2
	osThreadFlagsSet(buttonTaskHandle, 0x01);
#else
	BaseType_t woken = pdFALSE;
	vTaskNotifyGiveFromISR((TaskHandle_t)buttonTaskHandle, &woken);
	portYIELD_FROM_ISR(woken);
#endif

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinkTask */
/**
* @brief Function implementing the blinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
#if STACK_OVERFLOW_TEST
    volatile uint8_t big[1900];
    for (uint32_t i = 0; i < sizeof(big); i++) big[i] = (uint8_t)i;
#endif

	BSP_LED_Toggle(LED2);

	uint32_t c0 = DWT->CYCCNT;
	LOG("[blink] 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\r\n");
	uint32_t us = (DWT->CYCCNT - c0) / (SystemCoreClock / 1000000U);
	if (us > blink_log_max_us) blink_log_max_us = us;

	osEventFlagsSet(aliveFlagsHandle, ALIVE_BLINK);

	osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
	int32_t temp_x10;
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start(&hadc1);
	  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	  {
		  uint32_t raw = HAL_ADC_GetValue(&hadc1);
		  int32_t mv = (int32_t) (raw * 3300U / 4095U);
		  temp_x10 = (mv - 760) * 4 +250;

	      osMessageQueuePut(tempQueueHandle, &temp_x10, 0, 0);
	      osEventFlagsSet(aliveFlagsHandle, ALIVE_SENSOR);

	#if WDG_FAULT_TEST
	      static uint32_t n = 0;
	      if (++n >= 10)
	      {
	        for (;;) { }   // simulate a hung task
	      }
	#endif
	  }
    osDelay(1000);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartPrintTask */
/**
* @brief Function implementing the printTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void *argument)
{
  /* USER CODE BEGIN StartPrintTask */
  /* Infinite loop */
	int32_t t;
	uint32_t seq = 0;
  for(;;)
  {


    if(osMessageQueueGet(tempQueueHandle, &t, NULL, osWaitForever) == osOK)
    {
    	LOG_IMPORTANT("[%lu] temp=%ld.%ld C max_log=%luus drop=%lu\r\n",
            seq++, t / 10, t % 10, blink_log_max_us, log_dropped);

    	osEventFlagsSet(aliveFlagsHandle, ALIVE_PRINT);

        if ((seq % 10) == 0)
        {
          LOG_IMPORTANT("stk free: blink=%lu sens=%lu prt=%lu btn=%lu log=%lu\r\n",
                        osThreadGetStackSpace(blinkTaskHandle),
                        osThreadGetStackSpace(sensorTaskHandle),
                        osThreadGetStackSpace(printTaskHandle),
                        osThreadGetStackSpace(buttonTaskHandle),
                        osThreadGetStackSpace(logTaskHandle));
        }
    }
  }
  /* USER CODE END StartPrintTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
	  uint32_t presses = 0;

#if BUTTON_SYNC == 0
	  	  osSemaphoreAcquire(buttonSemHandle, 0);	//	drain initial token
#endif

	  for(;;)
	  {
#if BUTTON_SYNC == 0
		  osSemaphoreAcquire(buttonSemHandle, osWaitForever);
#elif BUTTON_SYNC == 1
		  osSemaphoreAcquire(buttonCountSemHandle, osWaitForever);
#elif BUTTON_SYNC == 2
		  osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
#else
		  ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
#endif

	    presses++;

	    LOG_IMPORTANT(">>> button: %lu\r\n", presses);

	    osDelay(2000);   // simulate slow processing
	  }
  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
* @brief Function implementing the logTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogTask */
void StartLogTask(void *argument)
{
  /* USER CODE BEGIN StartLogTask */
	#if USE_GATEKEEPER
	  static char buf[LOG_MSG_LEN];
	  for(;;)
	  {
	    osMessageQueueGet(logQueueHandle, buf, NULL, osWaitForever);
	    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, strlen(buf));
	    osThreadFlagsWait(FLAG_TX_DONE, osFlagsWaitAny, osWaitForever);
	  }
	#else
	  osThreadExit();
	#endif
  /* USER CODE END StartLogTask */
}

/* USER CODE BEGIN Header_StartWdgTask */
/**
* @brief Function implementing the wdgTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWdgTask */
void StartWdgTask(void *argument)
{
	  /* USER CODE BEGIN StartWdgTask */
	  for(;;)
	  {
	    uint32_t f = osEventFlagsWait(aliveFlagsHandle, ALIVE_ALL, osFlagsWaitAll, 1500);

	    if ((f & osFlagsError) == 0)
	    {
	      HAL_IWDG_Refresh(&hiwdg);          // every monitored task checked in
	    }
	    else
	    {
	      uint32_t got = osEventFlagsGet(aliveFlagsHandle);
	      uint32_t missing = ALIVE_ALL & ~got;
	      wdg_info_save(missing);
	      LOG_IMPORTANT("WDG: missing 0x%02lx\r\n", (unsigned long)missing);
	      osEventFlagsClear(aliveFlagsHandle, ALIVE_ALL);
	    }
	  }
	  /* USER CODE END StartWdgTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
