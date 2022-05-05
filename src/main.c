/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//Declare all the variables that will be used in the code
int cnt;
unsigned int x, y;
char buffer[200];
unsigned int max_depth = 0;
unsigned int mins, hrs;
enum teller_states {active, on_break, waiting, finish};
enum teller_states teller1_state, teller2_state, teller3_state ;
enum bank_st {open, closed};
enum bank_st bank_state;

unsigned int t1b, t2b, t3b, t1b2, t2b2, t3b2, t1c, t2c, t3c, t1w, t2w, t3w;

int teller1_openforbreak = 0, teller2_openforbreak = 0, teller3_openforbreak = 0;

int time11, time12, time21, time22, time31, time32;

int bankopen = 1, statflag = 0;

int customer_max_wait = 0;
float customer_avg_wait, teller1_waittime, teller2_waittime, teller3_waittime;

int teller1_button =0, teller2_button =0, teller3_button =0, button1flag = 0, button2flag = 0, button3flag = 0;

unsigned int teller1_totalcust = 0, teller1_totaltime = 0, teller1_breakready = 0, teller1_maxtranstime = 0, teller1_maxwait = 0, teller1_totalbreaks = 0, teller1_avgbreak = 0, teller1_lbreak = 1, teller1_sbreak = 4, teller1_breaktime = 0, teller1_buttonbreak = 0;
unsigned int teller2_totalcust = 0, teller2_totaltime = 0, teller2_breakready = 0, teller2_maxtranstime = 0, teller2_maxwait = 0, teller2_totalbreaks = 0, teller2_avgbreak = 0, teller2_lbreak = 1, teller2_sbreak = 4, teller2_breaktime = 0, teller2_buttonbreak = 0;
unsigned int teller3_totalcust = 0, teller3_totaltime = 0, teller3_breakready = 0, teller3_maxtranstime = 0, teller3_maxwait = 0, teller3_totalbreaks = 0, teller3_avgbreak = 0, teller3_lbreak = 1, teller3_sbreak = 4, teller3_breaktime = 0, teller3_buttonbreak = 0;


//int bankopen = 1;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for teller1 */
osThreadId_t teller1Handle;
const osThreadAttr_t teller1_attributes = {
  .name = "teller1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for teller2 */
osThreadId_t teller2Handle;
const osThreadAttr_t teller2_attributes = {
  .name = "teller2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for teller3 */
osThreadId_t teller3Handle;
const osThreadAttr_t teller3_attributes = {
  .name = "teller3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for qtask */
osThreadId_t qtaskHandle;
const osThreadAttr_t qtask_attributes = {
  .name = "qtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for manager */
osThreadId_t managerHandle;
const osThreadAttr_t manager_attributes = {
  .name = "manager",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for mfstask */
osThreadId_t mfstaskHandle;
const osThreadAttr_t mfstask_attributes = {
  .name = "mfstask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for customers */
osMessageQueueId_t customersHandle;
const osMessageQueueAttr_t customers_attributes = {
  .name = "customers"
};
/* USER CODE BEGIN PV */

const char SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0X80,0X90};
/* Byte maps to select digit 1 to 4 */
const char SEGMENT_SELECT[] = {0xF1,0xF2,0xF4,0xF8};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartTeller1(void *argument);
void StartTeller2(void *argument);
void StartTeller3(void *argument);
void QTask(void *argument);
void StartManager(void *argument);
void StartMFS(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_led_1( uint32_t on ) // Function to turn on LED1 on MFS
{
	if ( on ) // a5
		HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_SET);
}

void set_led_2( uint32_t on ) // Function to turn on LED2 on MFS
{
	if ( on ) //a6
		HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_SET);
}

void set_led_3( uint32_t on ) // Function to turn on LED3 on MFS
{
	if ( on ) //a7
		HAL_GPIO_WritePin(SHLD_D11_GPIO_Port, SHLD_D11_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SHLD_D11_GPIO_Port, SHLD_D11_Pin, GPIO_PIN_SET);
}


void shiftOut(uint8_t val)
{
      for(int ii=0x80; ii; ii>>=1) {
    	  HAL_GPIO_WritePin(SHLD_D7_SEG7_Clock_GPIO_Port,SHLD_D7_SEG7_Clock_Pin, GPIO_PIN_RESET);    // clear clock pin
      		if(ii & val)						                                                     // if this bit in `value` is set
      			HAL_GPIO_WritePin(SHLD_D8_SEG7_Data_GPIO_Port, SHLD_D8_SEG7_Data_Pin,GPIO_PIN_SET);  //   set it in shift register
      		else
      			HAL_GPIO_WritePin(SHLD_D8_SEG7_Data_GPIO_Port, SHLD_D8_SEG7_Data_Pin,GPIO_PIN_RESET); 	//   else clear it

      		HAL_GPIO_WritePin(SHLD_D7_SEG7_Clock_GPIO_Port,SHLD_D7_SEG7_Clock_Pin, GPIO_PIN_SET);       // set clock pin
      	}
}

/* Write a decimal number between 0 and 9 to one of the 4 digits of the display */
void WriteNumberToSegment(int Segment, int Value)
{
  HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_RESET);
  shiftOut(SEGMENT_MAP[Value]);
  shiftOut(SEGMENT_SELECT[Segment] );
  HAL_GPIO_WritePin(SHLD_D4_SEG7_Latch_GPIO_Port, SHLD_D4_SEG7_Latch_Pin, GPIO_PIN_SET);
}


void segdisplay(int x) // Function to display a number on MFS 7 segment display
{
	if (x < 10 && x >= 0)
	{
		WriteNumberToSegment(3, x);
	}

	if (x < 100 && x >= 10)
	{
		WriteNumberToSegment(3, x%10);
		WriteNumberToSegment(2, x/10);
	}

	if (x < 1000 && x >= 100)
	{
		WriteNumberToSegment(3, x%10);
		WriteNumberToSegment(2, (x/10)%10);
		WriteNumberToSegment(1, x/100);
	}

	if (x < 10000 && x >= 1000)
	{
		WriteNumberToSegment(3, x%10);
		WriteNumberToSegment(2, (x/10)%10);
		WriteNumberToSegment(1, (x/100)%10);
		WriteNumberToSegment(0, x/1000);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

teller1_state = waiting; // Teller states are initially in waiting condition
teller2_state = waiting;
teller3_state = waiting;
	t1b2 = HAL_RNG_GetRandomNumber(&hrng); // Generate the first break for teller after starting work
	t1b2 = (t1b2 % 60) + 30;
	t2b2 = HAL_RNG_GetRandomNumber(&hrng);
	t2b2 = (t2b2 % 60) + 30;
	t3b2 = HAL_RNG_GetRandomNumber(&hrng);
	t3b2 = (t3b2 % 60) + 30;

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
  MX_RNG_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of customers */
  customersHandle = osMessageQueueNew (100, sizeof(uint32_t), &customers_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of teller1 */
  teller1Handle = osThreadNew(StartTeller1, NULL, &teller1_attributes);

  /* creation of teller2 */
  teller2Handle = osThreadNew(StartTeller2, NULL, &teller2_attributes);

  /* creation of teller3 */
  teller3Handle = osThreadNew(StartTeller3, NULL, &teller3_attributes);

  /* creation of qtask */
  qtaskHandle = osThreadNew(QTask, NULL, &qtask_attributes);

  /* creation of manager */
  managerHandle = osThreadNew(StartManager, NULL, &manager_attributes);

  /* creation of mfstask */
  mfstaskHandle = osThreadNew(StartMFS, NULL, &mfstask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

		  /* GPIO Ports Clock Enable */
		  __HAL_RCC_GPIOC_CLK_ENABLE();
		  __HAL_RCC_GPIOH_CLK_ENABLE();
		  __HAL_RCC_GPIOA_CLK_ENABLE();
		  __HAL_RCC_GPIOB_CLK_ENABLE();

		  /*Configure GPIO pin Output Level */
		  HAL_GPIO_WritePin(GPIOA, SHLD_D13_Pin|SHLD_D12_Pin|SHLD_D11_Pin|SHLD_D7_SEG7_Clock_Pin
		                          |SHLD_D8_SEG7_Data_Pin, GPIO_PIN_SET);

		  /*Configure GPIO pin Output Level */
		  HAL_GPIO_WritePin(GPIOB, SHLD_D4_SEG7_Latch_Pin|SHLD_D10_Pin, GPIO_PIN_SET);

		  /*Configure GPIO pin : B1_Pin */
		  GPIO_InitStruct.Pin = B1_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_A5_Pin SHLD_A4_Pin */
		  GPIO_InitStruct.Pin = SHLD_A5_Pin|SHLD_A4_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_A0_Pin SHLD_D2_Pin */
		  GPIO_InitStruct.Pin = SHLD_A0_Pin|SHLD_D2_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_A1_Pin SHLD_A2_Pin */
		  GPIO_InitStruct.Pin = SHLD_A1_Pin|SHLD_A2_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
		  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_D13_Pin SHLD_D12_Pin SHLD_D11_Pin SHLD_D7_SEG7_Clock_Pin */
		  GPIO_InitStruct.Pin = SHLD_D13_Pin|SHLD_D12_Pin|SHLD_D11_Pin|SHLD_D7_SEG7_Clock_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_A3_Pin */
		  GPIO_InitStruct.Pin = SHLD_A3_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  HAL_GPIO_Init(SHLD_A3_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_D6_Pin SHLD_D5_Pin */
		  GPIO_InitStruct.Pin = SHLD_D6_Pin|SHLD_D5_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D9_Pin */
		  GPIO_InitStruct.Pin = SHLD_D9_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(SHLD_D9_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D8_SEG7_Data_Pin */
		  GPIO_InitStruct.Pin = SHLD_D8_SEG7_Data_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		  HAL_GPIO_Init(SHLD_D8_SEG7_Data_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D4_SEG7_Latch_Pin */
		  GPIO_InitStruct.Pin = SHLD_D4_SEG7_Latch_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		  HAL_GPIO_Init(SHLD_D4_SEG7_Latch_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pin : SHLD_D10_Pin */
		  GPIO_InitStruct.Pin = SHLD_D10_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		  HAL_GPIO_Init(SHLD_D10_GPIO_Port, &GPIO_InitStruct);

		  /*Configure GPIO pins : SHLD_D15_Pin SHLD_D14_Pin */
		  GPIO_InitStruct.Pin = SHLD_D15_Pin|SHLD_D14_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTeller1 */
/**
  * @brief  Function implementing the teller1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTeller1 */
void StartTeller1(void *argument)
{
  /* USER CODE BEGIN 5 */
	int add = 0;
  /* Infinite loop */
  for(;;)
  {

	  if(osMessageQueueGetCount(customersHandle) > 0 && teller1_state == waiting) // Check for customers and waiting status of teller
	  {
		  teller1_state = active; // Make teller active
		  t1w = 0;
		  osMessageQueueGet(customersHandle, &add, 0, 0xFFFFFFFF); // Get customer from the queue
			int current_time = ((__HAL_TIM_GET_COUNTER(&htim2) - add)/100); // Note down current wait time of customer
			if (current_time > customer_max_wait) { // Check if this is the maximum wait time
				customer_max_wait = current_time;
			}
			customer_avg_wait = (customer_avg_wait + (current_time)); // Add the wait time to total
			t1c = HAL_RNG_GetRandomNumber(&hrng); // Generate transaction time value
			t1c = (t1c % 480) + 30;
			teller1_totaltime += t1c; // Add to total teller transaction time
			if(t1c > teller1_maxtranstime) // Check if this is the maximum transaction time
			{

				teller1_maxtranstime = t1c;

			}

			teller1_totalcust++; // Increment teller customer
			osDelay(t1c*1.66); // Delay by transaction time

			teller1_state = waiting; // Put teller back to waiting state

	  }

	 if( teller1_openforbreak == 1) // Check if teller is available for break
	 {
		 t1b = HAL_RNG_GetRandomNumber(&hrng); // Generate teller break time
		 t1b = (t1b % 4) + 1;
		 teller1_state = on_break; // Put teller in break state
		 teller1_totalbreaks++; // Increment teller break
		 teller1_breaktime += t1b; // Add to total break time of teller
		 teller1_avgbreak = teller1_breaktime/teller1_totalbreaks; // Calculate avg break time of teller

		 if(t1b > teller1_lbreak) // Check if current break is the longest
		 {

			 teller1_lbreak = t1b;

		 }
		 if(t1b < teller1_sbreak) // Check if current break is the shortest
		 {

			 teller1_sbreak = t1b;

		 }

		 osDelay(t1b*100); // Delay until break is over


		 teller1_state = waiting; // Put teller back to waiting state
		 t1b2 = HAL_RNG_GetRandomNumber(&hrng); // Generate next break time
		 t1b2 = (t1b2 % 60) + 30;
		 teller1_openforbreak = 0; // Make teller unavailable for break


	 }

	if(bankopen == 0 && osMessageQueueGetCount(customersHandle)==0 && teller1_state != active) // Check if teller can end his day
	{
		teller1_state = finish; // Put teller in finish state
	}
	cnt = vApplicationIdleHook(); // Get idle hook value
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTeller2 */
/**
* @brief Function implementing the teller2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTeller2 */
void StartTeller2(void *argument)
{
  /* USER CODE BEGIN StartTeller2 */
	int add2 = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGetCount(customersHandle) > 0 && teller2_state == waiting)
	  {
		  teller2_state = active;
		  t2w = 0;
		  osMessageQueueGet(customersHandle, &add2, 0, 0xFFFFFFFF);
			int current_time = ((__HAL_TIM_GET_COUNTER(&htim2) - add2)/100);
			if (current_time > customer_max_wait) {
				customer_max_wait = current_time;
			}
			customer_avg_wait = (customer_avg_wait + (current_time));
			t2c = HAL_RNG_GetRandomNumber(&hrng);
			t2c = (t2c % 480) + 30;
			teller2_totaltime += t2c;
			if(t2c > teller2_maxtranstime)
			{

				teller2_maxtranstime = t2c;

			}

			teller2_totalcust++;
			osDelay(t2c*1.66);

			teller2_state = waiting;

	  }

	 if( teller2_openforbreak == 1)
	 {
		 t2b = HAL_RNG_GetRandomNumber(&hrng);
		 t2b = (t2b % 4) + 1;
		 teller2_state = on_break;
		 teller2_totalbreaks++;
		 teller2_breaktime += t2b;
		 teller2_avgbreak = teller2_breaktime/teller2_totalbreaks;

		 if(t2b > teller2_lbreak)
		 {

			 teller2_lbreak = t2b;

		 }
		 if(t2b < teller2_sbreak)
		 {

			 teller2_sbreak = t2b;

		 }

		 osDelay(t2b*100);


		 teller2_state = waiting;
		 t2b2 = HAL_RNG_GetRandomNumber(&hrng);
		 t2b2 = (t2b2 % 60) + 30;
		 teller2_openforbreak = 0;


	 }

		if(bankopen == 0 && osMessageQueueGetCount(customersHandle)==0 && teller2_state != active)
		{
			teller2_state = finish;
		}
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "> q:%d\r\n",  osMessageQueueGetCount(customersHandle)), 500); // test statement
    //osDelay(1);
		cnt = vApplicationIdleHook();
  }
  /* USER CODE END StartTeller2 */
}

/* USER CODE BEGIN Header_StartTeller3 */
/**
* @brief Function implementing the teller3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTeller3 */
void StartTeller3(void *argument)
{
  /* USER CODE BEGIN StartTeller3 */
int add3 = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGetCount(customersHandle) > 0 && teller3_state == waiting)
	  {
		  teller3_state = active;
		  t3w = 0;
		  osMessageQueueGet(customersHandle, &add3, 0, 0xFFFFFFFF);
			int current_time = ((__HAL_TIM_GET_COUNTER(&htim2) - add3)/100);
			if (current_time > customer_max_wait) {
				customer_max_wait = current_time;
			}
			customer_avg_wait = (customer_avg_wait + (current_time));
			t3c = HAL_RNG_GetRandomNumber(&hrng);
			t3c = (t3c % 480) + 30;
			teller3_totaltime += t3c;
			if(t3c > teller3_maxtranstime)
			{

				teller3_maxtranstime = t3c;

			}

			teller3_totalcust++;
			osDelay(t3c*1.66);

			teller3_state = waiting;

	  }

	 if( teller3_openforbreak == 1)
	 {
		 t3b = HAL_RNG_GetRandomNumber(&hrng);
		 t3b = (t3b % 4) + 1;
		 teller3_state = on_break;
		 teller3_totalbreaks++;
		 teller3_breaktime += t3b;
		 teller3_avgbreak = teller3_breaktime/teller3_totalbreaks;

		 if(t3b > teller3_lbreak)
		 {

			 teller3_lbreak = t3b;

		 }
		 if(t3b < teller3_sbreak)
		 {

			 teller3_sbreak = t3b;

		 }

		 osDelay(t3b*100);


		 teller3_state = waiting;
		 t3b2 = HAL_RNG_GetRandomNumber(&hrng);
		 t3b2 = (t3b2 % 60) + 30;
		 teller3_openforbreak = 0;


	 }

		if(bankopen == 0 && osMessageQueueGetCount(customersHandle)==0 && teller3_state != active)
		{
			teller3_state = finish;
		}
	  //HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "> Teller3\r\n"), 500); // test statement
		cnt = vApplicationIdleHook();
  }
  /* USER CODE END StartTeller3 */
}

/* USER CODE BEGIN Header_QTask */
/**
* @brief Function implementing the qtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_QTask */
void QTask(void *argument)
{
  /* USER CODE BEGIN QTask */
	uint32_t q, address;
  /* Infinite loop */
  for(;;)
  {
	  if(bankopen == 1) // Check if bank is open
	  {

			q = HAL_RNG_GetRandomNumber(&hrng); // Generate time until next customer entry
			q = (q % 4) + 1;
			//q = 1 + q % 4;
			address = __HAL_TIM_GET_COUNTER(&htim2); // Initialize address to be current time
			osMessageQueuePut(customersHandle, &address, 0, 0xFFFFFFFF); // Add customer to queue
			if (osMessageQueueGetCount(customersHandle) > max_depth) { // Check if current depth is max depth
				max_depth = osMessageQueueGetCount(customersHandle);

			}

			osDelay(100 * q); // Delay until next customer enters

	  }
	  cnt = vApplicationIdleHook();
  }
  /* USER CODE END QTask */
}

/* USER CODE BEGIN Header_StartManager */
/**
* @brief Function implementing the manager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartManager */
void StartManager(void *argument)
{
  /* USER CODE BEGIN StartManager */
  /* Infinite loop */
  for(;;)
  {
		if(mins == 60) // Check if an hour is complete
		{
			mins =0; // Change mins back to 0
			if(hrs == 6) // Check if the closing time
			{
				bankopen = 0; // Close bank
				HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>>>>> Bank Closed <<<<<\r\n"), 500);

			}
			hrs++; // Increment hrs
		}
		mins++; // Increment mins

		if(statflag == 0) // Check if status has been printed
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\nTime:%d:%d | ",hrs+9,mins), 500);
		}

		x = x+1;
		//x = (__HAL_TIM_GET_COUNTER(&htim2));
		if(teller1_state == on_break && statflag == 0) // Check if teller 1 is on break
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller1:Break:%d | ",teller1_totalcust), 500);
		}

		else if(teller1_state == active && statflag == 0) // Check if teller 1 is active
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller1:Active:%d | ",teller1_totalcust), 500);
		}


		if(statflag == 0 && osMessageQueueGetCount(customersHandle)==0 && teller1_state == waiting ) // Check if teller is idle and waiting during open hours
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller1:Waiting:%d | ",teller1_totalcust), 500);

			teller1_waittime++; // Increment teller wait time
			t1w++;
			if (t1w > teller1_maxwait) // Check if current wait time is max wait time
			{
				teller1_maxwait = t1w;
			}

		}

		if(teller1_state == finish && statflag == 0)  // Check if teller 1 has finished
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller1:Finish:%d | ",teller1_totalcust), 500);
		}

		if(teller2_state == on_break && statflag == 0)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller2:Break:%d | ",teller2_totalcust), 500);
		}

		else if(teller2_state == active && statflag == 0)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller2:Active:%d | ",teller2_totalcust), 500);
		}


		if(statflag == 0 && osMessageQueueGetCount(customersHandle)==0 && teller2_state == waiting )
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller2:Waiting:%d | ",teller2_totalcust), 500);

			teller2_waittime++;
			t2w++;
			if (t2w > teller2_maxwait)
			{
				teller2_maxwait = t2w;
			}

		}

		if(teller2_state == finish && statflag == 0)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller2:Finish:%d | ",teller2_totalcust), 500);
		}

		if(teller3_state == on_break && statflag == 0)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller3:Break:%d",teller3_totalcust), 500);
		}

		else if(teller3_state == active && statflag == 0)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller3:Active:%d",teller3_totalcust), 500);
		}


		if(statflag == 0 && osMessageQueueGetCount(customersHandle)==0 && teller3_state == waiting )
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller3:Waiting:%d",teller3_totalcust), 500);

			teller3_waittime++;
			t3w++;
			if (t3w > teller3_maxwait)
			{
				teller3_maxwait = t3w;
			}

		}

		if(teller3_state == finish && statflag == 0)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "Teller3:Finish:%d",teller3_totalcust), 500);
		}


		if(statflag == 0) // Update current queue number on 7 segment display
		{
		//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, " |Que:%lu",osMessageQueueGetCount(customersHandle)), 500);
		segdisplay(osMessageQueueGetCount(customersHandle));
		}


		if(teller1_openforbreak == 0) // Check if teller is not open for break
		{
			t1b2--; // Decrement timer count
			if(t1b2 == 0) // If timer count is 0
			{
				teller1_openforbreak = 1; // Teller is eligible for break
			}
		}

		if(teller2_openforbreak == 0)
		{
			t2b2--;
			if(t2b2 == 0)
			{
				teller2_openforbreak = 1;
			}
		}

		if(teller3_openforbreak == 0)
		{
			t3b2--;
			if(t3b2 == 0)
			{
				teller3_openforbreak = 1;
			}
		}



		if(bankopen == 0 && statflag == 0 && osMessageQueueGetCount(customersHandle)==0 && teller3_state == finish && teller2_state == finish && teller1_state == finish) // Print status at the end
		{

			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>Teller1 Stats:\r\nTotal Customers:%d | Max Transaction Time:%d mins | Avg Wait Time:%f secs | Max Wait:%d mins",
																	teller1_totalcust, teller1_maxtranstime/60, (teller1_waittime/(teller1_waittime + (teller1_totaltime/60)))*60, teller1_maxwait), 500);
																	// "| Avg Wait Time:%d mins | Max Wait Time:%d mins"	, teller1_lbreak, teller1_sbreak
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>Teller1 Break Stats:\r\nTotal Breaks:%d | Average Break:%d mins | Longest Break:%d mins | Shortest Break:%d mins",
														teller1_totalbreaks, teller1_avgbreak, teller1_lbreak, teller1_sbreak), 500);

			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>Teller2 Stats:\r\nTotal Customers:%d | Max Transaction Time:%d mins | Avg Wait Time:%f secs | Max Wait:%d mins",
																	teller2_totalcust, teller2_maxtranstime/60, (teller2_waittime/(teller2_waittime + (teller2_totaltime/60)))*60, teller2_maxwait), 500);
																	// "| Avg Wait Time:%d mins | Max Wait Time:%d mins"	, teller1_lbreak, teller1_sbreak
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>Teller2 Break Stats:\r\nTotal Breaks:%d | Average Break:%d mins | Longest Break:%d mins | Shortest Break:%d mins",
														teller2_totalbreaks, teller2_avgbreak, teller2_lbreak, teller2_sbreak), 500);

			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>Teller3 Stats:\r\nTotal Customers:%d | Max Transaction Time:%d mins | Avg Wait Time:%f secs | Max Wait:%d mins",
																	teller3_totalcust, teller3_maxtranstime/60, (teller3_waittime/(teller3_waittime + (teller3_totaltime/60)))*60, teller3_maxwait), 500);
																	// "| Avg Wait Time:%d mins | Max Wait Time:%d mins"	, teller1_lbreak, teller1_sbreak
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>Teller3 Break Stats:\r\nTotal Breaks:%d | Average Break:%d mins | Longest Break:%d mins | Shortest Break:%d mins",
														teller3_totalbreaks, teller3_avgbreak, teller3_lbreak, teller3_sbreak), 500);


			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\n>Customer Stats:\r\n Total Customers:%d | Max Depth:%d | Max Wait:%d mins | Avg Wait:%f secs | Avg Teller Time:%d mins",
					teller1_totalcust+teller2_totalcust+teller3_totalcust, max_depth, customer_max_wait, (customer_avg_wait/(teller1_totalcust+teller2_totalcust+teller3_totalcust))*60, (teller1_totaltime/60)/teller1_totalcust), 500);

			//cnt = hook();
			//cnt = vApplicationIdleHook();
			HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "\r\nIdle Hook:%d", cnt), 500);

			statflag = 1;
		}
		//if(statflag == 1)
		//{

		//}
		osDelay(100); // Executes every 100ms or 1 min sim time
		cnt = vApplicationIdleHook();
  }

  /* USER CODE END StartManager */
}

/* USER CODE BEGIN Header_StartMFS */
/**
* @brief Function implementing the mfstask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMFS */
void StartMFS(void *argument)
{
  /* USER CODE BEGIN StartMFS */

  /* Infinite loop */
  for(;;)
  {
		if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_RESET) { // If button is pressed
			set_led_1(1); // Light up LED 1 if teller 1 is on button break
			set_led_2(0);
			set_led_3(0);

			if(teller1_state == waiting || teller1_state == on_break) // Check if teller is not active
			{
				if(button1flag == 0)
				{
					time11 = __HAL_TIM_GET_COUNTER(&htim2); // Note down current time
					button1flag = 1; // Flag to know button was pressed once
				}

				teller1_openforbreak = 0; // Teller becomes ineligible for break
				teller1_state = on_break; // Teller is in break state
				teller1_button = 1; // Flag to indicate button held down
				 t1b2 = HAL_RNG_GetRandomNumber(&hrng); // Generate tellers next break from now
				 t1b2 = (t1b2 % 60) + 30;
			}

		}
		else if (HAL_GPIO_ReadPin(SHLD_A1_GPIO_Port, SHLD_A1_Pin)==GPIO_PIN_SET) { // If button is released
			set_led_1(0);
			set_led_2(0);
			set_led_3(0);
			if(teller1_button == 1 ) // Check if button was recently pressed and released
			{

				time12 = __HAL_TIM_GET_COUNTER(&htim2); // Get current time
				teller1_buttonbreak = (time12 - time11)/100; // Calculate teller break time
				teller1_breaktime += teller1_buttonbreak; // Add to total break time
				if(teller1_buttonbreak > teller1_lbreak) // Check for longest break
				{
					teller1_lbreak = teller1_buttonbreak;
				}
				if(teller1_buttonbreak < teller1_sbreak) // Check for shortest break
				{
					teller1_sbreak = teller1_buttonbreak;
				}
				button1flag = 0; // Put flag back in initial state
				teller1_button = 0; // Indicate teller button is no longer pressed
				teller1_totalbreaks++; // Increment teller breaks
				teller1_state = waiting; // Put teller in waiting state
			}

			if(teller1_button == 1 && teller1_state == finish)
			{

				time12 = __HAL_TIM_GET_COUNTER(&htim2);
				teller1_buttonbreak = (time12 - time11)/100;
				teller1_breaktime += teller1_buttonbreak;
				if(teller1_buttonbreak > teller1_lbreak)
				{
					teller1_lbreak = teller1_buttonbreak;
				}
				if(teller1_buttonbreak < teller1_sbreak)
				{
					teller1_sbreak = teller1_buttonbreak;
				}
				button1flag = 0;
				teller1_button = 0;
				teller1_totalbreaks++;
			}
		}


		if(HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_RESET){
			set_led_1(0);
			set_led_2(1);
			set_led_3(0);
			if(teller2_state == waiting || teller2_state == on_break)
			{

				if(button2flag == 0)
				{
					time21 = __HAL_TIM_GET_COUNTER(&htim2);
					button2flag = 1;
				}

				teller2_openforbreak = 0;
				teller2_state = on_break;
				teller2_button = 1;
				 t2b2 = HAL_RNG_GetRandomNumber(&hrng);
				 t2b2 = (t2b2 % 60) + 30;
			}
		}

		else if (HAL_GPIO_ReadPin(SHLD_A2_GPIO_Port, SHLD_A2_Pin)==GPIO_PIN_SET)
		{
			set_led_1(0);
			set_led_2(0);
			set_led_3(0);
			if(teller2_button == 1 )
			{

				time22 = __HAL_TIM_GET_COUNTER(&htim2);
				teller2_buttonbreak = (time22 - time21)/100;
				teller2_breaktime += teller2_buttonbreak;
				if(teller2_buttonbreak > teller2_lbreak)
				{
					teller2_lbreak = teller2_buttonbreak;
				}
				if(teller2_buttonbreak < teller2_sbreak)
				{
					teller2_sbreak = teller2_buttonbreak;
				}
				button2flag = 0;
				teller2_button = 0;
				teller2_totalbreaks++;
				teller2_state = waiting;
			}

			if(teller2_button == 1 && teller2_state == finish)
			{

				time22 = __HAL_TIM_GET_COUNTER(&htim2);
				teller2_buttonbreak = (time22 - time21)/100;
				teller2_breaktime += teller2_buttonbreak;
				if(teller2_buttonbreak > teller2_lbreak)
				{
					teller2_lbreak = teller2_buttonbreak;
				}
				if(teller2_buttonbreak < teller2_sbreak)
				{
					teller2_sbreak = teller2_buttonbreak;
				}
				button2flag = 0;
				teller2_button = 0;
				teller2_totalbreaks++;
			}
		}

		if(HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_RESET){
			set_led_1(0);
			set_led_2(0);
			set_led_3(1);
			if(teller3_state == waiting || teller3_state == on_break)
			{

				if(button3flag == 0)
				{
					time31 = __HAL_TIM_GET_COUNTER(&htim2);
					button3flag = 1;
				}

				teller3_openforbreak = 0;
				teller3_state = on_break;
				teller3_button = 1;
				 t3b2 = HAL_RNG_GetRandomNumber(&hrng);
				 t3b2 = (t3b2 % 60) + 30;
			}
		}
		else if(HAL_GPIO_ReadPin(SHLD_A3_GPIO_Port, SHLD_A3_Pin)==GPIO_PIN_SET){
			set_led_1(0);
			set_led_2(0);
			set_led_3(0);
			if(teller3_button == 1 )
			{

				time32 = __HAL_TIM_GET_COUNTER(&htim2);
				teller3_buttonbreak = (time32 - time31)/100;
				teller3_breaktime += teller3_buttonbreak;
				if(teller3_buttonbreak > teller3_lbreak)
				{
					teller3_lbreak = teller3_buttonbreak;
				}
				if(teller3_buttonbreak < teller3_sbreak)
				{
					teller3_sbreak = teller3_buttonbreak;
				}
				button3flag = 0;
				teller3_button = 0;
				teller3_totalbreaks++;
				teller3_state = waiting;
			}

			if(teller3_button == 1 && teller3_state == finish)
			{

				time32 = __HAL_TIM_GET_COUNTER(&htim2);
				teller3_buttonbreak = (time32 - time31)/100;
				teller3_breaktime += teller3_buttonbreak;
				if(teller3_buttonbreak > teller3_lbreak)
				{
					teller3_lbreak = teller3_buttonbreak;
				}
				if(teller3_buttonbreak < teller3_sbreak)
				{
					teller3_sbreak = teller3_buttonbreak;
				}
				button3flag = 0;
				teller3_button = 0;
				teller3_totalbreaks++;
			}
		}
		cnt = vApplicationIdleHook();
  }
  /* USER CODE END StartMFS */
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
  if (htim->Instance == TIM6) {
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

#ifdef  USE_FULL_ASSERT
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

