/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "mpu6050_lib.h" //i2c interface with mpu6050
#include "drone_PID.h" //drone PID library made by me
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
												.name = "defaultTask",
												.stack_size = 128 * 4,
												.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PIDTask */
osThreadId_t PIDTaskHandle;
const osThreadAttr_t PIDTask_attributes = {
											.name = "PIDTask",
											.stack_size = 128 * 4,
											.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for orientationTask */
osThreadId_t orientationTaskHandle;
const osThreadAttr_t orientationTask_attributes = {
													.name = "orientationTask",
													.stack_size = 128 * 4,
													.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for inputsTask */
osThreadId_t inputsTaskHandle;
const osThreadAttr_t inputsTask_attributes = {
												.name = "inputsTask",
												.stack_size = 128 * 4,
												.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for xMutex */
osMutexId_t xMutexHandle;
const osMutexAttr_t xMutex_attributes = {
											.name = "xMutex"
};
/* USER CODE BEGIN PV */
mpu6050_sensor_data sensor_data_1;

uint16_t packetSize;
uint16_t fifoCount;
int a;
uint8_t fifoBuffer[64];

Quaternion q; //store quaternion data from MPU6050 sensor
VectorFloat gravity; //used to calculate ypr
float32_t ypr[3]; //yaw pitch roll data from sensor

//Variables for measuring PWM duty cycle from the channels of the HobbyKing controller's receiver.
uint16_t throttleCaptureIndex = 0; //TIM1 Channels 1 and 2
uint16_t yawCaptureIndex = 0; //TIM1 Channels 3 and 4
uint16_t pitchCaptureIndex = 0; //TIM2 Channels 1 and 2
uint16_t rollCaptureIndex = 0; //TIM2 Channels 3 and 4

uint32_t throttleInputCaptureValue1 = 0;
uint32_t throttleInputCaptureValue2 = 0;
uint32_t throttleDiffCapture = 0;

uint32_t yawInputCaptureValue1 = 0;
uint32_t yawInputCaptureValue2 = 0;
uint32_t yawDiffCapture = 0;

uint32_t pitchInputCaptureValue1 = 0;
uint32_t pitchInputCaptureValue2 = 0;
uint32_t pitchDiffCapture = 0;

uint32_t rollInputCaptureValue1 = 0;
uint32_t rollInputCaptureValue2 = 0;
uint32_t rollDiffCapture = 0;

//controller variables
drone_motor_controller drone_controller;
pid_controller pitch_PID_controller;
pid_controller roll_PID_controller;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void updatePID(void *argument);
void getOrientation(void *argument);
void getInputs(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_I2C2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	//timer3 used for measuring duty cycle of incoming PWM signal of throttle, yaw, pitch and roll from receiver.
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK) //CH1 Throttle
	{
		Error_Handler();  // Error starting input capture for channel 1
	}
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK) //CH2 Yaw
	{
		Error_Handler();  // Error starting input capture for channel 2
	}
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3) != HAL_OK) //CH3 Pitch
	{
		Error_Handler();  // Error starting input capture for channel 3
	}
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4) != HAL_OK) //CH4 Roll
	{
		Error_Handler();  // Error starting input capture for channel 4
	}

	//timer2 used to generate pwm signals to send to motor drivers
	/* Start channel 1 */
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
	{
	Error_Handler();
	}
	/* Start channel 2 */
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
	{
	Error_Handler();
	}
	/* Start channel 3 */
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
	{
	Error_Handler();
	}
	/* Start channel 4 */
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
	{
	Error_Handler();
	}

	//change duty cycle
//	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,60);
//	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,800);
	//HAL i2c notes:
	//address of MPU6050 device is 1101000, but we shift it to left because the transmit and receive functions require that. So we are left with 0xD0
	//Argument to right of MPU6050_ADDR_LSL1 is the register address, see the register description in onenote.
	uint8_t reg_addr[1];
	/* We compute the MSB and LSB parts of the memory address */
	reg_addr[0] = (uint8_t) (0x6A);

	//delay for init functions to see if it stops glitch of i2c transmission never completing
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	//test if transmission works
	HAL_StatusTypeDef returnValue = HAL_I2C_Master_Transmit_DMA(&hi2c2, MPU6050_ADDR_LSL1, reg_addr, 1);
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	if (returnValue != HAL_OK)
	{
		Error_Handler();
	}
	if (__HAL_DMA_GET_FLAG(&hdma_i2c1_tx, (0x00000002U)))
	{ // Transfer error
		printf("DMA Transfer Error\n");
		// Handle error here
	}
	HAL_DMA_IRQHandler(&hdma_i2c2_tx);
	while (!i2c_TX_done);
	i2c_TX_done = 0;

	//setup storage for data
	// mpu6050_init(&hi2c2); //write to registers in mpu6050 to configure initial settings
	// moved to private variables to see values of attributes in structs in debug mode easier
	//	mpu6050_sensor_data sensor_data_1;
	//	kalman_filter filter1;

	mpu6050_init_dmp(&hi2c2); //initialize mpu6050 to use dmp

	setDMPEnabled(&hi2c2, true); //enable the dmp

	packetSize = 42; //FIXME, use this: packetSize = mpu.dmpGetFIFOPacketSize();

	//define starting position
	sensor_data_init(&sensor_data_1); //likely not necessary

	fifoCount = getFIFOCount(&hi2c2);
	fifoCount = getFIFOCount(&hi2c2);
	resetFIFO(&hi2c2);
	fifoCount = getFIFOCount(&hi2c2);
	fifoCount = getFIFOCount(&hi2c2);

	resetFIFO(&hi2c2);

	//setup PID controllers for pitch and roll
	initialize_PID(&pitch_PID_controller, 0);
	set_gains_PID(&pitch_PID_controller, 700, 0, 0);

	initialize_PID(&roll_PID_controller, 0);
	set_gains_PID(&roll_PID_controller, 700, 0, 0);

	//setup drone motor controller
	initialize_drone_motor_controller(&drone_controller, &pitch_PID_controller, &roll_PID_controller);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of xMutex */
	xMutexHandle = osMutexNew(&xMutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of PIDTask */
	PIDTaskHandle = osThreadNew(updatePID, NULL, &PIDTask_attributes);

	/* creation of orientationTask */
	orientationTaskHandle = osThreadNew(getOrientation, NULL, &orientationTask_attributes);

	/* creation of inputsTask */
	inputsTaskHandle = osThreadNew(getInputs, NULL, &inputsTask_attributes);

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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 3;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 999;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//i2c callback functions, remember i2c interface is in mpu6050_lib.c
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_TX_done = 1; //defined in mpu6050_lib.c
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	i2c_RX_done = 1; //defined in mpu6050_lib.c
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_12)
	{
		orientation_data_ready = 1; //defined in mpu6050_lib.c, get DMP data as soon as its ready from mpu6050
	}

}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //PWM duty cycle calculations, called when timer detects rising and falling edges
{ //most code based off of f303k8 TIM_InputCapture example from 1.11.15 firmware package, some is based off of chapter 11.3.5 of mastering stm32 book.
	if (htim->Instance == TIM3)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (throttleCaptureIndex == 0)
			{
				throttleInputCaptureValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				throttleCaptureIndex = 1;

				//change polarity so that next time the falling edge is detected
				htim->Instance->CCER &= ~TIM_CCER_CC1E;     // Disable channel
				htim->Instance->CCER |= TIM_CCER_CC1P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC1E;      // Re-enable channel
			}
			else if (throttleCaptureIndex == 1)
			{
				throttleInputCaptureValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				if (throttleInputCaptureValue2 > throttleInputCaptureValue1)
				{
					throttleDiffCapture = throttleInputCaptureValue2 - throttleInputCaptureValue1;
				}
				else if (throttleInputCaptureValue2 < throttleInputCaptureValue1)
				{
					throttleDiffCapture = (htim->Instance->ARR - throttleInputCaptureValue1) + throttleInputCaptureValue2;
				}
				else
				{
					/* If capture values are equal, we have reached the limit of frequency
					 measures */
					Error_Handler();
				}
				//				uint32_t uwFrequency = HAL_RCC_GetPCLK2Freq();
				throttleCaptureIndex = 0;

				//change polarity so that next time the rising  edge is detected
				htim->Instance->CCER &= ~TIM_CCER_CC1E;     // Disable channel
				htim->Instance->CCER &= ~TIM_CCER_CC1P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC1E;      // Re-enable channel
			}

		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (yawCaptureIndex == 0)
			{
				yawInputCaptureValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				yawCaptureIndex = 1;

				htim->Instance->CCER &= ~TIM_CCER_CC2E;     // Disable channel
				htim->Instance->CCER |= TIM_CCER_CC2P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC2E;      // Re-enable channel
			}
			else if (yawCaptureIndex == 1)
			{
				yawInputCaptureValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				if (yawInputCaptureValue2 > yawInputCaptureValue1)
				{
					yawDiffCapture = yawInputCaptureValue2 - yawInputCaptureValue1;
				}
				else if (yawInputCaptureValue2 < yawInputCaptureValue1)
				{
					yawDiffCapture = (htim->Instance->ARR - yawInputCaptureValue1) + yawInputCaptureValue2;
				}
				else
				{
					/* If capture values are equal, we have reached the limit of frequency
					 measures */
					Error_Handler();
				}
				//				uint32_t uwFrequency = HAL_RCC_GetPCLK2Freq();
				yawCaptureIndex = 0;

				htim->Instance->CCER &= ~TIM_CCER_CC2E;     // Disable channel
				htim->Instance->CCER &= ~TIM_CCER_CC2P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC2E;      // Re-enable channel
			}

		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if (pitchCaptureIndex == 0)
			{
				pitchInputCaptureValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				pitchCaptureIndex = 1;

				htim->Instance->CCER &= ~TIM_CCER_CC3E;     // Disable channel
				htim->Instance->CCER |= TIM_CCER_CC3P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC3E;      // Re-enable channel
			}
			else if (pitchCaptureIndex == 1)
			{
				pitchInputCaptureValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				if (pitchInputCaptureValue2 > pitchInputCaptureValue1)
				{
					pitchDiffCapture = pitchInputCaptureValue2 - pitchInputCaptureValue1;
				}
				else if (pitchInputCaptureValue2 < pitchInputCaptureValue1)
				{
					pitchDiffCapture = (htim->Instance->ARR - pitchInputCaptureValue1) + pitchInputCaptureValue2;
				}
				else
				{
					/* If capture values are equal, we have reached the limit of frequency
					 measures */
					Error_Handler();
				}
				//				uint32_t uwFrequency = HAL_RCC_GetPCLK2Freq();
				pitchCaptureIndex = 0;

				htim->Instance->CCER &= ~TIM_CCER_CC3E;     // Disable channel
				htim->Instance->CCER &= ~TIM_CCER_CC3P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC3E;      // Re-enable channel
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if (rollCaptureIndex == 0)
			{
				rollInputCaptureValue1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				rollCaptureIndex = 1;

				htim->Instance->CCER &= ~TIM_CCER_CC4E;     // Disable channel
				htim->Instance->CCER |= TIM_CCER_CC4P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC4E;      // Re-enable channel
			}
			else if (rollCaptureIndex == 1)
			{
				rollInputCaptureValue2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				if (rollInputCaptureValue2 > rollInputCaptureValue1)
				{
					rollDiffCapture = rollInputCaptureValue2 - rollInputCaptureValue1;
				}
				else if (rollInputCaptureValue2 < rollInputCaptureValue1)
				{
					rollDiffCapture = (htim->Instance->ARR - rollInputCaptureValue1) + rollInputCaptureValue2;
				}
				else
				{
					/* If capture values are equal, we have reached the limit of frequency
					 measures */
					Error_Handler();
				}
				//				uint32_t uwFrequency = HAL_RCC_GetPCLK2Freq();
				rollCaptureIndex = 0;

				htim->Instance->CCER &= ~TIM_CCER_CC4E;     // Disable channel
				htim->Instance->CCER &= ~TIM_CCER_CC4P;      // Toggle polarity
				htim->Instance->CCER |= TIM_CCER_CC4E;      // Re-enable channel
			}
		}
	}

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_updatePID */
/**
 * @brief Function implementing the PIDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_updatePID */
void updatePID(void *argument)
{
	/* USER CODE BEGIN updatePID */
	/* Infinite loop */
	for (;;)
	{
		// The DiffCapture variables have values between 65 to 125, which represents 1ms to 2ms pulse time in ticks (ticks occur at a frequency of 64,000 Hz).
		// We want to map the pulse time of the PWM signal to desired tilt angles in radians.
		// The controllers are imperfect, so we set thresholds for DiffCapture values to be set at their max or min. Anything over 120 is considered max and anything below 80 is considered min.
		// The middle is 100 ticks of pulse width as a result, we subtract 100 to make the input 0 when joystick is in middle.
		// The max is now 20 and min is now -20. Now, we can scale appropriately to get our desired control input by dividing or multiplying.
		float32_t pitch_setpoint = ((int32_t) pitchDiffCapture-100);
		if(pitch_setpoint > 20)
		{
			pitch_setpoint = 20;
		} else if(pitch_setpoint < -20)
		{
			pitch_setpoint = -20;
		}
		pitch_setpoint = (pitch_setpoint/4)*3.14/180; //For pitch and roll, we scale appropriately so the max tilt angle setpoint back and forth is around 5 degrees, but converted to radians

		float32_t roll_setpoint = ((int32_t) rollDiffCapture-100);
		if(roll_setpoint > 20)
		{
			roll_setpoint = 20;
		} else if(roll_setpoint < -20)
		{
			roll_setpoint = -20;
		}
		roll_setpoint = (roll_setpoint/4)*3.14/180;

		float32_t yaw_signal = ((int32_t) yawDiffCapture-100)*5;
		if(yaw_signal > 100)
		{
			yaw_signal = 100;
		}
		if(yaw_signal < -100)
		{
			yaw_signal = -100;
		}

		float32_t throttle_signal = (throttleDiffCapture - 60)*17;
		if(throttle_signal > 950)
		{
			throttle_signal = 950;
		}
		if(throttle_signal < 0)
		{
			throttle_signal = 0;
		}
		update_PID(&pitch_PID_controller, ypr[1], pitch_setpoint);
		update_PID(&roll_PID_controller, ypr[2], roll_setpoint);
		update_signals(&drone_controller, throttle_signal, yaw_signal);
		update_motor_input(&drone_controller, &htim2);
		osDelay(1);
	}
	/* USER CODE END updatePID */
}

/* USER CODE BEGIN Header_getOrientation */
/**
 * @brief Function implementing the orientationTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_getOrientation */
void getOrientation(void *argument)
{
	/* USER CODE BEGIN getOrientation */
	/* Infinite loop */
	for (;;)
	{
		//credits: https://github.com/Pluscrafter/i2cdevlib/blob/master/STM32_HAL/Nucleo-144F722ZE/Src/main.cpp

		while (!orientation_data_ready); //wait until external interrupt fires to get data when it is freshly ready
		if (osMutexAcquire(xMutexHandle, osWaitForever) == osOK) //try to aquire mutex
		{
			fifoCount = getFIFOCount(&hi2c2);
			while (fifoCount < packetSize)
			{
				//insert here your code
				fifoCount = getFIFOCount(&hi2c2);
			}
			if (fifoCount >= 1024)
			{
				resetFIFO(&hi2c2);
				//Serial.println(F("FIFO overflow!"));
			}
			else
			{
				if (fifoCount % packetSize != 0)
				{
					resetFIFO(&hi2c2);
					fifoCount = getFIFOCount(&hi2c2);
					//getFIFOBytes(&hi2c2, fifoBuffer, packetSize); //remove later
				}
				else
				{
					while (fifoCount >= packetSize)
					{

						getFIFOBytes(&hi2c2, fifoBuffer, packetSize);
						fifoCount -= packetSize;

					}
					dmpGetQuaternionQuatStruct(&q, fifoBuffer);
					dmpGetGravity(&gravity, &q);
					dmpGetYawPitchRoll(ypr, &q, &gravity);
				}
			}
			a++;

		}
		osMutexRelease(xMutexHandle);
		HAL_Delay(20);
		orientation_data_ready = 0;
	}
	/* USER CODE END getOrientation */
}

/* USER CODE BEGIN Header_getInputs */
/**
 * @brief Function implementing the inputsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_getInputs */
void getInputs(void *argument)
{
	/* USER CODE BEGIN getInputs */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END getInputs */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4)
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
