/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"

#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "stm32l4s5i_iot01_qspi.h"



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

OSPI_HandleTypeDef hospi1;

UART_HandleTypeDef huart1;

osThreadId readSensors_TasHandle;
osThreadId UART_TaskHandle;
osThreadId buttonPress_TasHandle;
/* USER CODE BEGIN PV */

char buffer[200];

int16_t gyro[3];

int16_t magneto[3];

int16_t temp[1];

int16_t pressure[1];

int counter = -1;

int deleting = 0;

int hasPassed = 0;

int gettingAverages = 0;

int pleasePrint = 0;


int32_t samplesGyro= 0, samplesMagneto= 0, samplesTemp= 0, samplesPressure = 0;

int32_t xGyroAverage= 0, yGyroAverage= 0, zGyroAverage = 0;

int32_t xMagnetoAverage= 0, yMagnetoAverage= 0, zMagnetoAverage = 0;

int32_t xGyroVariance = 0, yGyroVariance = 0, zGyroVariance = 0;

int32_t xMagnetoVariance = 0, yMagnetoVariance = 0, zMagnetoVariance = 0;

int32_t pressureAverage = 0, temperatureAverage = 0;

int32_t pressureVariance = 0, temperatureVariance = 0;


//Addresses are: 0x00000000, 0x00010000, 0x00020000, 0x00030000, 0x00040000, 0x00050000, 0x00060000, 0x00070000


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
void readFromSensors(void const * argument);
void transmitViaUART(void const * argument);
void hasButtonBeenPressed(void const * argument);

/* USER CODE BEGIN PFP */

void Error_Handler(void);
void deleteBlocks(void);

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */

//Init all the sensors

  BSP_GYRO_Init();
  BSP_MAGNETO_Init();
  BSP_PSENSOR_Init();
  BSP_TSENSOR_Init();


  //Init the qspi
  if(BSP_QSPI_Init() != QSPI_OK) {
	  Error_Handler();
  }

  //Turn red led off
  HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_SET);


  //Delete the memory blocks
  deleteBlocks();


  //Say we are ready
  memset(buffer, 0, 200);
  sprintf(&buffer, "Ready to go \n");
  HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);



  /* USER CODE END 2 */

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
  /* definition and creation of readSensors_Tas */
  osThreadDef(readSensors_Tas, readFromSensors, osPriorityNormal, 0, 128);
  readSensors_TasHandle = osThreadCreate(osThread(readSensors_Tas), NULL);

  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, transmitViaUART, osPriorityNormal, 0, 256);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* definition and creation of buttonPress_Tas */
  osThreadDef(buttonPress_Tas, hasButtonBeenPressed, osPriorityNormal, 0, 128);
  buttonPress_TasHandle = osThreadCreate(osThread(buttonPress_Tas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Red_Led_Pin */
  GPIO_InitStruct.Pin = Red_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Red_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_Led_Pin */
  GPIO_InitStruct.Pin = Green_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_Led_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void getPressureAverage(){

		int16_t pressureArray[samplesPressure]; // Create an array to store the values

		memset(pressureArray, 0, samplesPressure);

		int16_t test[1] = {0};


		// all the values in pressureArray, can calculate the average
		pressureAverage = 0;
		for (int i = 0; i < samplesPressure; i++) {

			if(BSP_QSPI_Read(test, 0x00070000 + sizeof(pressure) * i, sizeof(pressure)) != QSPI_OK)
			{
				Error_Handler();
			}

			pressureArray[i] = test[0];

			pressureAverage += test[0];
		}

		pressureAverage /= samplesPressure;


		//S² = Σ(xi - x̄)² / (n - 1)
		double sumSquaredDeviations = 0.0;

		for (int i = 0; i < samplesPressure; i++) {
			  double deviation = pressureArray[i] - pressureAverage;
			  sumSquaredDeviations += deviation * deviation;
		}

		double sampleVariance = sumSquaredDeviations / (samplesPressure - 1);

		pressureVariance = (int16_t) sampleVariance;


}

void getTemperatureAverage(){

		int16_t temperatureArray[samplesTemp]; // Create an array to store the values
		memset(temperatureArray, 0, samplesTemp);


		int16_t test[1] = {0};


		// all the values in pressureArray, can calculate the average
		temperatureAverage = 0;
		for (int i = 0; i < samplesTemp; i++) {

			if(BSP_QSPI_Read(test, 0x00060000 + sizeof(temp) * i, sizeof(temp)) != QSPI_OK)
			{
				Error_Handler();
			}

			temperatureArray[i] = test[0];

			temperatureAverage += temperatureArray[i];
		}


		temperatureAverage /= samplesTemp;

		//S² = Σ(xi - x̄)² / (n - 1)
		double sumSquaredDeviations = 0.0;

		for (int i = 0; i < samplesTemp; i++) {
		  double deviation = temperatureArray[i] - pressureAverage;
		  sumSquaredDeviations += deviation * deviation;
		}

		double sampleVariance = sumSquaredDeviations / (samplesTemp - 1);

		temperatureVariance = (int16_t) sampleVariance;



}


void getGyroAverage(){

    //populate an array with the max number of sample gotten
    int16_t gyroArray[samplesGyro * 3];
	memset(gyroArray, 0, samplesGyro * 3);

    int16_t test[3];


    xGyroAverage = 0;
    yGyroAverage = 0;
    zGyroAverage = 0;

    for(int i = 0; i < samplesGyro; i++){


    	test[0] = 0;
    	test[1] = 0;
    	test[2] = 0;

        if(BSP_QSPI_Read(test, 0x00000000 + i * sizeof(gyro), sizeof(gyro)) != QSPI_OK)
    	{
        	Error_Handler();
    	}


    	gyroArray[3*i] = test[0];
    	gyroArray[3*i + 1] = test[1];
    	gyroArray[3*i + 2] = test[2];

    	xGyroAverage += test[0]; 	//get x values

    	yGyroAverage += test[1];	//get y values

    	zGyroAverage += test[2];	//get z values

    }

    xGyroAverage /= samplesGyro;
    yGyroAverage /= samplesGyro;
    zGyroAverage /= samplesGyro;


	//S² = Σ(xi - x̄)² / (n - 1)
	double XsumSquaredDeviations, YsumSquaredDeviations, ZsumSquaredDeviations = 0.0;

	for (int i = 0; i < samplesGyro; i++) {
		  double Xdeviation = gyroArray[3*i] - xGyroAverage;
		  XsumSquaredDeviations += Xdeviation * Xdeviation;

		  double Ydeviation = gyroArray[3*i + 1] - yGyroAverage;
		  YsumSquaredDeviations += Ydeviation * Ydeviation;

		  double Zdeviation = gyroArray[3*i + 2] - zGyroAverage;
		  ZsumSquaredDeviations += Zdeviation * Zdeviation;
	}

	double XsampleVariance = XsumSquaredDeviations / (samplesGyro - 1);

	double YsampleVariance = YsumSquaredDeviations / (samplesGyro - 1);

	double ZsampleVariance = ZsumSquaredDeviations / (samplesGyro - 1);

	xGyroVariance = (int32_t) XsampleVariance;
	yGyroVariance = (int32_t) YsampleVariance;
	zGyroVariance = (int32_t) ZsampleVariance;





}

void getMagnetoAverage(){

    //populate an array with the max number of sample gotten
    int16_t magnetoArray[samplesMagneto * 3];
	memset(magnetoArray, 0, samplesMagneto * 3);


    int16_t test[3];


    xMagnetoAverage = 0;
    yMagnetoAverage = 0;
    zMagnetoAverage = 0;

    for(int i = 0; i < samplesMagneto; i++){


    	test[0] = 0;
    	test[1] = 0;
    	test[2] = 0;

        if(BSP_QSPI_Read(test, 0x00030000 + i * sizeof(magneto), sizeof(magneto)) != QSPI_OK)
    	{
        	Error_Handler();
    	}


        magnetoArray[3*i] = test[0];
        magnetoArray[3*i + 1] = test[1];
        magnetoArray[3*i + 2] = test[2];

        xMagnetoAverage += test[0]; 	//get x values

    	yMagnetoAverage += test[1];	//get y values

    	zMagnetoAverage += test[2];	//get z values

    }

    xMagnetoAverage /= samplesGyro;
    yMagnetoAverage /= samplesGyro;
    zMagnetoAverage /= samplesGyro;


	//S² = Σ(xi - x̄)² / (n - 1)
	double XsumSquaredDeviations, YsumSquaredDeviations, ZsumSquaredDeviations = 0.0;

	for (int i = 0; i < samplesGyro; i++) {
		  double Xdeviation = magnetoArray[3*i] - xMagnetoAverage;
		  XsumSquaredDeviations += Xdeviation * Xdeviation;

		  double Ydeviation = magnetoArray[3*i + 1] - xMagnetoAverage;
		  YsumSquaredDeviations += Ydeviation * Ydeviation;

		  double Zdeviation = magnetoArray[3*i + 2] - xMagnetoAverage;
		  ZsumSquaredDeviations += Zdeviation * Zdeviation;
	}

	double XsampleVariance = XsumSquaredDeviations / (samplesMagneto - 1);

	double YsampleVariance = YsumSquaredDeviations / (samplesMagneto - 1);

	double ZsampleVariance = ZsumSquaredDeviations / (samplesMagneto - 1);

	xMagnetoVariance = (int32_t) XsampleVariance;
	yMagnetoVariance = (int32_t) YsampleVariance;
	zMagnetoVariance = (int32_t) ZsampleVariance;


}

void printAllAverages(){


//	//Pressure
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "\nPressure: Number of samples is: %d, ", (int)samplesPressure);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Average: %d, ", (int)pressureAverage);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Variance: %d \n", (int)pressureVariance);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	//Temperature
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "\nTemperature: Number of samples is: %d, ", (int)samplesTemp);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Average: %d, ",(int) temperatureAverage);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Variance: %d \n",(int) temperatureVariance);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	//Gyro
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "\nGYRO: Number of samples is: %d, ",(int) samplesGyro);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Average: x: %d, y: %d, z: %d, ",(int) xGyroAverage,(int) yGyroAverage,(int) zGyroAverage);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Variance: x: %d, y: %d, z: %d \n",(int) xGyroVariance, (int)yGyroVariance,(int) zGyroVariance);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	//Magneto
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "\nMAGNETO: Number of samples is: %d, ",(int) samplesMagneto);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Average: x: %d, y: %d, z: %d, ", (int)xMagnetoAverage, (int)yMagnetoAverage,(int) zMagnetoAverage);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);
//
//	memset(buffer, 0, 100);
//	sprintf(&buffer, "Variance: x: %d, y: %d, z: %d \n \n",(int) xMagnetoVariance,(int) yMagnetoVariance,(int) zMagnetoVariance);
//	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);


	memset(buffer, 0, 200);
	// Pressure
	strcpy(buffer, "\nPressure: Number of samples is: ");
	char samplesPressureStr[20]; // Assuming a reasonable buffer size for the number
	sprintf(samplesPressureStr, "%d, ", (int)samplesPressure);
	strcat(buffer, samplesPressureStr);
	sprintf(samplesPressureStr, "Average: %d, ", (int)pressureAverage);
	strcat(buffer, samplesPressureStr);
	sprintf(samplesPressureStr, "Variance: %d \n", (int)pressureVariance);
	strcat(buffer, samplesPressureStr);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 5000);

	memset(buffer, 0, 200);
	// Temperature
	strcpy(buffer, "\nTemperature: Number of samples is: ");
	char samplesTempStr[20];
	sprintf(samplesTempStr, "%d, ", (int)samplesTemp);
	strcat(buffer, samplesTempStr);
	sprintf(samplesTempStr, "Average: %d, ", (int)temperatureAverage);
	strcat(buffer, samplesTempStr);
	sprintf(samplesTempStr, "Variance: %d \n", (int)temperatureVariance);
	strcat(buffer, samplesTempStr);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 5000);

	memset(buffer, 0, 200);
	// Gyro
	strcpy(buffer, "\nGYRO: Number of samples is: ");
	char samplesGyroStr[20];
	sprintf(samplesGyroStr, "%d, ", (int)samplesGyro);
	strcat(buffer, samplesGyroStr);
	sprintf(samplesGyroStr, "Average: x: %d, y: %d, z: %d, ", (int)xGyroAverage, (int)yGyroAverage, (int)zGyroAverage);
	strcat(buffer, samplesGyroStr);
	sprintf(samplesGyroStr, "Variance: x: %d, y: %d, z: %d \n", (int)xGyroVariance, (int)yGyroVariance, (int)zGyroVariance);
	strcat(buffer, samplesGyroStr);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 5000);

	memset(buffer, 0, 200);
	// Magneto
	strcpy(buffer, "\nMAGNETO: Number of samples is: ");
	char samplesMagnetoStr[20];
	sprintf(samplesMagnetoStr, "%d, ", (int)samplesMagneto);
	strcat(buffer, samplesMagnetoStr);
	sprintf(samplesMagnetoStr, "Average: x: %d, y: %d, z: %d, ", (int)xMagnetoAverage, (int)yMagnetoAverage, (int)zMagnetoAverage);
	strcat(buffer, samplesMagnetoStr);
	sprintf(samplesMagnetoStr, "Variance: x: %d, y: %d, z: %d \n \n", (int)xMagnetoVariance, (int)yMagnetoVariance, (int)zMagnetoVariance);
	strcat(buffer, samplesMagnetoStr);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 5000);


//Delete the memory blocks
//	deleteBlocks();
//	samplesGyro = 0;
//	samplesMagneto = 0;
//	samplesTemp = 0;
//	samplesPressure = 0;

	 xGyroVariance = 0, yGyroVariance = 0, zGyroVariance = 0;

	 xMagnetoVariance = 0, yMagnetoVariance = 0, zMagnetoVariance = 0;


}

//Function to delete blocks at 8 different addresses
void deleteBlocks(){

		deleting = 1;
		memset(buffer, 0, 200);
		sprintf(&buffer, "Deleting blocks...\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

	//Get 8 different addresses for 8 different values to be stored

	  	if(BSP_QSPI_Erase_Block(0x00000000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	if(BSP_QSPI_Erase_Block(0x00010000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	if(BSP_QSPI_Erase_Block(0x00020000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	if(BSP_QSPI_Erase_Block(0x00030000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	if(BSP_QSPI_Erase_Block(0x00040000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	if(BSP_QSPI_Erase_Block(0x00050000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	if(BSP_QSPI_Erase_Block(0x00060000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	if(BSP_QSPI_Erase_Block(0x00070000) != QSPI_OK){
	  		Error_Handler();
	  	}

	  	memset(buffer, 0, 200);
		sprintf(&buffer, "Blocks deleted ! \n");
	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

	  	deleting = 0;

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_readFromSensors */
/**
  * @brief  Function implementing the readSensors_Tas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_readFromSensors */
void readFromSensors(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(100);

	  	if(!deleting && !gettingAverages){


	  		if(counter == 0){
	  			BSP_GYRO_GetXYZ(gyro); // (x, y, z)

				//Store gyro x,y,z values
				//Use addresses from 0x00000000 to 0x00030000
				if(BSP_QSPI_Write(gyro, 0x00000000 + sizeof(gyro) * samplesGyro , sizeof(gyro)) != QSPI_OK){
					Error_Handler();
				}
				else{
					samplesGyro++;
				}
			}

	  		else if(counter == 1){
				BSP_MAGNETO_GetXYZ(magneto); // (x, y, z)

				//Store magneto x,y,z values
				//Use addresses from 0x00030000 to 0x00060000
				if(counter == 1 && BSP_QSPI_Write(magneto, 0x00030000 + sizeof(magneto) * samplesMagneto, sizeof(magneto)) != QSPI_OK){
					Error_Handler();
				}
				else{
					samplesMagneto++;
				}
	  		}

	  		else if(counter == 2){

				temp[0] = BSP_TSENSOR_ReadTemp();

				//Store temperature values
				if(counter == 2 && BSP_QSPI_Write(temp, 0x00060000 + sizeof(temp) * samplesTemp, sizeof(temp)) != QSPI_OK){
					Error_Handler();
				}else{
					samplesTemp++;

				}
	  		}

	  		else if(counter == 3){
			   pressure[0] = BSP_PSENSOR_ReadPressure();

				//Store pressure values
				if(counter == 3 && BSP_QSPI_Write(pressure, 0x00070000 + sizeof(pressure) * samplesPressure, sizeof(pressure)) != QSPI_OK){
					Error_Handler();
				}else{
					samplesPressure++;

				}

	  		}
	 }


  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_transmitViaUART */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmitViaUART */
void transmitViaUART(void const * argument)
{
  /* USER CODE BEGIN transmitViaUART */
  /* Infinite loop */
  for(;;)
  {
      osDelay(200);



	  	if(counter == 0){

	  		hasPassed = 0;

	  		int x = gyro[0];
	  		int y = gyro[1];
	  		int z = gyro[2];

	  		memset(buffer, 0, 200);
	  		sprintf(&buffer, "GYRO: x: %d, y: %d, z: %d \n", x,y,z);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

	  	}

	  	else if(counter == 1){

	  		int x = magneto[0];
	  		int y = magneto[1];
	  		int z = magneto[2];

	  		memset(buffer, 0, 200);
	  		sprintf(&buffer, "Magneto: x: %d, y: %d, z: %d \n", x,y,z);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

	  	}

	  	else if(counter == 2){

	  		memset(buffer, 0, 200);
	  		sprintf(&buffer, "Temperature: %d \n", (int) temp[0]);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

	  	}

	  	else if(counter == 3){

	  		memset(buffer, 0, 200);
	  		sprintf(&buffer, "Pressure: %d \n", (int) pressure[0]);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

	  	}

	  	else if(!hasPassed && counter == 4 ){

	  		osDelay(100);

			memset(buffer, 0, 200);
			sprintf(&buffer, "Getting averages \n");
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

			gettingAverages = 1;
			//osDelay(10);
			getGyroAverage();
			//osDelay(10);
			getMagnetoAverage();
			//osDelay(10);
			getTemperatureAverage();
			//osDelay(10);
			getPressureAverage();

			//osDelay(10);
			printAllAverages();
			//osDelay(10);

	  	  	hasPassed = 1;

			memset(buffer, 0, 200);
			sprintf(&buffer, "Done averages. \n");
			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);


	  	  	osDelay(500);

			gettingAverages = 0;


	  	  	//counter = 0;
	  	}


	}
  /* USER CODE END transmitViaUART */
}

/* USER CODE BEGIN Header_hasButtonBeenPressed */
/**
* @brief Function implementing the buttonPress_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hasButtonBeenPressed */
void hasButtonBeenPressed(void const * argument)
{
  /* USER CODE BEGIN hasButtonBeenPressed */
	int passedBy = 0;

	  /* Infinite loop */
	  for(;;)
	  {

		  osDelay(10);


		GPIO_PinState button_state = HAL_GPIO_ReadPin(User_Button_GPIO_Port, User_Button_Pin);

		 while(button_state == GPIO_PIN_RESET){ // button pressed (when button pushed, grounded)

			 // ensure that do not repeat;
		    	if(!passedBy){
		    		counter = (counter + 1) % 5;
		    		passedBy = 1;
		    	}

		 button_state = HAL_GPIO_ReadPin(User_Button_GPIO_Port, User_Button_Pin);


		 }

		passedBy = 0;

	  }
  /* USER CODE END hasButtonBeenPressed */
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

  memset(buffer, 0, 100);
  sprintf(&buffer, "Error at error handler \n");
  HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 5000);

  HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_RESET);
  __BKPT();

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
