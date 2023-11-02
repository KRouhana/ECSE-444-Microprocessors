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

char buffer[100];

int16_t gyro[3];

int16_t magneto[3];

uint32_t flashAddresses[2];

int16_t temp;

int16_t pressure;

int counter = -1;

int16_t averageGyro[2];
int16_t averageMagneto[2];
int16_t averageTemp[2];
int16_t averagePressure[2];

int16_t samplesGyro, samplesMagneto, samplesTemp, samplesPressure = 0;

int16_t xGyroAverage, yGyroAverage, zGyroAverage = 0;


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


//  BSP_ACCELERO_Init();
//
//  BSP_HSENSOR_Init();

  BSP_GYRO_Init();

  BSP_MAGNETO_Init();

  BSP_PSENSOR_Init();

  BSP_TSENSOR_Init();

  BSP_QSPI_Init();

  HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_SET);



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
  osThreadDef(UART_Task, transmitViaUART, osPriorityIdle, 0, 128);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* definition and creation of buttonPress_Tas */
  osThreadDef(buttonPress_Tas, hasButtonBeenPressed, osPriorityIdle, 0, 128);
  buttonPress_TasHandle = osThreadCreate(osThread(buttonPress_Tas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	memset(buffer, 0, 100);
	sprintf(&buffer, "Ready to go \n");
  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, 100, 5000);

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


void getAverages(){


    //populate an array with the max number of sample gotten
    int16_t tempGyro[samplesGyro * 3];

    if(BSP_QSPI_Read(tempGyro, 0x00000000, samplesGyro * sizeof(gyro)) != QSPI_OK)
	{
    	Error_Handler();
	}



    //Stop 2 before to not get out of bounds
    for(int i = 0; i < samplesGyro; i++){

    	xGyroAverage = xGyroAverage + tempGyro[3*i]; 		//get x values

    	yGyroAverage = yGyroAverage + tempGyro[3*i+1];	//get y values

    	zGyroAverage = zGyroAverage + tempGyro[3*i+2];	//get z values

    }

    xGyroAverage = xGyroAverage/samplesGyro;
    yGyroAverage = yGyroAverage/samplesGyro;
    zGyroAverage = zGyroAverage/samplesGyro;



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


	  	BSP_GYRO_GetXYZ(gyro); // (x, y, z)

	  	//Store gyro x,y,z values
	  	//Use addresses from 0x00000000 to 0x00030000
	  	if(BSP_QSPI_Write(gyro, 0x00000000 + sizeof(gyro) * samplesGyro , sizeof(gyro)) != QSPI_OK){
	  		Error_Handler();
	  	}
	  	else{
	  		samplesGyro++;
	  	}


	  	BSP_MAGNETO_GetXYZ(magneto); // (x, y, z)


	  	//Store magneto x,y,z values
	  	//Use addresses from 0x00030000 to 0x00060000
	  	if(BSP_QSPI_Write(magneto, 0x00030000 + sizeof(magneto) * samplesMagneto, sizeof(magneto)) != QSPI_OK){
	  		Error_Handler();
	  	}
	  	else{
	  		samplesMagneto++;
	  	}



	  	temp = BSP_TSENSOR_ReadTemp();

	  	//Store temperature values
	  	if(BSP_QSPI_Write(temp, 0x00060000 + sizeof(temp) * samplesTemp, sizeof(temp)) != QSPI_OK){
	  		Error_Handler();
	  	}else{
	  		samplesTemp++;

	  	}


	  	pressure = BSP_PSENSOR_ReadPressure();

	  	//Store pressure values
	  	if(BSP_QSPI_Write(temp, 0x00070000 + sizeof(pressure) * samplesPressure, sizeof(pressure)) != QSPI_OK){
	  		Error_Handler();
	  	}else{
	  		samplesPressure++;

	  	}


	      osDelay(100);

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
	int hasPassed = 0;
  /* Infinite loop */
  for(;;)
  {
	  memset(buffer, 0, 100);

	  	if(counter == 0){

	  		hasPassed = 0;

	  		int x = gyro[0];
	  		int y = gyro[1];
	  		int z = gyro[2];

	  		sprintf(&buffer, "GYRO: x: %d, y: %d, z: %d \n", x,y,z);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, 100, 5000);

	  	}

	  	else if(counter == 1){

	  		hasPassed = 0;

	  		int x = magneto[0];
	  		int y = magneto[1];
	  		int z = magneto[2];

	  		sprintf(&buffer, "Magneto: x: %d, y: %d, z: %d \n", x,y,z);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, 100, 5000);

	  	}

	  	else if(counter == 2){

	  		hasPassed = 0;

	  		sprintf(&buffer, "Temperature: %d \n", (int) temp);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, 100, 5000);

	  	}

	  	else if(counter == 3){

	  		hasPassed = 0;

	  		sprintf(&buffer, "Pressure: %d \n", (int) pressure);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, 100, 5000);


	  	}

	  	else if(!hasPassed && counter == 4 ){

	  		getAverages();
	  		sprintf(&buffer, "GYRO Average: x: %d, y: %d, z: %d \nNumber of samples is: %d \n", (int) xGyroAverage,(int) yGyroAverage,(int) zGyroAverage, (int) samplesGyro);
	  	  	HAL_UART_Transmit(&huart1, (uint8_t*) buffer, 100, 5000);
	  	  	hasPassed = 1;

	  	}

	      osDelay(100);
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
	int was_pressed = 0;

	  /* Infinite loop */
	  for(;;)
	  {
		GPIO_PinState button_state = HAL_GPIO_ReadPin(User_Button_GPIO_Port, User_Button_Pin);


		 if(button_state == GPIO_PIN_RESET) // button pressed (when button pushed, grounded)
		    {
		    	if(was_pressed == 0) // ensure that do not repeat;
		    	{
		    		counter = (counter + 1) % 5;
		    	}
		    	was_pressed = 1;
		    }
		    else if(button_state == GPIO_PIN_SET)
		    {
		    	was_pressed = 0;
		    }



	    osDelay(10);
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
