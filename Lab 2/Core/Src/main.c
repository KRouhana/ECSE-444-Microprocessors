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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))

/* memory address from datasheet */
#define VREFINT ((uint16_t*)((uint32_t) 0x1FFF75AA))
#define TS_CAL1 ((uint16_t*)((uint32_t) 0x1FFF75A8))
#define TS_CAL2 ((uint16_t*)((uint32_t) 0x1FFF75CA))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define TS_CAL1_TEMP 30
#define TS_CAL2_TEMP 130

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */


//Variable for each wave
uint32_t triangleValue, sawValue, sineValue, temperatureValue = 0;

float x, temperatureMeasured = 0;

int isIncreasing, soundCounter, timer = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ADC Voltage Initialization */
void ADC_Voltage_Init(){
	// ADC for voltage by using polling
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_VREFINT;
	  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	  sConfig.Rank = ADC_REGULAR_RANK_1;

	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
		  Error_Handler();
	  }

}

//ADC_SAMPLETIME_640CYCLES_5
//ADC_SAMPLETIME_247CYCLES_5
/* ADC Temperature Initialization */
void ADC_Temperature_Init(){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
		Error_Handler();
	}

}
/* Retrieve ADC Value from channel selected */
uint32_t getADCValue(){
	HAL_ADC_Start(&hadc1); // Activates ADC peripheral and starts conversion

	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK){ // Waits for ADC conversion to be done
		  Error_Handler();
	}

	uint32_t ADC_value = HAL_ADC_GetValue(&hadc1); // Retrieve the converted value
	HAL_ADC_Stop(&hadc1); // Stops conversion and disables the ADC peripherals

	return ADC_value;
}



/*
 * Formula found in the Chip Document p
*/
float Voltage_Conversion(uint32_t raw_ADC_voltage_value){

	return 3000 * (*VREFINT)/ raw_ADC_voltage_value;
}

/*
 * Formula found in the Chip Document p
*/
float Temperature_Conversion(uint32_t raw_ADC_temperature_value, float VREF){

	float ts_data = raw_ADC_temperature_value * VREF/3000;

	return (TS_CAL2_TEMP - TS_CAL1_TEMP)/ ((float)*TS_CAL2 - (float)*TS_CAL1) * ((float)ts_data - (float)*TS_CAL1) + 30;
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
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  //Flag for state of button
  GPIO_PinState buttonState;

  //Variable to check which state we're in, to check if the button has been pressed once
  int flag, passedBy = 0;

  //Variable at program boot
  int initialState = 1;

  //The output for the DAC
  uint16_t outputFrequency = 0;


  //Launch the DAC channels
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

  //Launch the timer
  HAL_TIM_Base_Start_IT(&htim2);

  //Start with the LED off
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Get state of button
	  buttonState =  HAL_GPIO_ReadPin(userButton_GPIO_Port, userButton_Pin);

	  //Check if the button has been pressed
  	  while(!buttonState){

  		  //Switch states
  		  if(!passedBy){
  			  flag = !flag;
  			  //We pressed the button once, hence we switched states, so don't switch again (this is because sometimes one press can
  			  //be registered as two or more presses.
  			  passedBy = 1;
  		  }

  		  initialState = 0;
  		  buttonState = HAL_GPIO_ReadPin(userButton_GPIO_Port, userButton_Pin);

  	  }

  	  //Change back the flag to 0
  	  passedBy = 0;


  	    //Check which state we're in

	    //If we just started the program, play a dull sound
	  	  if(initialState){

	  		  outputFrequency = sawValue;

	    //If we pressed the button, cycle through sound waves
	  	  }else if(!flag){

	  		//Set the LED to Off
	  		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

	  		//Switch between sounds every second
			if(soundCounter == 0){

						outputFrequency = sineValue;
			}

			else if(soundCounter == 1){
						outputFrequency = sawValue;
			}

			else if(soundCounter == 2){
						outputFrequency = triangleValue;

			}


		//If we pressed the button again, change to temperature value
	  	  }else{

		  //Set the LED to On
		  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

		  //Get measure temperature value
		  ADC_Temperature_Init();
		  uint32_t raw_ADC_temperature_value = getADCValue(); // Retrieve the converted value

		  //Get voltage reference value
		  ADC_Voltage_Init();
		  uint32_t raw_ADC_voltage_value = getADCValue(); // Retrieve the converted value

		  float VREF = Voltage_Conversion(raw_ADC_voltage_value);

		  temperatureMeasured = Temperature_Conversion(raw_ADC_temperature_value, VREF);

		  float temp = temperatureMeasured;

		  outputFrequency = temperatureValue;
  		 }


  	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, outputFrequency);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, outputFrequency);


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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim2.Init.Prescaler = 5000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6;
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
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : userButton_Pin */
  GPIO_InitStruct.Pin = userButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/*
 * The timer interrupts every ~0.29ms when i checked with ITM_Port (I tried to get a clock cycle of 0.25ms)
 * The period is 4 ms, which is why the step is 16
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//Debugging
	//ITM_Port32(31) = timer;


	//Get Triangle Value
	//I'm stopping at 4088 because it's the max int before getting to 4095 using integer division (4095/8 = 511.875 => 511)
	if(isIncreasing == 0){
		if(triangleValue < 4088) {
			triangleValue+= 4095/8;
		}else{
			isIncreasing = 1;
			}
	}else{
		if(triangleValue > 0) {
			triangleValue-= 4095/8;
		}else {
			isIncreasing = 0;
		}
	}


	//Get Saw Value
	//I'm stopping at 4080 because it's the max int before getting to 4095 using integer division (4095/16 = 255.9375 => 255)
	if(sawValue < 4080){
		sawValue+=4095/16;
	}else{
		sawValue = 0;
	}


	//Get Sine Value
	float twopi = 6.28319;
	float period = 16.0;

	x += twopi/period;

	float sin_wave = arm_sin_f32(x);


	if(x >= twopi){
		x -= twopi;
	}

	sineValue = (uint16_t)((sin_wave * 2048) + 2048);


	//Get temperature dependent SawFunction
	if(temperatureValue < 4095){
		temperatureValue+=4095/(temperatureMeasured);
	}else{
		temperatureValue = 0;
	}


	//Keep a counter
	timer++;

	//When we get to one second, reset the timer counter (T = 29 ms, 1s ~ 3500*29ms)
	if(timer == 3500){
		timer = 0;
		soundCounter++;
	}

	//When 3 seconds has elapsed, reset the cycle counter
	if(soundCounter == 3){
		soundCounter = 0;
	}


}


/* USER CODE END 4 */

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
