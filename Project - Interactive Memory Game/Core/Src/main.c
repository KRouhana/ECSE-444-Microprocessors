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
#include "config.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "stm32l4s5i_iot01_qspi.h"

#include "accelerow.h"
#include "sound.h"
// Voice lines
#ifndef COMPRESSED
#include "welcome.h"
#include "instructions.h"
#include "player_turn.h"
#include "win.h"
#include "loss.h"
#include "start.h"
#include "try_again.h"
#include "calibrating.h"
#else
#include "welcome_compressed.h"
#include "instructions_compressed.h"
#include "player_turn_compressed.h"
#include "win_compressed.h"
#include "loss_compressed.h"
#include "start_compressed.h"
#include "try_again_compressed.h"
#include "calibrating_compressed.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HIGHSCORE_ADDRESS 0x00
#define NUMBER_OF_SCORES 10
#define STARTING_LEVEL 2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
bool button_pressed = false;
uint8_t difficulty_level = STARTING_LEVEL;
uint32_t last_press = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
enum BoardPosition duringGame();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef DEMO_ACCELEROW
enum BoardPosition
{
  DOUBLE_UP,
  BOARD_UP,
  BOARD_CENTER,
  BOARD_DOWN,
  DOUBLE_DOWN
};

enum BoardPosition update_position(enum BoardPosition previous_position,
    enum Direction direction)
{
  if (previous_position == BOARD_DOWN && direction == ACCELEROW_DOWN) return BOARD_DOWN;
  if (previous_position == BOARD_DOWN && direction == ACCELEROW_UP) return BOARD_CENTER;
  if (previous_position == BOARD_CENTER && direction == ACCELEROW_DOWN) return BOARD_DOWN;
  if (previous_position == BOARD_CENTER && direction == ACCELEROW_UP) return BOARD_UP;
  if (previous_position == BOARD_UP && direction == ACCELEROW_DOWN) return BOARD_CENTER;
  if (previous_position == BOARD_UP && direction == ACCELEROW_UP) return BOARD_UP;
  // Should never happen
  return BOARD_CENTER;
}

void demo_accelerow()
{
  // Turn LEDs on to indicate initialization is occurring
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);

  accelerow_init(&htim2, &huart1, Error_Handler);
  accelerow_calibrate();
  enum Direction direction;
  enum BoardPosition position = BOARD_CENTER;

  // Done initialization and calibration
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);

  while (1)
  {
    direction = accelerow_wait_for_move();
    position = update_position(position, direction);
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, position == BOARD_UP ? GPIO_PIN_SET : GPIO_PIN_RESET);
    // The blue LED lights up for GPIO_PIN_RESET
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, position == BOARD_DOWN ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
}
#endif

#ifdef DEMO_SOUND
// Plays a scale
void demo_sound()
{
  sound_init(&hdac1, &htim6, 16000);

  float scale[] = {C6_FREQUENCY, D6_FREQUENCY, E6_FREQUENCY, F6_FREQUENCY, G6_FREQUENCY, A6_FREQUENCY, B6_FREQUENCY, C7_FREQUENCY};
  size_t scale_length = sizeof(scale) / sizeof(float);

  for (size_t i = 0; i < scale_length; i++)
  {
    play_pure_tone(scale[i], 0.3);
    wait_for_sound_to_finish();
  }

  play_recording(welcome_data, sizeof(welcome_data) / sizeof(welcome_data[0]));
  wait_for_sound_to_finish();
}
#endif


size_t min(size_t a, size_t b)
{
  return a < b ? a : b;
}

uint32_t seed = 427986U;

uint32_t random_uint()
{
  seed ^= seed << 13;
  seed ^= seed >> 17;
  seed ^= seed << 5;
  return seed;
}

uint8_t add_high_score(uint8_t score)
{
  uint8_t high_score_list[NUMBER_OF_SCORES] = { 0 };
  if(BSP_QSPI_Read(high_score_list, HIGHSCORE_ADDRESS, NUMBER_OF_SCORES * sizeof(uint8_t))!=QSPI_OK) Error_Handler();

  //insert score
  int i = 0;
  bool new_score_set = false;
  for (i = 0; i < NUMBER_OF_SCORES; i++)
  {
    if (score >= high_score_list[i])
    {
      for(int j = NUMBER_OF_SCORES - 1; j > i; j--)
      {
        high_score_list[j] = high_score_list[j-1];
      }
      high_score_list[i] = score;
      new_score_set = true;
      break;
    }
  }
  if (new_score_set)
  {
    if(BSP_QSPI_Erase_Block(HIGHSCORE_ADDRESS)!= QSPI_OK) Error_Handler();
    if(BSP_QSPI_Write(high_score_list, HIGHSCORE_ADDRESS, NUMBER_OF_SCORES * sizeof(uint8_t))!=QSPI_OK) Error_Handler();
  }
  return new_score_set ? i:NUMBER_OF_SCORES;
}

// Plays a random sequence of intervals, and fills in the sequence of directions in which the board
// should be moved to win.
void get_random_sequence(enum Direction directions[])
{
  float tones[] = {C6_FREQUENCY, D6_FREQUENCY, E6_FREQUENCY, F6_FREQUENCY, G6_FREQUENCY, A6_FREQUENCY, B6_FREQUENCY, C7_FREQUENCY};
  for (int i = 0; i < difficulty_level; i++)
  {
    // Pick a first tone from E6 to A6
    int first_tone_index = 2 + (random_uint() % 4);
    float first_tone = tones[first_tone_index];
    // Pick a random interval from -2, -1, 1, and 2
    int interval = (random_uint() % 4);
    if (interval < 2)
    {
      interval -= 2;
    }
    else
    {
      interval -= 1;
    }
    float second_tone = tones[first_tone_index + interval];
    // Play the interval
    play_pure_tone(first_tone, 1);
    wait_for_sound_to_finish();
    play_pure_tone(second_tone, 1);
    wait_for_sound_to_finish();
    // Record the correct direction
    if (interval > 0)
    {
      directions[i] = ACCELEROW_UP;
    }
    else
    {
      directions[i] = ACCELEROW_DOWN;
    }
    // Wait before playing next interval
    HAL_Delay(500);
  }
}

void play_correct_sound()
{
  play_pure_tone(B6_FREQUENCY, 0.1);
  wait_for_sound_to_finish();
  play_pure_tone(G6_FREQUENCY, 0.3);
  wait_for_sound_to_finish();
}

void play_wrong_sound()
{
  play_pure_tone(C6_FREQUENCY, 0.1);
  wait_for_sound_to_finish();
  HAL_Delay(100);
  play_pure_tone(C6_FREQUENCY, 0.3);
  wait_for_sound_to_finish();
}

/**
 * Basic print function
 */
void print_to_uart(UART_HandleTypeDef *huart, char buffer[])
{
  HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 5000);
}

bool players_turn(enum Direction expected_directions[], enum Direction actual_directions[])
{
  print_to_uart(&huart1, "NOW MOVE! \r\n");
  bool made_a_mistake = false;
  for (int i = 0; i < difficulty_level; i++)
  {
    actual_directions[i] = accelerow_wait_for_move();
    if (actual_directions[i] == expected_directions[i])
    {
      play_correct_sound();
    }
    else
    {
      play_wrong_sound();
      made_a_mistake = true;
    }
  }
  return made_a_mistake;
}

// Inserts the given directions into the given string. The string must have space for at least len + 1 characters,
// including the null terminator.
void copy_sequence_into_str(enum Direction directions[], char *message, size_t len)
{
  for (int i = 0; i < len; i++)
  {
    message[i] = (directions[i] == ACCELEROW_UP) ? 'U' : 'D';
  }
  message[len] = '\0';
}

void print_diff(enum Direction expected[], enum Direction actual[], size_t len)
{
  char expected_sequence_msg[1024] = "Output: ";
  // Avoid buffer overflow by just clipping the length. 1024 notes should be more than high enough. Subtract 3 to leave
  // space for the trailing \r\n\0
  copy_sequence_into_str(expected, expected_sequence_msg + 8, min(len, 1024 - strlen(expected_sequence_msg) - 3));
  size_t current_len = strlen(expected_sequence_msg);
  expected_sequence_msg[current_len] = '\r';
  expected_sequence_msg[current_len+1] = '\n';
  expected_sequence_msg[current_len+2] = '\0';
  print_to_uart(&huart1, expected_sequence_msg);

  char actual_sequence_msg[1024] = " Input: ";
  copy_sequence_into_str(actual, actual_sequence_msg + 8, min(len, 1024 - strlen(actual_sequence_msg) - 3));
  current_len = strlen(actual_sequence_msg);
  actual_sequence_msg[current_len] = '\r';
  actual_sequence_msg[current_len+1] = '\n';
  actual_sequence_msg[current_len+2] = '\0';
  print_to_uart(&huart1, actual_sequence_msg);
}


void clear_scores()
{

  uint8_t high_score_list[NUMBER_OF_SCORES] = { 0 };
  if(BSP_QSPI_Erase_Block(HIGHSCORE_ADDRESS)!= QSPI_OK) Error_Handler();
  if(BSP_QSPI_Write(high_score_list, HIGHSCORE_ADDRESS, NUMBER_OF_SCORES*sizeof(uint8_t))!=QSPI_OK) Error_Handler();
  print_to_uart(&huart1, "High Scores Cleared!\r\n");
}
/**
 * Callback function for when the button has been pressed
 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BUTTON_Pin)
  {
    uint32_t press_time = HAL_GetTick();
    uint32_t time_diff = press_time-last_press;
    if (0 < time_diff && time_diff < 500)
    {
      clear_scores();
    }
    else
    {
      button_pressed = true;
    }
    last_press = press_time;



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
  MX_OCTOSPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

#ifdef DEMO_ACCELEROW
  demo_accelerow();
#elif defined(DEMO_SOUND)
  demo_sound();
#else
  sound_init(&hdac1, &htim6, 16000);
  accelerow_init(&htim2, &huart1, Error_Handler);
  if (BSP_QSPI_Init() != QSPI_OK) Error_Handler();
  float startUpTone[] = {C6_FREQUENCY, E6_FREQUENCY, G6_FREQUENCY, E6_FREQUENCY, C6_FREQUENCY};
  for (size_t i = 0; i < 4; i++)
  {
    play_pure_tone(startUpTone[i], 0.3);
    wait_for_sound_to_finish();
  }
  play_pure_tone(startUpTone[4], 0.6);
  wait_for_sound_to_finish();

  play_recording(welcome_data, sizeof(welcome_data) / sizeof(welcome_data[0]));
  print_to_uart(&huart1, "Welcome to the Marvelous Microprocessor Memory Match!\r\n");
  wait_for_sound_to_finish();

  play_recording(instructions_data, sizeof(instructions_data) / sizeof(instructions_data[0]));
  print_to_uart(&huart1, "A sequence of intervals will play. When instructed, move the board up for each interval that was increasing in pitch and down for each interval that was decreasing in pitch.\r\n");
  wait_for_sound_to_finish();

  play_recording(start_data, sizeof(start_data) / sizeof(start_data[0]));
  print_to_uart(&huart1, "Press the blue button to start.\r\n");
  wait_for_sound_to_finish();

  print_to_uart(&huart1, "Double-click the blue button to clear scores.\r\n");


  while (true)
  {
    // Wait for the player to be ready
    if(!button_pressed) continue;

    play_recording(calibrating_data, sizeof(calibrating_data) / sizeof(calibrating_data[0]));
    print_to_uart(&huart1, "Get ready, calibrating...\r\n");
    wait_for_sound_to_finish();

    // Turn blue LED on to show we're calibrating. RESET means on for this LED.
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
    accelerow_calibrate();
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);

    enum Direction expected_directions[difficulty_level];
    enum Direction actual_directions[difficulty_level];
    get_random_sequence(expected_directions);

    play_recording(player_turn_data, sizeof(player_turn_data) / sizeof(player_turn_data[0]));
    print_to_uart(&huart1, "Your turn!\r\n");
    wait_for_sound_to_finish();

    bool made_a_mistake = players_turn(expected_directions, actual_directions);

    if (made_a_mistake)
    {
      play_recording(loss_data, sizeof(loss_data) / sizeof(loss_data[0]));
      print_to_uart(&huart1, "You lose...\r\n");
      wait_for_sound_to_finish();
      print_diff(expected_directions, actual_directions, difficulty_level);

      uint8_t score = difficulty_level - 1;
      uint8_t score_position;
      if (difficulty_level == STARTING_LEVEL)
      {
        score_position = NUMBER_OF_SCORES;
      }
      else
      {
        score_position = add_high_score(score);
      }

      uint8_t high_score_list[NUMBER_OF_SCORES] = { 0 };
      if (BSP_QSPI_Read(high_score_list, HIGHSCORE_ADDRESS, NUMBER_OF_SCORES * sizeof(uint8_t)) != QSPI_OK) Error_Handler();
      print_to_uart(&huart1, "High Scores:\r\n");
      char message[64] = "";
      for(int i = 0; i < NUMBER_OF_SCORES; i++){
        if (i == score_position)
        {
          snprintf(message, 64, "%d: %d <- Your Score! \n\r", i + 1, high_score_list[i]);
          print_to_uart(&huart1, message);
        }
        else if (high_score_list[i] == 0)
        {
          break;
        }
        else
        {
          snprintf(message, 64, "%d: %d \n\r", i + 1, high_score_list[i]);
          print_to_uart(&huart1, message);
        }
      }

      difficulty_level = STARTING_LEVEL;
    }
    else
    {
      play_recording(win_data, sizeof(win_data) / sizeof(win_data[0]));
      print_to_uart(&huart1, "You win!\r\n");
      wait_for_sound_to_finish();
      difficulty_level++;
    }

    play_recording(try_again_data, sizeof(try_again_data) / sizeof(try_again_data[0]));
    print_to_uart(&huart1, "Press the blue button to try again\r\n");
    wait_for_sound_to_finish();

    button_pressed = false;

  }
#endif
  /* USER CODE END 2 */

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
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim2.Init.Prescaler = 374;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7500 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_LED_Pin */
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
