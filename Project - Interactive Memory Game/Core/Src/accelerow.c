#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "accelerow.h"
#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01_accelero.h"
#ifdef ACCELEROW_DEBUG
#include <stdio.h>
#include <string.h>
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
static int16_t mean(int16_t[], size_t);

// Take a moving average over this number of samples
#define MOVING_AVG_LEN 5
// A move is detected when the moving average of acceleration has a magnitude
// at least this large
#define ACTIVATION_THRESHOLD 200
// A move is finished when the moving average of acceleration has a magnitude
// less than DEACTIVATION_THRESHOLD for at least DEACTIVATION_LEN samples
#define DEACTIVATION_THRESHOLD 50
#define DEACTIVATION_LEN 10
// Number of samples to take during calibration
#define NUM_CALIBRATION_SAMPLES 64

static TIM_HandleTypeDef *measurement_timer;
static UART_HandleTypeDef *uart;
static void (*error)(void);
volatile static bool is_time_to_measure = false;
static int16_t baseline_acceleration = 0;
static bool is_calibrated = false;

void accelerow_init(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart, void (*error_)(void))
{
  measurement_timer = htim;
  uart = huart;
  error = error_;

  if (HAL_TIM_Base_Start_IT(htim) != HAL_OK) error();
  if (BSP_QSPI_Init() != QSPI_OK) error();
  if (BSP_ACCELERO_Init() != ACCELERO_OK) error();
}

// Louis' board seems to have a different gain along the positive and negative
// z axes! Maybe measuring those here would lead to better results, although
// it would make the calibration routine more complex and less user-friendly.
void accelerow_calibrate(void)
{
  int16_t z_accelerations[NUM_CALIBRATION_SAMPLES] = { 0 };
  int index = 0;
  int16_t accelero_readings[3] = { 0 };

  while (1)
  {
    if (!is_time_to_measure) continue;
    is_time_to_measure = false;

    BSP_ACCELERO_AccGetXYZ(accelero_readings);
    z_accelerations[index] = accelero_readings[2];
    index++;
    if (index == NUM_CALIBRATION_SAMPLES) break;
  }

  // We could also try measuring the variance here and adjust the acceleration
  // threshold accordingly, if that's necessary.
  baseline_acceleration = mean(z_accelerations, NUM_CALIBRATION_SAMPLES);
  is_calibrated = true;
}

enum Direction accelerow_wait_for_move()
{
  if (!is_calibrated) error();

  int16_t z_accelerations[MOVING_AVG_LEN] = { 0 };
  int index = 0;
  int16_t accelero_xyz[3] = { 0 };
#ifdef ACCELEROW_DEBUG
  #define MAX_MSG_LEN 64
  char message[MAX_MSG_LEN] = "";
#endif

  // Wait for the board to settle down so that we don't detect a new move
  // before the previous one is done

  // Number of consecutive samples in which the board is basically immobile
  int num_still_samples = 0;
  bool buffer_full = false;
  while (true)
  {
    if (!is_time_to_measure) continue;
    is_time_to_measure = false;

    BSP_ACCELERO_AccGetXYZ(accelero_xyz);
    z_accelerations[index] = accelero_xyz[2] - baseline_acceleration;
#ifdef ACCELEROW_DEBUG
    snprintf(message, MAX_MSG_LEN, "%d,", z_accelerations[index]);
    HAL_UART_Transmit(uart, (uint8_t *) message, strlen(message), 1000);
#endif
    if (index == MOVING_AVG_LEN - 1) buffer_full = true;
    index = (index + 1) % MOVING_AVG_LEN;

    // Wait for the buffer to fill up, otherwise the moving average will
    // already be basically zero at first
    if (!buffer_full) continue;

    int16_t running_avg = mean(z_accelerations, MOVING_AVG_LEN);
    bool is_board_immobile = abs(running_avg) < DEACTIVATION_THRESHOLD;
    if (is_board_immobile)
    {
      num_still_samples++;
      if (num_still_samples >= DEACTIVATION_LEN) break;
    }
    else
    {
      num_still_samples = 0;
    }
  }

  while (true)
  {
    if (!is_time_to_measure) continue;
    is_time_to_measure = false;

    BSP_ACCELERO_AccGetXYZ(accelero_xyz);
    z_accelerations[index] = accelero_xyz[2] - baseline_acceleration;
#ifdef ACCELEROW_DEBUG
    snprintf(message, MAX_MSG_LEN, "%d,", z_accelerations[index]);
    HAL_UART_Transmit(uart, (uint8_t *) message, strlen(message), 1000);
#endif
    index = (index + 1) % MOVING_AVG_LEN;

    int16_t running_avg = mean(z_accelerations, MOVING_AVG_LEN);
    if (running_avg >= ACTIVATION_THRESHOLD) return ACCELEROW_UP;
    else if (running_avg <= -ACTIVATION_THRESHOLD) return ACCELEROW_DOWN;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == measurement_timer) is_time_to_measure = true;
}

static int16_t mean(int16_t values[], size_t len)
{
  int64_t sum = 0;
  for (int i = 0; i < len; i++)
  {
    sum += values[i];
  }
  return (int16_t)(sum / len);
}
