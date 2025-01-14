/*
 * High-level wrapper for accelerometer inputs.
 */

#ifndef INC_ACCELEROW_H_
#define INC_ACCELEROW_H_

#include "stm32l4xx_hal.h"

enum Direction {
  ACCELEROW_UP,
  ACCELEROW_DOWN
};

/*
 * Performs any required initialization.
 */
void accelerow_init(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart, void (*error)(void));

/*
 * Performs calibration under the assumption the user is currently holding the
 * board still.
 */
void accelerow_calibrate(void);

/*
 * Waits for the user to move the board and returns the direction in which
 * they moved it.
 */
enum Direction accelerow_wait_for_move();

#endif /* INC_ACCELEROW_H_ */
