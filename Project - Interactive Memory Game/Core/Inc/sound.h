/*
 * sound.h
 *
 *  Created on: Nov 9, 2023
 *      Author: Gabriel Lacroix <lacroixgabriel@gmail.com>
 */

#ifndef INC_SOUND_H_
#define INC_SOUND_H_

#include "config.h"
#include <stdint.h>
#include <stddef.h>
#include "stm32l4xx_hal.h"

/**
 * How to use this driver
 *
 * First, import arm_math.h and its associated static library as seen in lab 0.
 *
 * In .ioc file:
 * 1. Under Analog -> DAC1 -> Mode -> Out 1 connected to, select "only to external pin".
 * 2. Under Analog -> DAC1 -> Configuration -> Parameter Settings -> Output Buffer, select "Enable".
 * 3. Under Analog -> DAC1 -> Configuration -> Parameter Settings -> Trigger, select "Timer 6 Trigger Out event".
 * 4. Under Analog -> DAC1 -> Configuration -> DMA Settings, click "Add" and leave all the settings to their default values.
 * 5. Under Timers -> TIM6 -> Mode, check "Activated".
 * 6. Under Timers -> TIM6 -> Configuration -> Parameter Settings -> Counter Settings, set the counter period to "(Internal Clock Frequency / Desired Sample Rate) - 1". For an internal clock frequency of 120 MHz and a desired sample rate of 40KHz, this would be 2999.
 * 7. Under Timers -> TIM6 -> Configuration -> Parameter Settings -> Trigger Event Selection, select "Update Event".
 *
 * In your code:
 * 1. `#include` this header file.
 * 2. Call `sound_init` once.;
 * 3. Use `play_recording` and `play_pure_tone` for simple playback. Each of these have documentation comments that explain how to use them.
 * 4. To wait for a sound to finish playing, use `wait_for_sound_to_finish`.
 * 5. For lower level interaction with the driver, use `play_buffer_iterator`. More details on how to use it are available in its documentation comment.
 *
 * In hardware:
 * The output of the DAC should be available on pin D7.
 *
 * Some example frequencies for the notes C6 through C7 are defined below for convenience.
 */

// A simple musical scale:
#define C6_FREQUENCY 1046.502
#define D6_FREQUENCY 1174.659
#define E6_FREQUENCY 1318.510
#define F6_FREQUENCY 1396.913
#define G6_FREQUENCY 1567.982
#define A6_FREQUENCY 1760.000
#define B6_FREQUENCY 1975.533
#define C7_FREQUENCY 2093.005

/**
 * Initializes the sound driver.
 *
 * Note: The maximum playable frequency (with at least two points per period) is 1/2 of the DAC frequency.
 *
 * @param hdac Pointer to the handle of the DAC to use for playback
 * @param htim Pointer to the handle of the timer with which DAC DMA is configured
 * @param sample_rate The sample rate of the DAC (given by the timer) in Hz
 */
void sound_init(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, float sample_rate);

/**
 * Plays a pure tone of the given frequency for the given duration.
 *
 * @param frequency The frequency of the tone to play in Hz.
 * @param duration The duration of the tone to play in seconds.
 */
void play_pure_tone(float frequency, float duration);

#ifndef COMPRESSED
typedef uint16_t sample_t;
#else
typedef uint8_t sample_t;
#endif

/**
 * Plays the given recording.
 *
 * @param recording The recording to play.
 * @param recording_length The length of the recording in samples.
 */
void play_recording(const sample_t *recording, size_t recording_length);

/**
 * Waits for the currently playing sound to finish.
 */
void wait_for_sound_to_finish();

/**
 * A buffer of data to play.
 */
typedef struct {
    const uint16_t *data;
    size_t data_length;
} buffer;

/**
 * Plays the given buffer iterator.
 *
 * A buffer iterator is defined by two values:
 * 1. A mutable state, `buffer_iterator_state`.
 * 2. A function that takes the state and returns the next buffer to play, `next_buffer`.
 *
 * This function repeatedly calls `next_buffer` with the current state and expects to receive a buffer to play with length smaller or equal to `MAX_DMA_BUFFER_LENGTH`. If the buffer's data pointer is NULL or the length is 0, the playback will stop.
 *
 * @param buffer_iterator_state
 * @param next_buffer
 */
void play_buffer_iterator(void *buffer_iterator_state, buffer (*next_buffer)(void *));

#endif /* INC_SOUND_H_ */
