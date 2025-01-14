/*
 * sound.c
 *
 *  Created on: Nov 10, 2023
 *      Author: Gabriel Lacroix <lacroixgabriel@gmail.com>
 */

#include "sound.h"

// Private includes
#include <stdbool.h>
#include "arm_math.h"
// Included here in order to use the `Error_Handler` function.
#include "main.h"

// The length of the buffer is limited to a 16-bit unsigned integer (0, 2^16 - 1)
#define MAX_DMA_BUFFER_LENGTH 65535

bool initialized = false;
DAC_HandleTypeDef *hdac;
float dac_frequency;

void sound_init(DAC_HandleTypeDef *hdac_, TIM_HandleTypeDef *htim, float sample_rate) {
    hdac = hdac_;
    dac_frequency = sample_rate;
    initialized = true;
    if (HAL_TIM_Base_Start_IT(htim) != HAL_OK) Error_Handler();
}

void *currently_playing_buffer_iterator_state;
buffer (*currently_playing_buffer_iterator_next_buffer)(void *);
buffer currently_playing_buffer;
bool currently_playing = false;

void play_buffer_iterator(void *buffer_iterator_state, buffer (*next_buffer)(void *)) {
    if (!initialized) {
        Error_Handler();
    }

    // Stop whatever's currently playing to avoid a data race when modifying the global state.
    HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);

    currently_playing_buffer_iterator_state = buffer_iterator_state;
    currently_playing_buffer_iterator_next_buffer = next_buffer;
    currently_playing = true;

    HAL_DAC_ConvCpltCallbackCh1(hdac);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac_) {
    // Only one DAC can be used at a time
    if (hdac_ != hdac) {
        Error_Handler();
    }

    currently_playing_buffer = currently_playing_buffer_iterator_next_buffer(currently_playing_buffer_iterator_state);

    if (currently_playing_buffer.data != NULL && currently_playing_buffer.data_length > 0) {
        if (HAL_DAC_Start_DMA(
                hdac,
                DAC_CHANNEL_1,
                (uint32_t *) currently_playing_buffer.data,
                currently_playing_buffer.data_length,
                DAC_ALIGN_12B_R
        ) != HAL_OK) {
            Error_Handler();
        }
    } else {
        HAL_DAC_Stop_DMA(hdac, DAC_CHANNEL_1);
        currently_playing = false;
    }
}

void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac) {
    Error_Handler();
}

void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac) {
    Error_Handler();
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
    // Do nothing.
}

/**
 * Waits for the currently playing sound to finish.
 */
void wait_for_sound_to_finish() {
    while (currently_playing) {
        // Wait for the sound to finish.
    }
}

// Buffer iterator used to play a fixed length recording.
typedef struct {
    const sample_t *recording;
    size_t recording_length;
    size_t next_buffer_start;
} recording_buffer_iterator_state;


#define RECORDING_BUFFER_LENGTH 16000
uint16_t recording_buffer[RECORDING_BUFFER_LENGTH];

static inline uint16_t decompress(uint8_t input) {
    if (input >> 4 == 0) {
        return (0b10000000 << 4) | (input & 0b00001111);
    } else if (input >> 4 == 1) {
        return (0b10000001 << 4) | (input & 0b00001111);
    } else if (input >> 4 == 0b1000) {
        return (0b01111111 << 4) | (input & 0b00001111);
    } else if (input >> 4 == 0b1001) {
        return (0b01111110 << 4) | (input & 0b00001111);
    } else if (input >> 4 > 0b1001) {
        uint16_t output = (0xFFFF << 5) | (input & 0b00001111);
        uint16_t shift = (input >> 4) - 0b1001;
        output <<= shift;
        output &= 0b11111111111; // Keep only the 11 least significant bits.
        output |= 1 << (shift - 1);
        return output;
    } else /* input >> 4 ~= 0b0xxx */ {
        uint16_t output = input & 0b00001111;
        uint16_t shift = (input >> 4) - 1;
        output <<= shift;
        output |= 1 << (shift - 1);
        output |= 1 << 11;
        output |= 1 << (shift + 4);
        return output;
    }
}

buffer recording_buffer_iterator_next(void *_state) {
    recording_buffer_iterator_state *state = (recording_buffer_iterator_state *) _state;
    buffer next;
    if (state->next_buffer_start >= state->recording_length)
    {
        next.data = NULL;
        next.data_length = 0;
        return next;
    }
    sample_t *start = (sample_t *) state->recording + state->next_buffer_start;
    int length;
    if (state->next_buffer_start + RECORDING_BUFFER_LENGTH > state->recording_length) {
        length = state->recording_length - state->next_buffer_start;
    } else {
        length = RECORDING_BUFFER_LENGTH;
    }
    state->next_buffer_start += RECORDING_BUFFER_LENGTH;

    for (int i = 0; i < length; i++) {
        #ifndef COMPRESSED
        recording_buffer[i] = start[i];
        #else
        recording_buffer[i] = decompress(start[i]);
        #endif
    }

    next.data = recording_buffer;
    next.data_length = length;

    return next;
}

recording_buffer_iterator_state recording_state;

/**
 * Plays the given recording.
 *
 * @param recording The recording to play.
 * @param recording_length The length of the recording in samples.
 */
void play_recording(const sample_t *recording, size_t recording_length)
{
    recording_state = (recording_buffer_iterator_state){
        .recording = recording,
        .recording_length = recording_length,
        .next_buffer_start = 0};
    play_buffer_iterator(&recording_state, recording_buffer_iterator_next);
}

// Buffer iterator used to play a pure tone for a fixed duration.
typedef struct {
    const uint16_t *period_buffer;
    size_t period_buffer_length;
    int iteration_number;
    int total_iterations;
} pure_tone_buffer_iterator_state;

buffer pure_tone_buffer_iterator_next(void *_state) {
    pure_tone_buffer_iterator_state *state = (pure_tone_buffer_iterator_state *) _state;
    buffer next;
    if (state->iteration_number == state->total_iterations) {
        next.data = NULL;
        next.data_length = 0;
    } else {
        state->iteration_number++;
        next.data = state->period_buffer;
        next.data_length = state->period_buffer_length;
    }
    return next;
}

uint16_t round_float(float x) {
    return (uint16_t) (x + 0.5);
}

void generate_sine_wave(uint16_t *buffer, size_t buffer_length, float frequency) {
    float amplitude = 2047;
    float offset = 2047;
    float sample_rate = dac_frequency;
    for (size_t i = 0; i < buffer_length; i++) {
        buffer[i] = round_float(amplitude * arm_sin_f32((float) i * 2 * PI * frequency / sample_rate) + offset);
    }
}

#define PURE_TONE_BUFFER_LENGTH 1000
uint16_t pure_tone_buffer[PURE_TONE_BUFFER_LENGTH];
pure_tone_buffer_iterator_state pure_tone_state;

/**
 * Plays a pure tone of the given frequency for the given duration.
 *
 * @param frequency The frequency of the tone to play in Hz.
 * @param duration The duration of the tone to play in seconds.
 */
void play_pure_tone(float frequency, float duration) {
    // Only compute the first period of the sine wave.
    size_t period_length = (size_t) (dac_frequency / frequency);
    generate_sine_wave(pure_tone_buffer, period_length, frequency);
    // Fill the buffer as much as possible to avoid DMA underruns.
    size_t number_of_periods_in_buffer = (PURE_TONE_BUFFER_LENGTH / period_length);
    size_t buffer_length = number_of_periods_in_buffer * period_length;
    for (size_t i = period_length; i < buffer_length; i += period_length) {
        for (size_t j = 0; j < period_length; j++) {
            pure_tone_buffer[i + j] = pure_tone_buffer[j];
        }
    }
    pure_tone_state = (pure_tone_buffer_iterator_state) {
            .period_buffer = pure_tone_buffer,
            .period_buffer_length = buffer_length,
            .iteration_number = 0,
            .total_iterations = (int) (duration * frequency / (float) number_of_periods_in_buffer)
    };
    play_buffer_iterator(&pure_tone_state, pure_tone_buffer_iterator_next);
}
