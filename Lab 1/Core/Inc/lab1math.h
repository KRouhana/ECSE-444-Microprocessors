/*
 * lab1math.h
 *
 *  Created on: Sep 9, 2023
 *      Author: karl
 */

#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_

void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmSqrt(const float32_t in, float32_t *out);

void cSqrt(const float in, float *out);

float cnewtonsMethod(float x, float w, float phi);

extern float asmTranscendental(float x0, float w, float phi);

#endif /* INC_LAB1MATH_H_ */
