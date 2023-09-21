/*
 * ctranscendental.c
 *
 *  Created on: Sep 12, 2023
 *      Author: karl
 */

#include "main.h"
#define ARM_MATH_CM4
#include "arm_math.h"
#include "math.h"

float defunc(float x0, float w, float phi, float B){

/**
 * f(x) = x^2 - cos(wx+φ) -> B = wx+φ
 */

	float y = x0*x0 - arm_cos_f32(B);

	return y;
}

float firstDerivative(float x0, float w, float phi, float B){

/**
 * f'(x) = 2x + wsin(wx+φ) -> B = wx+φ
 */

	float yPrime = 2*x0 + w*arm_sin_f32(B);

	return yPrime;
}

float cnewtonsMethod(float x0, float w, float phi){

	int maxIteration = 50;
	float tolerance = 0.015625;
	float epsilon = 0.015625;
	float x1;

	for(int i =0; i< maxIteration; i++){

		float B = x0*w + phi;

		float y = defunc(x0, w, phi, B);
		float yPrime = firstDerivative(x0, w, phi, B);

		if(fabs(yPrime) < epsilon){
			break;
		}

		x1 = x0 - y/yPrime;

		if(fabs(x1-x0) <= tolerance){
			return x1;
		}
		x0 = x1;

	}
	return x1;
}
