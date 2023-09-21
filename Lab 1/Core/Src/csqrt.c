/*
 * csqrt.c
 *
 *  Created on: Sep 10, 2023
 *      Author: karl
 */
#include "main.h"
#include "math.h"

void cSqrt(const float in, float *out){


	if(in <=0){
		*out = 0.0;
		return;
	}

	float guess = in;

	 while( fabs((guess * guess) / in - 1) >= 0.00001 ) {
	        guess = ((in/guess) + guess) / 2;


	 }

	 *out = guess;

}
