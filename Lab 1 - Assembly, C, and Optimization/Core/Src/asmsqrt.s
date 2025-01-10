/*
 * asmsqrt.s
 *
 *  Created on: Sep 10, 2023
 *      Author: karl
 */


// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmSqrt, which is expected by lab1math.h
.global asmSqrt

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
 * void asmSqrt(const float32_t in, float32_t *out);
 *
 * S0 = input
 * R3 = pointer to output
 */

//I know the output should be pointed to R1, but it does not work, hence I used R3

asmSqrt:
	PUSH {R3}
	VSUB.F32 S20, S20, S20	//To get 0, we do S20 - S20
	VCMP.F32 S0, S20		//Compare it with 0
	VMRS APSR_nzvc, FPSCR	// load the FP PSR to branch using FP conditions
	BLE negative

squareRoot:
	VSQRT.F32 S0, S0
  	VSTR.f32  S0, [R3]

done:
	POP {R3}
	BX LR

negative:
	VSUB.F32 S0, S0, S0		//If the number is less than 0, return 0
	VSTR.f32  S0, [R3]
	POP {R3}
	BX LR
