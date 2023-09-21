/*
 * asmtranscendental.s
 *
 *  Created on: Sep 12, 2023
 *      Author: karl
 */


// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmSqrt, which is expected by lab1math.h
.global asmTranscendental

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata




/**
 * float asmTranscendental(float x0, float w, float phi);
 * S0 -> X0
 * S1 -> w
 * S2 -> phi
 */


asmTranscendental:

PUSH {V8}

//Set S20 to be the tolerance and epsilon at 0.015625
VMOV S16, #4
VMOV S20, #1
VDIV.F32 S20, S20, S16
VDIV.F32 S20, S20, S16
VDIV.F32 S20, S20, S16

//Set V8 to be the max iterations
MOV V8, #50

loop:
//Check if it exceded the max iterations
CMP V8, #0
SUB V8, V8, #1
BEQ end

PUSH {LR}
BL defunc
POP {LR}

B loop

end:
POP {V8}
BX LR

break:
POP {LR}
POP {V8}
BX LR


defunc:

VMUL.F32 S3, S1, S0				//B = x0*w
VADD.F32 S3, S3, S2				//B = B + phi

VMOV S31, S3			//Store B into S31

VMOV S30, S0			//Store X0 into S30

VMOV S0, S31			//Load B into S0

//Call ArmCosFunction with S0 as argument
PUSH {LR}
BL arm_cos_f32
POP {LR}

VMOV S5, S0					//Store cos(B) into S5

VMOV S0, S31				//Reload B into S0

//Call ArmSinFunction with S0 as argument
PUSH {LR}
BL arm_sin_f32
POP {LR}

VMOV S7, S0				//Store sin(B) into S7

VMOV S0, S30			//Load X0 from S30

VMOV S4, S0				//Set X0 into S4


/**
 * f(x) = x^2 - cos(wx+φ) -> B = wx+φ
 */

VMUL.F32 S4, S4, S4		//X0*X0
VSUB.F32 S4, S4, S5		//X0^2 - cos(B) (This is f(x))


/**
 * f'(x) = 2x + wsin(wx+φ) -> B = wx+φ
 */

VMUL.F32 S6, S1, S7		//w*sin(b)
VMOV S8, S0				//Store X0 into S8
VMOV S9, #2				//Store 2 into S9
VMUL.F32 S8, S8, S9		//2*X0
VADD.F32 S8, S8, S6		//2*X0 + w*sin(b) (This is f'(x))


/**
* 	if(fabs(f'(x)) < epsilon)
*		break;
*
*/
VABS.F32 S21, S8		//Get |f'(x)|
VCMP.F32 S21, S20		//Compare it with epsilon
VMRS APSR_nzvc, FPSCR	// load the FP PSR to branch using FP conditions
BLT break				//If it's less than, we break out of the loop


/**
* x1 = x0 -  f(x)/f'(x)
*/

VDIV.F32 S9, S4, S8		//f(x)/f'(x)
VSUB.F32 S11, S0, S9 	//x0 - f(x)/f'(x)


/**
*	if(fabs(x1-x0) <= tolerance)
*		return x1;
*
*/

VSUB.F32 S12, S11, S0	//Get x1 - x0
VABS.F32 S12, S12		//Get |x1 - x0|
VCMP.F32 S12, S20		//Compare the difference with tolerance
VMRS APSR_nzvc, FPSCR	// load the FP PSR to branch using FP conditions
BLE returnX1			//If it's less than or equal, we return x1 and break out of the loop

VMOV S0, S11			//x0 = x1
BX LR


returnX1:
VMOV S0, S11			//x0 = x1
B break

