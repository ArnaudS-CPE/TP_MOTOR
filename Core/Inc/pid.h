/*
 * pid.h
 *
 *  Created on: Nov 22, 2024
 *      Author: arnaud
 */

#ifndef PID_H
#define PID_H

typedef struct {
	struct {
		float Kp; // Proportional coefficient
		float Ki; // Integral coefficient
		float Kd; // Derivative coefficient
		float integ_sat; // Windup control with integral saturation
		// so that abs(integ) <= abs(integ_sat)
		float error_stop; // Error below which integration stops
	} init;

	struct {
		float order; // order (what you want to reach)
		float feedback; // what you reached for now
	} input;

	struct {
		float error; // order - feedback
		float previous_error; // previous error... ^^
		float deriv; // i.e. output
		float integ; // i.e. output
		float output; // output = Ki*error + Ki*integ + Kd*deriv
	} process;
} PID_t;

// public functions
void PID_Init(PID_t* pidHandle, float Kp, float Ki, float Kd, float windup, float error_stop);
float PID_Execute(PID_t* pidHandle);
#endif
