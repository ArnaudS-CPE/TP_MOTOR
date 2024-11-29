/*
 * pid.c
 *
 *  Created on: Nov 22, 2024
 *      Author: arnaud
 */


#include "pid.h"
#include <math.h>


void PID_Init(PID_t* pidHandle, float Kp, float Ki, float Kd, float windup, float error_stop){

	 // Initialisation des coefficients PID
	 pidHandle->init.Kp = Kp;
	 pidHandle->init.Ki = Ki;
	 pidHandle->init.Kd = Kd;
	 pidHandle->init.integ_sat = windup;
	 pidHandle->init.error_stop = error_stop;

	 // Réinitialisation des variables de processus
	 pidHandle->process.error = 0.0f;
	 pidHandle->process.previous_error = 0.0f;
	 pidHandle->process.deriv = 0.0f;
	 pidHandle->process.integ = 0.0f;
	 pidHandle->process.output = 0.0f;
}


float PID_Execute(PID_t* pidHandle){
	 // error
	 pidHandle->process.error = pidHandle->input.order - pidHandle->input.feedback;

	 // derivate
	 pidHandle->process.deriv = pidHandle->process.error - pidHandle->process.previous_error;


	 // integer
	 if (fabsf(pidHandle->process.error) > pidHandle->init.error_stop) {
		 pidHandle->process.integ += pidHandle->process.error;
		 if (pidHandle->process.integ > pidHandle->init.integ_sat) {
			 pidHandle->process.integ = pidHandle->init.integ_sat;
		 } else if (pidHandle->process.integ < -pidHandle->init.integ_sat) {
			 pidHandle->process.integ = -pidHandle->init.integ_sat;
		 }
	 }

	 //command
	 pidHandle->process.output = pidHandle->init.Kp * pidHandle->process.error
			 	 	 	 	   + pidHandle->init.Ki * pidHandle->process.integ
							   - pidHandle->init.Kd * pidHandle->process.deriv;

	 pidHandle->process.previous_error = pidHandle->process.error;

	 return pidHandle->process.output;

}



