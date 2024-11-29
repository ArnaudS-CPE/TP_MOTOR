/*
 * motor.c
 *
 *  Created on: Nov 21, 2024
 *      Author: arnaud
 */

#include "motor.h"
#include "tim.h"
#include <stdio.h>
#include <math.h>

void Motor_Init(void){
	//printf("init_motor");
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	//htim1.Instance->CCR1 = htim1.Init.Period/2;
	//htim1.Instance->CCR2 = htim1.Init.Period/4;
}



void Motor_Pwm_Update(float in){

	if(in > 1.0){
		in = 1.0;
	}
	if(in < -1.0){
		in = -1.0;
	}

	uint32_t speed = htim1.Init.Period - (uint32_t)(fabs(in) * htim1.Init.Period);

	if(in > 0.0f){
		htim1.Instance->CCR1 = speed;
		htim1.Instance->CCR2 = htim1.Init.Period;
	}
	if(in < 0.0f){
		htim1.Instance->CCR2 = speed;
		htim1.Instance->CCR1 = htim1.Init.Period;
	}
	if(in == 0.0f){
		htim1.Instance->CCR1 = htim1.Init.Period;
		htim1.Instance->CCR2 = htim1.Init.Period;
	}
}




