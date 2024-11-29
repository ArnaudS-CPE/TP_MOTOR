/*
 * encoder.c
 *
 *  Created on: Nov 22, 2024
 *      Author: arnaud
 */

#include "encoder.h"
#include "tim.h"
#include <stdio.h>
#include <math.h>

static float last_angle = 0;

Encoder_Feedback_t enc = {0.0f, 0.0f, 0.0f};



void Encoder_Init (void){
	//printf("init_encoder");
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	htim3.Instance->CNT = 32767;
}




Encoder_Feedback_t Encoder_Read (void){

	uint32_t val = htim3.Instance->CNT;

	enc.angle_abs = enc.angle_abs + (float)((32767.0 - (float)val) * (float)M_PI * 2.0 / 24.0) / 75.0;

	enc.angle_rel = enc.angle_abs;
	if(enc.angle_rel > 0.0){
		while(enc.angle_rel >= (float)(2*M_PI)){
			enc.angle_rel = enc.angle_rel - (float)(2*M_PI);
		};
	};
	if(enc.angle_rel < 0.0){
		while(enc.angle_rel <= 0.0){
			enc.angle_rel = enc.angle_rel + (float)(2*M_PI);
		};
	};

	float delta_angle = (enc.angle_abs - last_angle);

	enc.d_angle = delta_angle / 0.04;

	last_angle = enc.angle_abs;
	htim3.Instance->CNT = 32767;

	return enc;
}


