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

Encoder_Feedback_t enc = {0.0f, 0.0f, 0.0f}; //creation de la variable encoder

void Encoder_Init (void){ //initialisation de l'encodeur
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	htim3.Instance->CNT = 32767; //on fixe la valeur du compteur "au milieu" pour eviter les cas 0->65535 et 65535->0
}

Encoder_Feedback_t Encoder_Read (void){

	uint32_t val = htim3.Instance->CNT; //lecture de la valeur du compteur

	enc.angle_abs = enc.angle_abs + (float)((32767.0 - (float)val) * (float)M_PI * 2.0 / 24.0) / 75.0; //conversion en rad

	enc.angle_rel = enc.angle_abs; //angle [0;2*pi[]
	if(enc.angle_rel > 0.0){
		while(enc.angle_rel >= (float)(2*M_PI)){
			enc.angle_rel = enc.angle_rel - (float)(2*M_PI);};
	};
	if(enc.angle_rel < 0.0){
		while(enc.angle_rel <= 0.0){
			enc.angle_rel = enc.angle_rel + (float)(2*M_PI);};
	};

	float delta_angle = (enc.angle_abs - last_angle);
	enc.d_angle = delta_angle / 0.04; //vitesse de rotation en rad/s

	last_angle = enc.angle_abs;
	htim3.Instance->CNT = 32767; //reinitialisation du comteur a chaque lecture

	return enc;
}

