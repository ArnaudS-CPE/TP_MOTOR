/*
 * DriveSyst.c
 *
 *  Created on: Nov 28, 2024
 *      Author: arnaud
 */

#include "DriveSyst.h"

PID_t pid;
Encoder_Feedback_t encoder;
Xyz acc;

int mode;


void DriveSyst(void){
	encoder = Encoder_Read();

	switch(mode){
		case 0: //vitesse
			pid.input.feedback = encoder.d_angle;
			break;
		case 1: //position
			pid.input.feedback = encoder.angle_abs;
			break;
		case 2: //accelerometre
			acc = MPU6050_Read_Accel();
			pid.input.feedback = encoder.angle_abs;
			//pid.input.order = M_PI * acc.y + M_PI;
			pid.input.order = atan2(acc.y, acc.z) + (M_PI/2);
			break;
	}

	float consigne = PID_Execute(&pid);
	Motor_Pwm_Update(consigne);
}



