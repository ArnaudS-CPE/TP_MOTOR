/*
 * encoder.h
 *
 *  Created on: Nov 22, 2024
 *      Author: arnaud
 */

#ifndef ENCODER_H
#define ENCODER_H

typedef struct {
	float angle_rel;
	float angle_abs;
	float d_angle;
} Encoder_Feedback_t;

// public functions
Encoder_Feedback_t Encoder_Read (void);
void Encoder_Init (void);

#endif
