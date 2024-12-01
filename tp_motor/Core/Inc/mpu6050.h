/*
 * mpu6050.h
 *
 *  Created on: Oct 8, 2020
 *      Author: benjamin.blanchin
 */

#ifndef SRC_MPU6050_H_
#define SRC_MPU6050_H_

typedef struct Xyz {
	 float x;
	 float y;
	 float z;
} Xyz;

void MPU6050_Init (void);
Xyz MPU6050_Read_Accel (void);


#endif /* SRC_MPU6050_H_ */
