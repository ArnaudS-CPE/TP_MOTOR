/*
 * DriveSyst.h
 *
 *  Created on: Nov 28, 2024
 *      Author: arnaud
 */

#ifndef INC_DRIVESYST_H_
#define INC_DRIVESYST_H_

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "mpu6050.h"

#include "gpio.h"
#include <stdio.h>
#include <math.h>

// public functions
void DriveSyst (void);

#endif /* INC_DRIVESYST_H_ */
