#include "main.h"
#include "mpu6050.h"
#include "i2c.h"

#define MPU6050_ADDR (0x68 << 1) // 0xD0 once shifted !
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
float Ax_bias = 0;
float Ay_bias = 0;
float Az_bias = 0;

void MPU6050_Init (void)
{
	uint8_t check = 0;
	uint8_t Data = 0;
	HAL_StatusTypeDef ret = 0;
	ret = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	if(ret != 0)
	{
		HAL_I2C_DeInit(&hi2c1); // Release IO port as GPIO, reset handle status flag
		HAL_I2C_Init(&hi2c1); //This sentence reinitializes the I2C controller
		HAL_Delay(100);
	}
	// check device ID WHO_AM_I
	ret = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	if (check >= 0x68 && check <= 0x73) // values of compatible MPU chips (6050,6500, 9250, 9255, ...)
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1,1000);
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1,1000);
		Xyz accel;
		accel = MPU6050_Read_Accel();
		Ax_bias = accel.x - 1;
		Ay_bias = accel.y;
		Az_bias = accel.z;
	}
}


Xyz MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6] = {0, 0, 0, 0, 0, 0};
	Xyz accel;
	HAL_StatusTypeDef ret = 0;
	int16_t Accel_X_RAW = 0;
	int16_t Accel_Y_RAW = 0;
	int16_t Accel_Z_RAW = 0;
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	ret = HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data,6, 1000);
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
	accel.x = ((float)Accel_X_RAW)/16384.0F - Ax_bias;
	accel.y = ((float)Accel_Y_RAW)/16384.0F - Ay_bias;
	accel.z = ((float)Accel_Z_RAW)/16384.0F - Az_bias;
	return accel;
}
