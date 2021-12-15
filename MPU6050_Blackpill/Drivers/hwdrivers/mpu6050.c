/*
 * mpu6050.c
 *
 *  Created on: Dec 15, 2021
 *      Author: coskun
 */

#include "mpu6050.h"


extern I2C_HandleTypeDef hi2c1;


sensor_status_e MPU6050_TestSensor()
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_I2C_ADRESS_AD1, 4, 100);

	if ( HAL_OK == status ) {
		return SENSOR_OK;
	}
	else {
		return SENSOR_ERROR;
	}
}


sensor_status_e MPU6050_ReadID(void)
{
	int8_t buffer[1];
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADRESS_AD1, MPU_REG_WHO_AM_I, 1, buffer, 1, 100);

	if (HAL_OK != status) {
		//todo
	}

	if (0x68 == buffer[0]) {
		return SENSOR_OK;
	} else {
		return SENSOR_ERROR;
	}
}
