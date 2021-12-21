/*
 * sensordriver.c
 *
 *  Created on: Dec 16, 2021
 *      Author: C.TASDEMIR
 */

#include "sensordriver.h"
#include "stm32f4xx_hal.h"

#define I2C_TIMEOUT 100
#define I2C_NUMBER_OF_TRIALS 4

extern I2C_HandleTypeDef hi2c1;


sensor_status_e sensor_test_device( uint8_t chipAdd )
{
	HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady( &hi2c1, chipAdd, I2C_NUMBER_OF_TRIALS, I2C_TIMEOUT );

	if ( HAL_OK == status ) {
		return SENSOR_OK;
	}
	else {
		return SENSOR_ERROR;
	}
}

uint8_t sensor_read_register8(uint8_t chipAdd, uint8_t regAdd)
{
	uint8_t data;
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read( &hi2c1, chipAdd, regAdd, 1, &data, 1, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return 0;
	} else {
		return data;
	}
}

uint16_t sensor_read_register16(uint8_t chipAdd, uint8_t regAdd)
{
	uint8_t data[2];
	uint16_t retVal;
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read( &hi2c1, chipAdd, regAdd, 1, data, 2, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return 0;
	} else {
		retVal = (data[1]<<8) | data[0];
		return retVal;
	}
}

sensor_status_e sensor_write_register8(uint8_t chipAdd, uint8_t regAdd, uint8_t value)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write( &hi2c1, chipAdd, regAdd, 1, &value, 1, I2C_TIMEOUT );

    if (HAL_OK != status) {
        return SENSOR_ERROR;
    } else {
        return SENSOR_OK;
    }
}

sensor_status_e sensor_write_register16(uint8_t chipAdd, uint8_t regAdd, uint16_t value)
{
	uint8_t data[2];
	HAL_StatusTypeDef status;

	data[0] = (uint8_t) (value | 0xFF);
    data[1] = (uint8_t) ((value >> 8) | 0xFF);
    status = HAL_I2C_Mem_Write( &hi2c1, chipAdd, regAdd, 1, data, 2, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}


sensor_status_e sensor_read_bytes(uint8_t chipAdd, uint8_t regAdd, uint8_t *pBuffer, uint8_t size)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, chipAdd, regAdd, 1, pBuffer, size, I2C_TIMEOUT);

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}

sensor_status_e sensor_write_bytes(uint8_t chipAdd, uint8_t regAdd, uint8_t *pBuffer, uint8_t size)
{
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write( &hi2c1, chipAdd, regAdd, 1, pBuffer, size, I2C_TIMEOUT );

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}
