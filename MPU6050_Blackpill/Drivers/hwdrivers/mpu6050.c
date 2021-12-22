/*
 * mpu6050.c
 *
 *  Created on: Dec 15, 2021
 *      Author: C
 */

#include "mpu6050.h"
#include "sensordriver.h"

static sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange);
static sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange);

sensor_status_e MPU6050_initialize( SensorData_t *pSensor, fs_sel_e gyroConfig, afs_sel_e acc_config )
{
    sensor_status_e retVal;

    retVal = MPU6050_set_gyro_range( pSensor, gyroConfig );

    if (retVal == SENSOR_OK) {
        retVal = MPU6050_set_acc_range( pSensor, acc_config );

        if (retVal == SENSOR_OK) {
            retVal = MPU6050_set_sleep_mode( SLEEPMODE_OFF );
        }
    }


    return retVal;
}

sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange)
{
	sensor_status_e retVal;
	uint8_t configReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_GYRO_CONFIG );

	configReg |= ((uint32_t)gyroRange << MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION );

	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_GYRO_CONFIG, configReg);

	switch (gyroRange)
	{
	 case FS_250:  pSensor->gyro_co = 131.0; break;
	 case FS_500:  pSensor->gyro_co = 65.5; break;
	 case FS_1000: pSensor->gyro_co = 32.8; break;
	 case FS_2000: pSensor->gyro_co = 16.4; break;
	 default: retVal = SENSOR_ERROR; break;
	}

	return retVal;
}

sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange)
{
	sensor_status_e retVal;
	uint8_t configReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_ACCEL_CONFIG );

	configReg |= ( (uint32_t) accRange << MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION );

	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_ACCEL_CONFIG, configReg);

	switch (accRange)
	{
	 case AFS_2G:  pSensor->acc_co = 16384; break;
	 case AFS_4G:  pSensor->acc_co = 8192; break;
	 case AFS_8G:  pSensor->acc_co = 4096; break;
	 case AFS_16G: pSensor->acc_co = 2048; break;
	 default: retVal = SENSOR_ERROR; break;
	}

	return retVal;
}

sensor_status_e MPU6050_test_sensor()
{
	sensor_status_e retVal;
	retVal = sensor_test_device(MPU6050_I2C_ADRESS_AD1);
	return retVal;
}


uint8_t MPU6050_read_id(void)
{
	uint8_t id = 0;
	id = sensor_read_register8(MPU6050_I2C_ADRESS_AD1,MPU_REG_WHO_AM_I );
	return id;
}

sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleepmode)
{
	sensor_status_e retVal;
	uint8_t powerReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_PWR_MGMT_1 );

	if(SLEEPMODE_ON == sleepmode) {
		SET_BIT(powerReg,1<<MPU_BIT_PWR_MGMT_1_SLEEP_MODE);
	}
	else {
		CLEAR_BIT(powerReg,1<<MPU_BIT_PWR_MGMT_1_SLEEP_MODE);
	}
	retVal = sensor_write_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_PWR_MGMT_1, powerReg);

	powerReg = sensor_read_register8(MPU6050_I2C_ADRESS_AD1, MPU_REG_PWR_MGMT_1 );

	return retVal;
}


sensor_status_e MPU6050_read_data(SensorData_t *pSensorData)
{
	sensor_status_e retVal;
	uint8_t buffer[14];
	retVal =  sensor_read_bytes(MPU6050_I2C_ADRESS_AD1, MPU_REG_ACCEL_XOUT_H, buffer, 14);

    if (retVal == SENSOR_OK) {
        pSensorData->accRaw.X = (int16_t) ((buffer[0] << 8) | buffer[1]);
        pSensorData->accRaw.Y = (int16_t) ((buffer[2] << 8) | buffer[3]);
        pSensorData->accRaw.Z = (int16_t) ((buffer[4] << 8) | buffer[5]);

        pSensorData->gyroRaw.X = (int16_t) ((buffer[8] << 8) | buffer[9]);
        pSensorData->gyroRaw.Y = (int16_t) ((buffer[10] << 8) | buffer[11]);
        pSensorData->gyroRaw.Z = (int16_t) ((buffer[12] << 8) | buffer[13]);

        pSensorData->acc.X = pSensorData->accRaw.X / pSensorData->acc_co;
        pSensorData->acc.Y = pSensorData->accRaw.Y / pSensorData->acc_co;
        pSensorData->acc.Z = pSensorData->accRaw.Z / pSensorData->acc_co;

        pSensorData->gyro.X = pSensorData->gyroRaw.X / pSensorData->gyro_co;
        pSensorData->gyro.Y = pSensorData->gyroRaw.Y / pSensorData->gyro_co;
        pSensorData->gyro.Z = pSensorData->gyroRaw.Z / pSensorData->gyro_co;

        return retVal;
    } else {
        return SENSOR_ERROR;
    }
}


