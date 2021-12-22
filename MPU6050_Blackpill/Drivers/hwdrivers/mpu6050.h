/*****************************************************
 * mpu6050.h
 *
 *  Created on: Dec 15, 2021
 *      Author: C.TASDEMIR
 ****************************************************/

#ifndef HWDRIVERS_MPU6050_H_
#define HWDRIVERS_MPU6050_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "sensordriver.h"

#define MPU6050_I2C_ADRESS_AD0 ( 0x68<<1 )
#define MPU6050_I2C_ADRESS_AD1 ( 0x69<<1 )

#define MPU_REG_SELF_TEST_X	( 13 )
#define MPU_REG_SELF_TEST_Y	( 14 )
#define MPU_REG_SELF_TEST_Z	( 15 )
#define MPU_REG_SELF_TEST_A	( 16 )
#define MPU_REG_SMPLRT_DIV	( 25 )
#define MPU_REG_CONFIG	( 26 )
#define MPU_REG_GYRO_CONFIG	( 27 )
#define MPU_REG_ACCEL_CONFIG	( 28 )
#define MPU_REG_ACCEL_XOUT_H	( 59 )
#define MPU_REG_ACCEL_XOUT_L	( 60 )
#define MPU_REG_ACCEL_YOUT_H	( 61 )
#define MPU_REG_ACCEL_YOUT_L	( 62 )
#define MPU_REG_ACCEL_ZOUT_H	( 63 )
#define MPU_REG_ACCEL_ZOUT_L	( 64 )
#define MPU_REG_ACCEL_TEMP_OUT_H ( 65 )
#define MPU_REG_ACCEL_TEMP_OUT_L	( 66 )
#define MPU_REG_GYRO_XOUT_H	( 67 )
#define MPU_REG_GYRO_XOUT_L	( 68 )
#define MPU_REG_GYRO_YOUT_H	( 69 )
#define MPU_REG_GYRO_YOUT_L	( 70 )
#define MPU_REG_GYRO_ZOUT_H	( 71 )
#define MPU_REG_GYRO_ZOUT_L	( 72 )
#define MPU_REG_USER_CTRL	( 106)
#define MPU_REG_PWR_MGMT_1	( 107 )
#define MPU_REG_PWR_MGMT_2	( 108 )
#define MPU_REG_FIFO_COUNTH ( 114 )
#define MPU_REG_FIFO_COUNTL ( 115 )
#define MPU_REG_FIFO_R_W	( 116 )
#define MPU_REG_WHO_AM_I	( 117 )

#define MPU_BIT_PWR_MGMT_1_SLEEP_MODE ( 6 )
#define MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION ( 3 )
#define MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION ( 3 )


typedef enum {
	FS_250, // 0
	FS_500, // 1
	FS_1000,// 2
	FS_2000,// 3
}fs_sel_e;

typedef enum {
	AFS_2G, //0
	AFS_4G, //1
	AFS_8G, //2
	AFS_16G //3
}afs_sel_e;

typedef enum {
	SLEEPMODE_OFF,
	SLEEPMODE_ON
}sleepmode_e;

typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
}AxisRawVal_t;

typedef struct {
	double X;
	double Y;
	double Z;
}AxisVal_t;

typedef struct {
	AxisRawVal_t accRaw;
	AxisRawVal_t gyroRaw;
	AxisVal_t acc;
	AxisVal_t gyro;
	double acc_co;
	double gyro_co;
}SensorData_t;



sensor_status_e MPU6050_initialize( SensorData_t *pSensor, fs_sel_e gyroConfig, afs_sel_e acc_config );
sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleemode);
sensor_status_e MPU6050_read_data(SensorData_t *pSensorData);
sensor_status_e MPU6050_test_sensor(void);
uint8_t MPU6050_read_id(void);

#endif /* HWDRIVERS_MPU6050_H_ */
