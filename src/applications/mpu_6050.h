#ifndef IMU_H
#define IMU_H

#include "gpio_driver.h"
#include "i2c_driver.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "FreeRTOSIncludes.h"

#define IMU_ADDRESS 0x68

#define WHO_AM_I 0x75
#define ACCEL_XOUT_H 0x3B // all of the other information we read in follows this register
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define SMPLRT_DIV 0x19
#define WR_MGMT_1 0x6B


struct imu_data
{
	float accel_x, accel_y, accel_z;
	float gyro_x, gyro_y, gyro_z;
};


bool imu_init(void);
void imu_read(struct imu_data *data);

#endif //IMU_H
