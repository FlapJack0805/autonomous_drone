#include "mpu_6050.h"

static void imu_bus_read(uint8_t reg, uint8_t *data, uint32_t len)
{
	i2c_transfer(IMU_ADDRESS, &reg, 1, data, len);
}


static void imu_bus_write(uint8_t reg, uint8_t data)
{
	uint8_t message[2] = {reg, data}; // a byte for every data byte and 1 for the reg
	
	i2c_transfer(IMU_ADDRESS, message, 2, 0, 0);
}

bool imu_init(void)
{
	uint8_t who;

	// Read in the WHO_AM_I value
	imu_bus_read(WHO_AM_I, &who, 1);

	// Check to make sure we have the right address
	/*
	if (who != IMU_ADDRESS)
	{
		return false;
	}
	*/

	// Reset the imu
	imu_bus_write(WR_MGMT_1, 1 << 7);
	
	vTaskDelay(pdMS_TO_TICKS(10));

	//PLL X gyro
	imu_bus_write(WR_MGMT_1, 0x01);

	//DLPF 44 Hz
	imu_bus_write(CONFIG, 0x03);

	//1 KHz
	imu_bus_write(SMPLRT_DIV, 0x00);

	// +- 2000 dps
	imu_bus_write(GYRO_CONFIG, 0x18);

	//+= 16g
	imu_bus_write(ACCEL_CONFIG, 0x18);

	vTaskDelay(100);

	return true;
}



void imu_read(struct imu_data *data)
{
	uint8_t buf[14]; // extra 2 bytes for temperature reading which we ignore
	imu_bus_read(ACCEL_XOUT_H, buf, 14);

	// most significant byte is read in first
	int16_t ax = (buf[0] << 8) | buf[1];
	int16_t ay = (buf[2] << 8) | buf[3];
	int16_t az = (buf[4] << 8) | buf[5];

	int16_t gx = (buf[8] << 8) | buf[9];
	int16_t gy = (buf[10] << 8) | buf[11];
	int16_t gz = (buf[12] << 8) | buf[13];

	data->accel_x = ax / 2048.0f;
	data->accel_y = ay / 2048.0f;
	data->accel_z = az / 2048.0f;

	data->gyro_x = gx / 2048.0f;
	data->gyro_y = gy / 2048.0f;
	data->gyro_z = gz / 2048.0f;
}
