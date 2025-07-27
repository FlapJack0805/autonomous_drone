
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "i2c_driver.h"
#include <stdbool.h>



void i2c_init(void)
{
	// probably need to activate some specific clock
	I2C1->CR2 |= I2C_CR2_ITEVTEN;
	I2C1->CR2 |= I2C_CR2_ITBUFEN;
	I2C1->CR2 &= ~I2C_CR2_FREQ_Msk;
	I2C1->CR2 |= (50 & I2C_CR2_FREQ_Msk) << I2C_CR2_FREQ_Pos;
	I2C1->OAR1 &= ~I2C_OAR1_ADDMODE_Msk;
	I2C1->OAR2 &= ~I2C_OAR2_ENDUAL;
	NVIC_EnableIRQ(I2C1_EV_IRQn);
}


static inline void send_start_condition(void)
{
	I2C1->CR1 |= I2C_CR1_START;
}


static inline void send_stop_condition(void)
{
	I2C1->CR1 |= I2C_CR1_STOP;
}



static void i2c_send_addr(uint8_t addr, bool is_transmition)
{
	I2C1->DR = addr << 1;
	if (!is_transmition)
	{
		I2C1->DR |= 1;
	}
}


void i2c_write(uint8_t addr, uint8_t *data, uint8_t data_len)
{
	send_start_condition();
	i2c_send_addr(addr, true);

	for (uint8_t i = 0; i < data_len; i++)
	{
		I2C1->DR = data[i];
	}
}

void i2c_read(uint8_t *data, uint8_t len);


void I2C1_EV_IRQHandler(void)
{
	if (I2C1->SR1 & I2C_SR1_TXE) // transmitting data
	{

	}

	else if (I2C1->SR1 & I2C_SR1_RXNE) // recieving data
	{
	}
}
