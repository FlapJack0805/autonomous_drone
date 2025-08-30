
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "i2c_driver.h"
#include <stdio.h>


static struct 
{
    volatile i2c_phase_e phase;
    i2c_transfer_t transfer;

    uint16_t write_idx; //index for when we are iterating through write buffer
    uint16_t bytes_remaining;   // how many bytes left to read
} i2c1_transfer_info;


void i2c_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR2 |= I2C_CR2_ITEVTEN;
	I2C1->CR2 |= I2C_CR2_ITBUFEN;
	I2C1->CR2 |= I2C_CR2_ITERREN;
	I2C1->CR2 &= ~I2C_CR2_FREQ_Msk;
	I2C1->CR2 |= 42 << I2C_CR2_FREQ_Pos;
	I2C1->CCR   = 210; // 100 kHz: 42e6/(2*100k)=210
	I2C1->TRISE = 43;  // FREQ+1 in std mode
	I2C1->OAR1  = (0x32u << 1); // any 7-bit own addr (master still needs valid)
	I2C1->OAR1 &= ~I2C_OAR1_ADDMODE_Msk;
	I2C1->OAR2 &= ~I2C_OAR2_ENDUAL;
	I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	i2c1_transfer_info.phase = I2C_IDLE;
}


static inline void send_start_condition(void)
{
	I2C1->CR1 |= I2C_CR1_START;
}


static inline void send_stop_condition(void)
{
	I2C1->CR1 |= I2C_CR1_STOP;
}


static inline int byte_transfer_finished(void)
{
	return I2C1->SR1 & I2C_SR1_BTF;
}


static inline int data_register_empty(void)
{
	return I2C1->SR1 & I2C_SR1_TXE;
}


static inline int data_register_not_empty(void)
{
	return I2C1->SR1 & I2C_SR1_RXNE;
}

static inline void clear_addr(void) 
{ 
	(void)I2C1->SR1; 
	(void)I2C1->SR2; 
}



/*
	* writes the 7 bit address to the DR shifted one to the right so it starts
	* at the MSB and then writes a 1 to the the LSB of DR if it is recieving
*/
static void send_addr(uint8_t addr, int is_transmition)
{
	(void)I2C1->SR1;
	I2C1->DR = (addr << 1) | ((is_transmition) ? 0u : 1u);
}


/*
 * i2c_write and i2c_read were made for polling
 * I commented them out once I designed an interrupt implementation
void i2c_write(uint8_t addr, uint8_t *data, uint8_t data_len)
{
	send_start_condition();
	i2c_send_addr(addr, true); // true in second argument states tx

	for (uint16_t i = 0; i < data_len; i++)
	{
		I2C1->DR = data[i];
	}

	send_stop_condition();
}

void i2c_read(uint8_t addr, uint8_t *data, uint8_t data_len)
{
	send_start_condition();
	i2c_send_addr(addr, false); // false in second argument states rx
	
	for (uint16_t i = 0; i < data_len; i++)
	{
		data[i] = I2C1->DR;
	}

	send_stop_condition();
}
*/


int i2c_transfer(uint8_t addr7, const uint8_t *transfer_buf, uint16_t transfer_buf_len, uint8_t *recieve_buf, uint16_t recieve_buf_len)
{
	i2c1_transfer_info.transfer.addr7 = addr7;
	i2c1_transfer_info.write_idx = 0;
	i2c1_transfer_info.transfer.done = 0;
	i2c1_transfer_info.bytes_remaining = recieve_buf_len;

	if (recieve_buf_len != 0)
	{
		i2c1_transfer_info.transfer.recieve_buf = recieve_buf;
		i2c1_transfer_info.phase = I2C_PHASE_ADDR_RX;
	}
	else
	{
		i2c1_transfer_info.transfer.recieve_buf = NULL;
	}

	if (transfer_buf_len != 0)
	{
		i2c1_transfer_info.transfer.transfer_buf = transfer_buf;
		i2c1_transfer_info.phase = I2C_PHASE_ADDR_TX;
	}
	else
	{
		i2c1_transfer_info.transfer.transfer_buf = NULL;
	}

	if (recieve_buf_len == 0 && transfer_buf_len == 0)
	{
		i2c1_transfer_info.transfer.done = 1;
		return 0;
	}

	i2c1_transfer_info.transfer.transfer_buf_len = transfer_buf_len;
	i2c1_transfer_info.transfer.recieve_buf_len = recieve_buf_len;

	send_start_condition();
	return 0;
}



void I2C1_EV_IRQHandler(void)
{

	//start bit just sent, send address now
	if (I2C1->SR1 & I2C_SR1_SB)
	{
		if (i2c1_transfer_info.phase == I2C_PHASE_ADDR_TX)
		{
			send_addr(i2c1_transfer_info.transfer.addr7, 1);
		}

		else
		{
			if (i2c1_transfer_info.transfer.recieve_buf_len == 1)
			{
				I2C1->CR1 &= ~I2C_CR1_ACK; //turn off ack bit because there's only one byte to read in and it should already be in the line
			}
			else if (i2c1_transfer_info.transfer.recieve_buf_len == 2)
			{
				I2C1->CR1 |= I2C_CR1_POS; //make the ack bit stop after reading the next bit in instread of after the current one
				I2C1->CR1 &= ~I2C_CR1_ACK; //turn off ack bit because there's only one byte to read in and it should already be in the line
			}
			else
			{
				I2C1->CR1 |= I2C_CR1_ACK; //Turn ack bit on so we continue getting more bytes after the ones in the line
			}
			send_addr(i2c1_transfer_info.transfer.addr7, 0);
		}
		return;
	}

	// addr bit just sen so now get pipeline reading to rx/tx data
	if (I2C1->SR1 & I2C_SR1_ADDR)
	{
		clear_addr();
		if (i2c1_transfer_info.phase == I2C_PHASE_ADDR_TX)
		{
			i2c1_transfer_info.phase = I2C_PHASE_TX;
			return;
		}

		if (i2c1_transfer_info.phase == I2C_PHASE_ADDR_RX)
		{
			if (i2c1_transfer_info.transfer.recieve_buf_len == 1)
			{
				i2c1_transfer_info.phase = I2C_PHASE_RX_LAST1;
				send_stop_condition(); // only byte has already been read in so we can stop
			}
			else if (i2c1_transfer_info.transfer.recieve_buf_len == 2)
			{
				i2c1_transfer_info.phase = I2C_PHASE_RX_LAST2;
			}
			// if 2 bytes in recieve buf do nothing right now and handle it below
			else
			{
				i2c1_transfer_info.phase = I2C_PHASE_RX;
			}
		}
		return;
	}

	if (i2c1_transfer_info.phase == I2C_PHASE_TX)
	{
		if (data_register_empty())
		{
			if (i2c1_transfer_info.transfer.transfer_buf_len > i2c1_transfer_info.write_idx && data_register_empty())
			{
				I2C1->DR = i2c1_transfer_info.transfer.transfer_buf[i2c1_transfer_info.write_idx];
				i2c1_transfer_info.write_idx++;
			}
		}

		if (byte_transfer_finished())
		{
			if (i2c1_transfer_info.transfer.recieve_buf_len != 0)
			{
				i2c1_transfer_info.phase = I2C_PHASE_ADDR_RX;
				send_start_condition();
			}
			else
			{
				send_stop_condition();
				i2c1_transfer_info.phase = I2C_IDLE;
				i2c1_transfer_info.transfer.done = 1;
			}
		}
	}

	if (i2c1_transfer_info.phase == I2C_PHASE_RX)
	{
		while ((data_register_not_empty()) && (i2c1_transfer_info.bytes_remaining > 3))
		{
			*i2c1_transfer_info.transfer.recieve_buf = I2C1->DR;
			i2c1_transfer_info.transfer.recieve_buf++;
			i2c1_transfer_info.bytes_remaining--;
		}

		if ((i2c1_transfer_info.bytes_remaining == 3) && (byte_transfer_finished()))
		{
			I2C1->CR1 &= ~I2C_CR1_ACK;
			*i2c1_transfer_info.transfer.recieve_buf = I2C1->DR;
			i2c1_transfer_info.transfer.recieve_buf++;
			i2c1_transfer_info.bytes_remaining--;
			send_stop_condition();
			i2c1_transfer_info.phase = I2C_PHASE_RX_LAST2;
		}

	}

	if ((i2c1_transfer_info.phase == I2C_PHASE_RX_LAST2) && (byte_transfer_finished()))
	{
		if (!byte_transfer_finished())
		{
			return;
		}
		send_stop_condition();
		if (data_register_not_empty())
		{
			*i2c1_transfer_info.transfer.recieve_buf = I2C1->DR;
			i2c1_transfer_info.transfer.recieve_buf++;
			if (data_register_not_empty())
			{
				*i2c1_transfer_info.transfer.recieve_buf = I2C1->DR;
				i2c1_transfer_info.transfer.recieve_buf++;
				I2C1->CR1 |= I2C_CR1_ACK;
				I2C1->CR1 &= ~I2C_CR1_POS;
				i2c1_transfer_info.transfer.done = 1;
				i2c1_transfer_info.phase = I2C_IDLE;
			}
		}
	}

	if (i2c1_transfer_info.phase == I2C_PHASE_RX_LAST1)
	{
		if (data_register_not_empty())
		{
			*i2c1_transfer_info.transfer.recieve_buf = I2C1->DR;
			i2c1_transfer_info.transfer.recieve_buf++;
			I2C1->CR1 |= I2C_CR1_ACK;
			i2c1_transfer_info.transfer.done = 1;
			i2c1_transfer_info.phase = I2C_IDLE;
			return;
		}
	}

}



void I2C1_ER_IRQHandler(void)
{
	uint32_t sr1 = I2C1->SR1;
	if (sr1 & I2C_SR1_BERR)   I2C1->SR1 &= ~I2C_SR1_BERR;
	if (sr1 & I2C_SR1_ARLO)   I2C1->SR1 &= ~I2C_SR1_ARLO;
	if (sr1 & I2C_SR1_AF)     I2C1->SR1 &= ~I2C_SR1_AF;     // NACK
	if (sr1 & I2C_SR1_OVR)    I2C1->SR1 &= ~I2C_SR1_OVR;
	if (sr1 & I2C_SR1_TIMEOUT)I2C1->SR1 &= ~I2C_SR1_TIMEOUT;

	send_stop_condition();
	i2c1_transfer_info.transfer.done = -1;
	i2c1_transfer_info.phase = I2C_IDLE;
}
