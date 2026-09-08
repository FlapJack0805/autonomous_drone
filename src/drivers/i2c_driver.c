
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "i2c_driver.h"
#include <stdio.h>

#define BUFFER_SIZE 20


struct i2c_transfer_info
{
	volatile i2c_phase_e phase;
	i2c_transfer_t transfer;

	uint16_t write_idx; //index for when we are iterating through write buffer
	uint16_t bytes_remaining;   // how many bytes left to read

	TaskHandle_t waiting_task;
};

static struct
{
	struct i2c_transfer_info i2c_messages[BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t num_i2c_messages;
} i2c_queue;

static struct i2c_transfer_info curr_i2c_transfer;
SemaphoreHandle_t i2c_semaphore;

static int push_i2c_queue(struct i2c_transfer_info *new_i2c_transfer)
{
	xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
	if (i2c_queue.num_i2c_messages == BUFFER_SIZE)
	{
		xSemaphoreGive(i2c_semaphore);
		return 0;
	}
	i2c_queue.i2c_messages[i2c_queue.head] = *new_i2c_transfer;

	i2c_queue.head = (i2c_queue.head + 1) % BUFFER_SIZE;
	++i2c_queue.num_i2c_messages;

	xSemaphoreGive(i2c_semaphore);
	return 1;
}




//NOTE: Check to make sure there is something in the queue before calling this
static struct i2c_transfer_info pop_i2c_queue(void)
{
	xSemaphoreTake(i2c_semaphore, portMAX_DELAY);
	struct i2c_transfer_info next_i2c_transfer = i2c_queue.i2c_messages[i2c_queue.tail];
	--i2c_queue.num_i2c_messages;
	i2c_queue.tail = (i2c_queue.tail + 1) % BUFFER_SIZE;

	xSemaphoreGive(i2c_semaphore);
	return next_i2c_transfer;
}


static bool i2c_queue_empty_isr(void)
{
	bool is_empty = i2c_queue.num_i2c_messages == 0;
	return is_empty;
}



void i2c_init(void)
{
	gpio_set_i2c(I2C1);
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
	curr_i2c_transfer.phase = I2C_IDLE;

	i2c_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(i2c_semaphore);
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


// This function is called to start the next i2c transfer
// It simply pops from the i2c queue, and sends the start condition
static void start_next_transfer(void)
{
	curr_i2c_transfer = pop_i2c_queue();
	send_start_condition();
}


int i2c_transfer(uint8_t addr7, const uint8_t *transfer_buf, uint16_t transfer_buf_len, uint8_t *recieve_buf, uint16_t recieve_buf_len)
{
	struct i2c_transfer_info new_i2c_transfer;
	new_i2c_transfer.transfer.addr7 = addr7;
	new_i2c_transfer.write_idx = 0;
	new_i2c_transfer.transfer.done = 0;
	new_i2c_transfer.bytes_remaining = recieve_buf_len;
	new_i2c_transfer.waiting_task = xTaskGetCurrentTaskHandle();

	if (recieve_buf_len != 0)
	{
		new_i2c_transfer.transfer.recieve_buf = recieve_buf;
		new_i2c_transfer.phase = I2C_PHASE_ADDR_RX;
	}

	else
	{
		new_i2c_transfer.transfer.recieve_buf = NULL;
	}

	if (transfer_buf_len != 0)
	{
		new_i2c_transfer.transfer.transfer_buf = transfer_buf;
		new_i2c_transfer.phase = I2C_PHASE_ADDR_TX;
	}

	else
	{
		new_i2c_transfer.transfer.transfer_buf = NULL;
	}

	if (recieve_buf_len == 0 && transfer_buf_len == 0)
	{
		new_i2c_transfer.transfer.done = 1;
		return 0;
	}

	new_i2c_transfer.transfer.transfer_buf_len = transfer_buf_len;
	new_i2c_transfer.transfer.recieve_buf_len = recieve_buf_len;

	if (push_i2c_queue(&new_i2c_transfer) == 0)
	{
		return I2C_BUFFER_FULL;
	}

	taskENTER_CRITICAL();
	if (curr_i2c_transfer.phase == I2C_IDLE)
	{
		start_next_transfer();
	}
	taskEXIT_CRITICAL();


	uint32_t transfer_result;

	if (xTaskNotifyWait(0, 0xFFFFFFFF, &transfer_result, pdMS_TO_TICKS(100)) == 0)
	{
		xTaskNotifyStateClear(NULL); // The ISR is still going while this is called so this will stop us from getting a late notification
		return I2C_TIMEOUT;
	}

	// At this point curr_i2c_transfer could have been overwritten so make sure to use new_i2c_transfer instead even though it's kind of unintuative
	if ((int32_t)transfer_result == -1)
	{
		return I2C_ERROR;
	}

	return I2C_OK;
}


static inline void end_transfer(void)
{
	TaskHandle_t task_to_wake = curr_i2c_transfer.waiting_task; // We need to do this because start_next_transfer will overwrite curr_i2c_transfer
	int finished_transfer_status = curr_i2c_transfer.transfer.done;

	if (!i2c_queue_empty_isr())
	{
		start_next_transfer();
	}

	BaseType_t woken_task = pdFALSE;
	xTaskNotifyFromISR(
	    task_to_wake,
	    (uint32_t)finished_transfer_status,
	    eSetValueWithOverwrite,
	    &woken_task
	);
	portYIELD_FROM_ISR(woken_task);
}



void I2C1_EV_IRQHandler(void)
{

	//start bit just sent, send address now
	if (I2C1->SR1 & I2C_SR1_SB)
	{
		//if (curr_i2c_transfer.transfer.transfer_buf_len > 0)
		if (curr_i2c_transfer.phase == I2C_PHASE_ADDR_TX)
		{
			send_addr(curr_i2c_transfer.transfer.addr7, 1);
		}

		else
		{
			if (curr_i2c_transfer.transfer.recieve_buf_len == 1)
			{
				I2C1->CR1 &= ~I2C_CR1_ACK; //turn off ack bit because there's only one byte to read in and it should already be in the line
			}
			else if (curr_i2c_transfer.transfer.recieve_buf_len == 2)
			{
				I2C1->CR1 |= I2C_CR1_POS; //make the ack bit stop after reading the next bit in instread of after the current one
				I2C1->CR1 &= ~I2C_CR1_ACK; //turn off ack bit because there's only one byte to read in and it should already be in the line
			}
			else
			{
				I2C1->CR1 |= I2C_CR1_ACK; //Turn ack bit on so we continue getting more bytes after the ones in the line
			}
			send_addr(curr_i2c_transfer.transfer.addr7, 0);
		}
		//return;
	}

	// addr bit just sen so now get pipeline reading to rx/tx data
	if (I2C1->SR1 & I2C_SR1_ADDR)
	{
		clear_addr();
		if (curr_i2c_transfer.phase == I2C_PHASE_ADDR_TX)
		{
			curr_i2c_transfer.phase = I2C_PHASE_TX;
			return;
		}

		if (curr_i2c_transfer.phase == I2C_PHASE_ADDR_RX)
		{
			if (curr_i2c_transfer.transfer.recieve_buf_len == 1)
			{
				curr_i2c_transfer.phase = I2C_PHASE_RX_RECIEVE_1;
				send_stop_condition(); // only byte has already been read in so we can stop
			}
			else if (curr_i2c_transfer.transfer.recieve_buf_len == 2)
			{
				curr_i2c_transfer.phase = I2C_PHASE_RX_RECIEVE_2;
			}
			// if 2 bytes in recieve buf do nothing right now and handle it below
			else
			{
				curr_i2c_transfer.phase = I2C_PHASE_RX;
			}
		}
		return;
	}

	if (curr_i2c_transfer.phase == I2C_PHASE_TX)
	{
		if (data_register_empty())
		{
			if (curr_i2c_transfer.transfer.transfer_buf_len > curr_i2c_transfer.write_idx && data_register_empty())
			{
				I2C1->DR = curr_i2c_transfer.transfer.transfer_buf[curr_i2c_transfer.write_idx];
				curr_i2c_transfer.write_idx++;
			}
		}

		if (byte_transfer_finished() && curr_i2c_transfer.write_idx == curr_i2c_transfer.transfer.transfer_buf_len)
		{
			if (curr_i2c_transfer.transfer.recieve_buf_len != 0)
			{
				curr_i2c_transfer.phase = I2C_PHASE_ADDR_RX;
				send_start_condition();
			}
			else
			{
				send_stop_condition();
				curr_i2c_transfer.phase = I2C_IDLE;
				curr_i2c_transfer.transfer.done = 1;

				end_transfer();
			}
		}
	}

	if (curr_i2c_transfer.phase == I2C_PHASE_RX)
	{
		while ((data_register_not_empty()) && (curr_i2c_transfer.bytes_remaining > 3))
		{
			*curr_i2c_transfer.transfer.recieve_buf = I2C1->DR;
			curr_i2c_transfer.transfer.recieve_buf++;
			curr_i2c_transfer.bytes_remaining--;
		}

		if ((curr_i2c_transfer.bytes_remaining == 3) && (byte_transfer_finished()))
		{
			I2C1->CR1 &= ~I2C_CR1_ACK;
			*curr_i2c_transfer.transfer.recieve_buf = I2C1->DR;
			curr_i2c_transfer.transfer.recieve_buf++;
			curr_i2c_transfer.bytes_remaining--;
			curr_i2c_transfer.phase = I2C_PHASE_RX_LAST2;
		}

	}

	if ((curr_i2c_transfer.phase == I2C_PHASE_RX_LAST2) && (byte_transfer_finished()))
	{
		send_stop_condition();

		// Recieve the first byte
		*curr_i2c_transfer.transfer.recieve_buf = I2C1->DR;
		curr_i2c_transfer.transfer.recieve_buf++;

		// Recieve the second byte
		*curr_i2c_transfer.transfer.recieve_buf = I2C1->DR;
		curr_i2c_transfer.transfer.recieve_buf++;

		I2C1->CR1 |= I2C_CR1_ACK;
		I2C1->CR1 &= ~I2C_CR1_POS;

		curr_i2c_transfer.transfer.done = 1;
		curr_i2c_transfer.phase = I2C_IDLE;

		end_transfer();
	}

	if (curr_i2c_transfer.phase == I2C_PHASE_RX_RECIEVE_2)
	{
		if (byte_transfer_finished())
		{
			send_stop_condition();

			// Recieve byte 1
			*curr_i2c_transfer.transfer.recieve_buf = I2C1->DR;
			++curr_i2c_transfer.transfer.recieve_buf;

			// Recieve byte 2
			*curr_i2c_transfer.transfer.recieve_buf = I2C1->DR;
			++curr_i2c_transfer.transfer.recieve_buf;

			I2C1->CR1 |= I2C_CR1_ACK;
			I2C1->CR1 &= ~I2C_CR1_POS;

			curr_i2c_transfer.transfer.done = 1;
			curr_i2c_transfer.phase = I2C_IDLE;

			end_transfer();
		}
	}

	if (curr_i2c_transfer.phase == I2C_PHASE_RX_RECIEVE_1)
	{
		if (data_register_not_empty())
		{
			*curr_i2c_transfer.transfer.recieve_buf = I2C1->DR;
			++curr_i2c_transfer.transfer.recieve_buf;
			I2C1->CR1 |= I2C_CR1_ACK;
			curr_i2c_transfer.transfer.done = 1;
			curr_i2c_transfer.phase = I2C_IDLE;

			end_transfer();
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
	curr_i2c_transfer.transfer.done = -1;
	curr_i2c_transfer.phase = I2C_IDLE;

	end_transfer();
}
