#include "uart_driver.h"

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define BUFFER_SIZE 64

static uint8_t uart_buffer[BUFFER_SIZE];
static uint8_t buffer_head;
static uint8_t buffer_tail;


/*
 * Turn on the USART1 clock
 * enable UART and TX
 * enable the USART1 interrupt
*/
void uart_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn);
}


static void write_byte(char c)
{
		uint8_t next_idx = (buffer_head + 1) % BUFFER_SIZE;
		while (next_idx == buffer_tail); //wait until done sending current byte
		uart_buffer[buffer_head] = c;
		buffer_head = next_idx;
		USART1->CR1 |= USART_CR1_TXEIE; //values added to buffer so turn on USART interrupt if not already on
}


/*
 * Writes len bytes into the uart_tx buffer which is implemented as a ring buffer
 * turns on the USART1 interrupt if it isn't already on because after this function
 * there is no chance the buffer is empty because we just added values to it
*/
void uart_write(uint8_t len, uint8_t *bytes)
{
	for (uint8_t i = 0; i < len; i++)
	{
		if (bytes[i] == '\n')
			write_byte('\r');

		write_byte(bytes[i]);
	}
}


/*
 *  read the FIFO value from the buffer and iterate the ring buffer values
 *  turns off the USART1 interrupt if the buffer is empty
*/
void USART1_IRQHandler(void)
{
	if (USART1->SR & USART_SR_TXE)
	{
		if (buffer_head != buffer_tail)
		{
			USART1->DR = uart_buffer[buffer_tail];
			uint8_t buffer_tail = (buffer_tail + 1) % BUFFER_SIZE;
		}

		else // buffer is empty so turn off the interrupt
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

