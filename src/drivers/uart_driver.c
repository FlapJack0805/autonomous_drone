#include "uart_driver.h"

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define BUFFER_CAPACITY 64

static uint8_t uart_buffer[BUFFER_CAPACITY];
static uint8_t buffer_head = 0;
static uint8_t buffer_tail = 0;
static uint8_t buffer_size = 0;


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


void _putchar(char c)
{
	while (buffer_size == BUFFER_CAPACITY);
	uint8_t next_idx = (buffer_head + 1) % BUFFER_CAPACITY;
	while (next_idx == buffer_tail); //wait until done sending current byte
	uart_buffer[buffer_head] = c;
	buffer_head = next_idx;
	buffer_size += 1;
	USART1->CR1 |= USART_CR1_TXEIE; //values added to buffer so turn on USART interrupt if not already on
	
	if (c == '\n')
	{
		_putchar('\r');
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
			uint8_t buffer_tail = (buffer_tail + 1) % BUFFER_CAPACITY;
			buffer_size -= 1;
		}

		else // buffer is empty so turn off the interrupt
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

