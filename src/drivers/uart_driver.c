#include "uart_driver.h"

#define BUFFER_CAPACITY 64

volatile static uint8_t uart_buffer[BUFFER_CAPACITY];
volatile static uint8_t buffer_head = 0;
volatile static uint8_t buffer_tail = 0;
volatile static uint8_t buffer_size = 0;





/*
 * Turn on the USART2 clock
 * enable UART and TX
 * enable the USART1 interrupt
*/
void uart_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->CR1 = 0;
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	// For USART1 (APB2=84 MHz)
	USART1->BRR = 0x008B;  // 115200 baud
	USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
	NVIC_SetPriority(USART1_IRQn, 5);
	NVIC_EnableIRQ(USART1_IRQn);
}


void _putchar(char c, USART_TypeDef *USART)
{
	if (c == '\n')
	{
		_putchar('\r', USART);
	}


	// Fast-path: if transmitter idle and buffer empty, write directly
	if (buffer_size == 0 && (USART->SR & USART_SR_TXE)) 
	{
		USART->DR = (uint8_t)c;
		return;
	}


	while (buffer_size == BUFFER_CAPACITY);
	uint8_t next_idx = (buffer_head + 1) % BUFFER_CAPACITY;
	uart_buffer[buffer_head] = c;
	buffer_head = next_idx;
	buffer_size += 1;
	
	USART->CR1 |= USART_CR1_TXEIE;
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
			buffer_tail = (buffer_tail + 1) % BUFFER_CAPACITY;
			buffer_size -= 1;
		}

		else // buffer is empty so turn off the interrupt
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}



/*
void USART2_IRQHandler(void)
{
	if (USART2->SR & USART_SR_TXE)
	{
		if (buffer_head != buffer_tail)
		{
			USART2->DR = uart_buffer[buffer_tail];
			buffer_tail = (buffer_tail + 1) % BUFFER_CAPACITY;
			buffer_size -= 1;
		}

		else // buffer is empty so turn off the interrupt
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;
		}
	}

}
*/
