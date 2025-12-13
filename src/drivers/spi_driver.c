#include "spi_driver.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"


static struct 
{
    spi_transfer_t transfer;

    uint16_t write_idx; //index for when we are iterating through write buffer
    uint16_t recieve_bytes_remaining;   // how many bytes left to read
} spi2_transfer_info;


void spi_init()
{
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; //might change which clock I enable based on which lines I use for SPI
	SPI2->CR1 &= ~SPI_CR1_SPE;
	SPI2->CR1 = 0;
	SPI2->CR1 |= SPI_CR1_MSTR;
	SPI2->CR1 &= ~SPI_CR1_BIDIMODE;
	SPI2->CR1 &= ~SPI_CR1_DFF;
	SPI2->CR1 &= ~SPI_CR1_RXONLY;
	SPI2->CR1 &= ~SPI_CR1_LSBFIRST;
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
	SPI2->CR1 |= (0b111 << SPI_CR1_BR_Pos); // 16MHz / 16 = 1MHz
	SPI2->CR2 = 0;
	SPI2->CR2 |= SPI_CR2_ERRIE;
	(void)SPI2->DR; (void)SPI2->SR; //clear flags
	NVIC_EnableIRQ(SPI2_IRQn);
	SPI2->CR1 |= SPI_CR1_SPE;
}

static inline int dr_ready_for_read(void)
{
	return SPI2->SR & SPI_SR_RXNE;
}


static inline int dr_ready_for_transfer(void)
{
	return SPI2->SR & SPI_SR_TXE;
}

static inline int spi_busy(void)
{
	return SPI2->SR & SPI_SR_BSY;
}

static inline void clear_ovr_if_set(void)
{
	if (SPI2->SR & SPI_SR_OVR)
	{
		(void)SPI2->DR;
		(void)SPI2->SR;
	}
}


int spi_transfer(const uint8_t *transfer_buf, uint16_t transfer_buf_len, uint8_t *recieve_buf, uint16_t recieve_buf_len)
{
	spi2_transfer_info.write_idx = 0;
	spi2_transfer_info.recieve_bytes_remaining = recieve_buf_len;
	spi2_transfer_info.transfer.done = 0;

	spi2_transfer_info.transfer.transfer_buf_len = transfer_buf_len;
	spi2_transfer_info.transfer.recieve_buf_len = recieve_buf_len;

	if (transfer_buf_len > 0)
	{
		spi2_transfer_info.transfer.transfer_buf = transfer_buf;
	}

	if (recieve_buf_len > 0)
	{
		SPI2->CR2 |= SPI_CR2_RXNEIE;
		spi2_transfer_info.transfer.recieve_buf = recieve_buf;
	}
	SPI2->CR2 |= SPI_CR2_TXEIE;

	//TODO: Put logic below to address the device we are communicating with
	// I believe we need to pull down the CS line connected to said device but
	// we can deal with that once we have our devices connected to the stm32
	// and know which lines we need to pull down
	
	return 1;
}


void SPI2_IRQHandler(void)
{
	if (SPI2->SR & SPI_SR_OVR)
	{
		clear_ovr_if_set();
	}

	if (SPI2->SR & (SPI_SR_OVR | SPI_SR_MODF))
	{
		(void)SPI2->SR;
		SPI2->CR1 |= SPI_CR1_SSI;   // keep NSS internal high
		SPI2->CR1 |= SPI_CR1_SPE;   // SPI disabled on MODF: re-enable
		spi2_transfer_info.transfer.done = -1;
	}

	while (dr_ready_for_read())
	{
		uint8_t recieved_byte = (uint8_t)SPI2->DR;
		if (spi2_transfer_info.recieve_bytes_remaining > 0)
		{
			*spi2_transfer_info.transfer.recieve_buf = recieved_byte;
			spi2_transfer_info.transfer.recieve_buf++;
			spi2_transfer_info.recieve_bytes_remaining--;
		}
	}

	while (dr_ready_for_transfer())
	{
		if (spi2_transfer_info.write_idx < spi2_transfer_info.transfer.transfer_buf_len)
		{
			SPI2->DR = spi2_transfer_info.transfer.transfer_buf[spi2_transfer_info.write_idx++];
		}
		else if (spi2_transfer_info.recieve_bytes_remaining > 0)
		{
			SPI2->DR = 0xFF; // insert dummy data once transfer is done
		}
		else
		{
			SPI2->CR2 &= ~SPI_CR2_TXEIE;
			break; //nothing left to clock
		}
	}

	if (spi2_transfer_info.recieve_bytes_remaining == 0 && spi2_transfer_info.write_idx >= spi2_transfer_info.transfer.transfer_buf_len && !spi_busy())
	{
		SPI2->CR2 &= ~SPI_CR2_RXNEIE;
		SPI2->CR2 &= ~SPI_CR2_TXEIE;
		spi2_transfer_info.transfer.done = 1;
	}
}

