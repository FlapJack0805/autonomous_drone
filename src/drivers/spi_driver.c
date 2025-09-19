#include "spi_driver.h"
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"


static struct 
{
    spi_transfer_t transfer;

    uint16_t write_idx; //index for when we are iterating through write buffer
    uint16_t recieve_bytes_remaining;   // how many bytes left to read
} spi1_transfer_info;


void spi_init()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //might change which clock I enable based on which lines I use for SPI
	SPI1->CR1 &= ~SPI_CR1_SPE;
	SPI1->CR1 = 0;
	SPI1->CR1 |= SPI_CR1_MSTR;
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	SPI1->CR1 &= ~SPI_CR1_DFF;
	SPI1->CR1 &= ~SPI_CR1_RXONLY;
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software NSS management: keep NSS internally high to avoid MODF
	SPI1->CR1 |= (0b011 << SPI_CR1_BR_Pos);
	SPI1->CR2 = 0;
	SPI1->CR2 |= SPI_CR2_ERRIE;
	(void)SPI1->DR; (void)SPI1->SR; //clear flags
	NVIC_EnableIRQ(SPI1_IRQn);
	SPI1->CR1 |= SPI_CR1_SPE;
}

static inline int dr_ready_for_read(void)
{
	return SPI1->SR & SPI_SR_RXNE;
}


static inline int dr_ready_for_transfer(void)
{
	return SPI1->SR & SPI_SR_TXE;
}

static inline int spi_busy(void)
{
	return SPI1->SR & SPI_SR_BSY;
}

static inline void clear_ovr_if_set(void)
{
	if (SPI1->SR & SPI_SR_OVR)
	{
		(void)SPI1->DR;
		(void)SPI1->SR;
	}
}


int spi_transfer(const uint8_t *transfer_buf, uint16_t transfer_buf_len, uint8_t *recieve_buf, uint16_t recieve_buf_len)
{
	spi1_transfer_info.write_idx = 0;
	spi1_transfer_info.recieve_bytes_remaining = recieve_buf_len;
	spi1_transfer_info.transfer.done = 0;

	spi1_transfer_info.transfer.transfer_buf_len = transfer_buf_len;
	spi1_transfer_info.transfer.recieve_buf_len = recieve_buf_len;

	if (transfer_buf_len > 0)
	{
		spi1_transfer_info.transfer.transfer_buf = transfer_buf;
	}

	if (recieve_buf_len > 0)
	{
		SPI1->CR2 |= SPI_CR2_RXNEIE;
		spi1_transfer_info.transfer.recieve_buf = recieve_buf;
	}
	SPI1->CR2 |= SPI_CR2_TXEIE;

	//TODO: Put logic below to address the device we are communicating with
	// I believe we need to pull down the CS line connected to said device but
	// we can deal with that once we have our devices connected to the stm32
	// and know which lines we need to pull down
	
	return 1;
}


void SPI1_IRQnHandler(void)
{
	if (SPI1->SR & SPI_SR_OVR)
	{
		clear_ovr_if_set();
	}

	if (SPI1->SR & (SPI_SR_OVR | SPI_SR_MODF))
	{
		(void)SPI1->SR;
		SPI1->CR1 |= SPI_CR1_SSI;   // keep NSS internal high
		SPI1->CR1 |= SPI_CR1_SPE;   // SPI disabled on MODF: re-enable
		spi1_transfer_info.transfer.done = -1;
	}

	while (dr_ready_for_read())
	{
		uint8_t recieved_byte = (uint8_t)SPI1->DR;
		if (spi1_transfer_info.recieve_bytes_remaining > 0)
		{
			*spi1_transfer_info.transfer.recieve_buf = recieved_byte;
			spi1_transfer_info.transfer.recieve_buf++;
			spi1_transfer_info.recieve_bytes_remaining--;
		}
	}

	while (dr_ready_for_transfer())
	{
		if (spi1_transfer_info.write_idx < spi1_transfer_info.transfer.transfer_buf_len)
		{
			SPI1->DR = spi1_transfer_info.transfer.transfer_buf[spi1_transfer_info.write_idx++];
		}
		else if (spi1_transfer_info.recieve_bytes_remaining > 0)
		{
			SPI1->DR = 0xFF; // insert dummy data once transfer is done
		}
		else
		{
			SPI1->CR2 &= ~SPI_CR2_TXEIE;
			break; //nothing left to clock
		}
	}

	if (spi1_transfer_info.recieve_bytes_remaining == 0 && spi1_transfer_info.write_idx >= spi1_transfer_info.transfer.transfer_buf_len && !spi_busy())
	{
		SPI1->CR2 &= ~SPI_CR2_RXNEIE;
		SPI1->CR2 &= ~SPI_CR2_TXEIE;
		spi1_transfer_info.transfer.done = 1;
	}
}

