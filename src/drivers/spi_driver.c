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
	NVIC_EnableIRQ(SPI1_IRQn);
}

static int dr_ready_for_read(void)
{
	return SPI1->SR & SPI_SR_RXNE;
}


static int dr_ready_for_transfer(void)
{
	return SPI1->SR & SPI_SR_RXNE;
}



int spi_transfer(const uint8_t *transfer_buf, uint16_t transfer_buf_len, uint8_t *recieve_buf, uint16_t recieve_buf_len)
{
	spi1_transfer_info.write_idx = 0;
	spi1_transfer_info.recieve_bytes_remaining = recieve_buf_len;
	spi1_transfer_info.transfer.done = 0;

	spi1_transfer_info.transfer.transfer_buf = transfer_buf;
	spi1_transfer_info.transfer.transfer_buf_len = transfer_buf_len;
	spi1_transfer_info.transfer.recieve_buf_len = recieve_buf_len;

	if (recieve_buf_len != 0)
	{
		spi1_transfer_info.phase = SPI_PHASE_RX;
	}

	if (transfer_buf_len != 0)
	{
		spi1_transfer_info.phase = SPI_PHASE_TX;
	}

	//TODO: Put logic below to address the device we are communicating with
	// I believe we need to pull down the CS line connected to said device but
	// we can deal with that once we have our devices connected to the stm32
	
	return 1;
}


void SPI1_IRQn_Handler(void)
{
	while (dr_ready_for_read())
	{
		uint8_t recieved_byte = SPI1->DR;
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
			break; //nothing left to clock
		}
	}

}

