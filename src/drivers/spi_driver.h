#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stdint.h"


typedef struct {
    const uint8_t *transfer_buf;   // pointer to data you want to write (TX buffer)
    uint16_t transfer_buf_len;         // number of bytes to write
    uint8_t *recieve_buf;         // pointer to buffer to store data you read (RX buffer)
    uint16_t recieve_buf_len;         // number of bytes to read
    volatile int done;     // transaction status: 0=busy, 1=success, <0=error
} spi_transfer_t;


void spi_init(void);
int spi_transfer(const uint8_t *word_buf, uint16_t word_buf_len, uint8_t *recieve_buf, uint16_t recieve_buf_len);


#endif
