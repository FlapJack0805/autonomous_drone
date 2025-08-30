
#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stdint.h"


typedef enum {
    I2C_IDLE=0,
    I2C_PHASE_ADDR_TX,      // addressing in write phase
    I2C_PHASE_TX,           // sending write bytes
    I2C_PHASE_RESTART,      // repeated START between write and read
    I2C_PHASE_ADDR_RX,      // addressing in read phase
    I2C_PHASE_RX,           // reading >2 bytes
    I2C_PHASE_RX_LAST3,     // reading last 3 (special sequence)
    I2C_PHASE_RX_LAST2,     // special case where only two bytes are given to read int
    I2C_PHASE_RX_LAST1,    // special case where only one byte is given to read in
    I2C_DONE,
    I2C_ERR
} i2c_phase_e;


typedef struct {
    uint8_t  addr7;        // 7-bit I²C slave address
    const uint8_t *transfer_buf;   // pointer to data you want to write (TX buffer)
    uint16_t transfer_buf_len;         // number of bytes to write
    uint8_t *recieve_buf;         // pointer to buffer to store data you read (RX buffer)
    uint16_t recieve_buf_len;         // number of bytes to read
    volatile int done;     // transaction status: 0=busy, 1=success, <0=error
} i2c_transfer_t;


void i2c_init(void);
int i2c_transfer(uint8_t addr7, const uint8_t *word_buf, uint16_t word_buf_len, uint8_t *recieve_buf, uint16_t recieve_buf_len);


#endif
