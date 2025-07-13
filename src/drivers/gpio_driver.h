
#ifndef IO_DRIVER_H
#define IO_DRIVER_H

#include <stdbool.h>
#include <stdint.h>



typedef enum
{
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15, 
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15, 
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15, 
} io_generic_e;


typedef enum
{
    IO_UNUSED_0 = PA0,
    IO_UNUSED_1 = PA1,
    IO_UNUSED_2 = PA2,
    IO_UNUSED_3 = PA3,
    IO_UNUSED_4 = PA4,
    GREEN_LED = PA5,
    IO_UNUSED_6 = PA6,
    IO_UNUSED_7 = PA7,
    IO_UNUSED_8 = PA8,
    IO_UNUSED_9 = PA9,
    IO_UNUSED_10 = PA10,
    IO_UNUSED_11 = PA11,
    IO_UNUSED_12 = PA12,
    BOOT0 = PA13,
    IO_UNUSED_14 = PA14,
    IO_UNUSED_15 = PA15,
    IO_UNUSED_16 = PB0,
    IO_UNUSED_17 = PB1,
    IO_UNUSED_18 = PB2,
    IO_UNUSED_19 = PB3,
    IO_UNUSED_20 = PB4,
    IO_UNUSED_21 = PB5,
    IO_UNUSED_22 = PB6,
    IO_UNUSED_23 = PB7,
    IO_UNUSED_24 = PB8,
    IO_UNUSED_25 = PB9,
    IO_UNUSED_26 = PB10,
    IO_UNUSED_27 = PB11,
    IO_UNUSED_28 = PB12,
    IO_UNUSED_29 = PB13,
    IO_UNUSED_30 = PB14,
    IO_UNUSED_31 = PB15,
    IO_UNUSED_32 = PC0,
    IO_UNUSED_33 = PC1,
    IO_UNUSED_34 = PC2,
    IO_UNUSED_35 = PC3,
    IO_UNUSED_36 = PC4,
    IO_UNUSED_37 = PC5,
    IO_UNUSED_38 = PC6,
    IO_UNUSED_39 = PC7,
    IO_UNUSED_40 = PC8,
    IO_UNUSED_41 = PC9,
    IO_UNUSED_42 = PC10,
    IO_UNUSED_43 = PC11,
    IO_UNUSED_44 = PC12,
    IO_UNUSED_45 = PC13,
    IO_UNUSED_46 = PC14,
    IO_UNUSED_47 = PC15,

} io_e;


// IO type and direction
typedef enum
{
    IO_MODE_INPUT,
    IO_MODE_OUTPUT,
    IO_MODE_AF,
    IO_MODE_ANALOG
} io_mode_e;


// Output Type
typedef enum
{
    IO_OTYPE_PUSH_PULL,
    IO_OTYPE_OPEN_DRAIN
} io_otype_e;


//Output Speed
typedef enum
{
    IO_OSPEED_LOW,
    IO_OSPEED_MEDIUM,
    IO_OSPEED_HIGH,
    IO_OSPEED_VERY_HIGH
} io_ospeed_e;


//Pull up/down Resistor connected to GPIO pin
typedef enum
{
    IO_RESISTOR_NONE,
    IO_RESISTOR_PULL_UP,
    IO_RESISTOR_PULL_DOWN,
    IO_RESISTOR_RESERVED
} io_resistor_e;


//Output pin state
typedef enum
{
    IO_OUTPUT_LOW,
    IO_OUTPUT_HIGH
} io_output_e;


//Input pin state
typedef enum
{
    IO_INPUT_LOW,
    IO_INPUT_HIGH
} io_input_e;



//Ports
typedef enum
{
    IO_PORTA,
    IO_PORTB,
    IO_PORTC
} io_port_e;


typedef enum
{
    IO_TRIGGER_NEITHER,
    IO_TRIGGER_FALLING,
    IO_TRIGGER_RISING,
    IO_TRIGGER_BOTH
} io_trigger_e;


//Structs

struct io_config
{
    io_mode_e mode;
    io_otype_e otype;
    io_ospeed_e ospeed;
    io_resistor_e resistor;
    io_output_e output;
};


//io definitions
void io_init(void);
void io_configure(io_e io, const struct io_config *config);
void io_get_current_config(io_e io, struct io_config *current_config);
bool io_config_compare(const struct io_config *cfg1, const struct io_config *cfg2);
void io_set_mode(io_e io, io_mode_e mode);
void io_set_otype(io_e io, io_otype_e otype);
void io_set_ospeed(io_e io, io_ospeed_e ospeed);
void io_set_resistor(io_e io, io_resistor_e resistor);
void io_set_output(io_e io, io_output_e output);
io_input_e io_get_input(io_e io);


//interrup definitions
typedef void (*isr_function)(void);
void interrupt_init(void);
void io_configure_interrupt(io_e io, io_trigger_e trigger, uint8_t priority, isr_function isr);
void io_deconfigure_interrupt(io_e io);
void io_enable_interrupt(io_e io);
void io_disable_interrupt(io_e io);


#endif
