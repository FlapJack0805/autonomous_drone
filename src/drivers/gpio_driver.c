#include "gpio_driver.h"

#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define IO_PIN_COUNT_PER_PORT (16u)
#define IO_PORT_COUNT (3u)
#define IO_PORT_OFFSET (4u)
#define IO_PORT_MASK (0x3u << IO_PORT_OFFSET)
#define IO_PIN_MASK (0xFu)

#define UNUSED_CONFIG                                                                              \
    {                                                                                              \
        IO_MODE_OUTPUT, IO_OTYPE_PUSH_PULL, IO_OSPEED_LOW, IO_RESISTOR_NONE, IO_OUTPUT_LOW                             \
    }

#define MODE_REGS_PER_PIN 2
#define OTYPE_REGS_PER_PIN 1
#define OSPEED_REGS_PER_PIN 2
#define RESISTOR_REGS_PER_PIN 2
#define OUTPUT_REGS_PER_PIN 1
#define MODE_MSK 3
#define OTYPE_MSK 1
#define OSPEED_MSK 3
#define RESISTOR_MSK 3
#define OUTPUT_MSK 1


static uint8_t io_port(io_e io)
{
	return (io & IO_PORT_MASK) >> IO_PORT_OFFSET;
}


static inline uint8_t io_pin_idx(io_e io)
{
	return io & IO_PIN_MASK;
}


static uint8_t io_pin_bit(io_e io)
{
	return 1 << io_pin_idx(io);
}

static const struct io_config io_initial_configs[IO_PORT_COUNT * IO_PIN_COUNT_PER_PORT] = 
{
    [IO_UNUSED_0] =  UNUSED_CONFIG,
    [IO_UNUSED_2] =  UNUSED_CONFIG,
    [IO_UNUSED_3] =  UNUSED_CONFIG,
    [IO_UNUSED_4] =  UNUSED_CONFIG,
    [GREEN_LED] =  UNUSED_CONFIG,
    [IO_UNUSED_6] =  UNUSED_CONFIG,
    [IO_UNUSED_7] =  UNUSED_CONFIG,
    [IO_UNUSED_8] =  UNUSED_CONFIG,
    [IO_UNUSED_9] =  UNUSED_CONFIG,
    [IO_UNUSED_10] = UNUSED_CONFIG,
    [IO_UNUSED_11] = UNUSED_CONFIG,
    [IO_UNUSED_12] = UNUSED_CONFIG,
    [BOOT0] = {IO_MODE_INPUT, IO_OTYPE_PUSH_PULL, IO_OSPEED_LOW, IO_RESISTOR_PULL_DOWN, IO_OUTPUT_LOW},
    [IO_UNUSED_14] = UNUSED_CONFIG,
    [IO_UNUSED_15] = UNUSED_CONFIG,
    [IO_UNUSED_16] = UNUSED_CONFIG,
    [IO_UNUSED_17] = UNUSED_CONFIG,
    [IO_UNUSED_18] = UNUSED_CONFIG,
    [IO_UNUSED_19] = UNUSED_CONFIG,
    [IO_UNUSED_20] = UNUSED_CONFIG,
    [IO_UNUSED_21] = UNUSED_CONFIG,
    [IO_UNUSED_22] = UNUSED_CONFIG,
    [IO_UNUSED_23] = UNUSED_CONFIG,
    [IO_UNUSED_24] = UNUSED_CONFIG,
    [IO_UNUSED_25] = UNUSED_CONFIG,
    [IO_UNUSED_26] = UNUSED_CONFIG,
    [IO_UNUSED_27] = UNUSED_CONFIG,
    [IO_UNUSED_28] = UNUSED_CONFIG,
    [IO_UNUSED_29] = UNUSED_CONFIG,
    [IO_UNUSED_30] = UNUSED_CONFIG,
    [IO_UNUSED_31] = UNUSED_CONFIG,
    [IO_UNUSED_32] = UNUSED_CONFIG,
    [IO_UNUSED_33] = UNUSED_CONFIG,
    [IO_UNUSED_34] = UNUSED_CONFIG,
    [IO_UNUSED_35] = UNUSED_CONFIG,
    [IO_UNUSED_36] = UNUSED_CONFIG,
    [IO_UNUSED_37] = UNUSED_CONFIG,
    [IO_UNUSED_38] = UNUSED_CONFIG,
    [IO_UNUSED_39] = UNUSED_CONFIG,
    [IO_UNUSED_40] = UNUSED_CONFIG,
    [IO_UNUSED_41] = UNUSED_CONFIG,
    [IO_UNUSED_42] = UNUSED_CONFIG,
    [IO_UNUSED_43] = UNUSED_CONFIG,
    [IO_UNUSED_44] = UNUSED_CONFIG,
    [IO_UNUSED_45] = UNUSED_CONFIG,
    [IO_UNUSED_46] = UNUSED_CONFIG,
    [IO_UNUSED_47] = UNUSED_CONFIG

};

static volatile uint32_t *const port_mode_regs[IO_PORT_COUNT] = {&GPIOA->MODER, &GPIOB->MODER, &GPIOC->MODER};
static volatile uint32_t *const port_otype_regs[IO_PORT_COUNT] = {&GPIOA->OTYPER, &GPIOB->OTYPER, &GPIOC->OTYPER};
static volatile uint32_t *const port_ospeed_regs[IO_PORT_COUNT] = {&GPIOA->OSPEEDR, &GPIOB->OSPEEDR, &GPIOC->OSPEEDR};
static volatile uint32_t *const port_resistor_regs[IO_PORT_COUNT] = {&GPIOA->PUPDR, &GPIOB->PUPDR, &GPIOC->PUPDR};
static volatile uint32_t *const port_output_regs[IO_PORT_COUNT] = {&GPIOA->ODR, &GPIOB->ODR, &GPIOC->ODR};
static volatile uint32_t *const port_input_regs[IO_PORT_COUNT] = {&GPIOA->IDR, &GPIOB->IDR, &GPIOC->IDR};


void io_configure(io_e io, const struct io_config *config)
{
	io_set_mode(io, config->mode);
	io_set_ospeed(io, config->ospeed);
	io_set_otype(io, config->otype);
	io_set_output(io, config->output);
	io_set_resistor(io, config->resistor);
}

void io_init(void)
{
	for (io_e io = (io_e)PA0; io < (io_e)PC15; io++)
	{
		io_configure(io, &io_initial_configs[io]);
	}
}


bool io_config_compare(const struct io_config *config1, const struct io_config *config2)
{
	return (config1->mode == config2->mode) && (config1->otype == config2->otype) && (config1->ospeed == config2->ospeed)
		&& (config1->output == config2->output) && (config1->resistor == config2->resistor);
}


void io_set_mode(io_e io, io_mode_e mode)
{
	const uint8_t port = io_port(io);
	const uint8_t pin = io_pin_idx(io);

	*port_mode_regs[port] &= ~(MODE_MSK << (pin * MODE_REGS_PER_PIN));
	*port_mode_regs[port] |= (mode << (pin * MODE_REGS_PER_PIN));
}



void io_set_otype(io_e io, io_otype_e otype)
{
	const uint8_t port = io_port(io);
	const uint8_t pin = io_pin_idx(io);

	*port_otype_regs[port] &= ~(OTYPE_MSK << (pin * OTYPE_REGS_PER_PIN));
	*port_otype_regs[port] |= (otype << (pin * OTYPE_REGS_PER_PIN));
}



void io_set_ospeed(io_e io, io_ospeed_e ospeed)
{
	const uint8_t port = io_port(io);
	const uint8_t pin = io_pin_idx(io);

	*port_ospeed_regs[port] &= ~(OSPEED_MSK << (pin * OSPEED_REGS_PER_PIN));
	*port_ospeed_regs[port] |= (ospeed << (pin * OSPEED_REGS_PER_PIN));
}


void io_set_resistor(io_e io, io_resistor_e resistor)
{
	const uint8_t port = io_port(io);
	const uint8_t pin = io_pin_idx(io);

	*port_resistor_regs[port] &= ~(RESISTOR_MSK << (pin * RESISTOR_REGS_PER_PIN));
	*port_resistor_regs[port] |= (resistor << (pin * RESISTOR_REGS_PER_PIN));
}


void io_set_output(io_e io, io_output_e output)
{
	const uint8_t port = io_port(io);
	const uint8_t pin = io_pin_idx(io);

	*port_output_regs[port] &= ~(OUTPUT_MSK << (pin * OUTPUT_REGS_PER_PIN));
	*port_output_regs[port] |= (output << (pin * OUTPUT_REGS_PER_PIN));
}


io_input_e io_get_input(io_e io)
{
	return (*port_input_regs[io_port(io)] & io_pin_bit(io)) ? IO_INPUT_HIGH : IO_INPUT_LOW;
}
