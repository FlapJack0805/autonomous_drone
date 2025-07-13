#include "gpio_driver.h"

#include <stdint.h>
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stddef.h>

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
#define RISING_EDGE_MSK 1
#define FALLING_EDGE_MSK 1
#define RISING_EDGE_REGS_PER_PIN 1
#define FALLING_EDGE_REGS_PER_PIN 1
#define IMR_MSK 1
#define IMR_REGS_PER_PIN 1
#define SYSCFG_MSK 0xF
#define SYSCFG_REGS_PER_PIN 4


//returns the port for the port number for the given io pin
static uint8_t io_port(io_e io)
{
	return (io & IO_PORT_MASK) >> IO_PORT_OFFSET;
}


//returns the io number for the given pin
static inline uint8_t io_pin_idx(io_e io)
{
	return io & IO_PIN_MASK;
}


//returns a bit offset by the pin index
static uint8_t io_pin_bit(io_e io)
{
	return 1 << io_pin_idx(io);
}

//initialize all pins whose uses can be defined before the program starts
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
static volatile uint32_t *const port_interrupt_flag[IO_PORT_COUNT] = {&GPIOA->IDR, &GPIOB->IDR, &GPIOC->IDR};
static isr_function isr_table[16] = {0};

//configure an io pin
void io_configure(io_e io, const struct io_config *config)
{
	io_set_mode(io, config->mode);
	io_set_ospeed(io, config->ospeed);
	io_set_otype(io, config->otype);
	io_set_output(io, config->output);
	io_set_resistor(io, config->resistor);
}


//called at the start of every program and configures every pin with its initial configuration
void io_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock
	volatile uint32_t dummy;
	dummy = RCC->AHB1ENR;
	dummy = RCC->AHB1ENR;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock
	dummy = RCC->AHB1ENR;
	dummy = RCC->AHB1ENR;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable GPIOC clock
	dummy = RCC->AHB1ENR;
	dummy = RCC->AHB1ENR;
	for (io_e io = (io_e)PA0; io <= (io_e)PC15; io++)
	{
		io_configure(io, &io_initial_configs[io]);
	}

}


//returns true if both io configs are the same
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

void io_enable_interrupt(io_e io)
{
	EXTI->IMR |= io_pin_bit(io);
}

void io_disable_interrupt(io_e io)
{
	EXTI->IMR &= ~io_pin_bit(io);
}


static void io_set_trigger(io_e io, io_trigger_e trigger)
{
	uint8_t pin = io_pin_idx(io);

	// set trigger
	switch (trigger)
	{
		case IO_TRIGGER_NEITHER:
			EXTI->FTSR &= ~(FALLING_EDGE_MSK << pin * FALLING_EDGE_REGS_PER_PIN);
			EXTI->RTSR &= ~(RISING_EDGE_MSK << pin * RISING_EDGE_REGS_PER_PIN);
			break;
			
		case IO_TRIGGER_FALLING:
			EXTI->FTSR |= (FALLING_EDGE_MSK << pin * FALLING_EDGE_REGS_PER_PIN);
			EXTI->RTSR &= ~(RISING_EDGE_MSK << pin * RISING_EDGE_REGS_PER_PIN);
			break;

		case IO_TRIGGER_RISING:
			EXTI->FTSR &= ~(FALLING_EDGE_MSK << pin * FALLING_EDGE_REGS_PER_PIN);
			EXTI->RTSR |= (RISING_EDGE_MSK << pin * RISING_EDGE_REGS_PER_PIN);
			break;

		case IO_TRIGGER_BOTH:
			EXTI->FTSR |= (FALLING_EDGE_MSK << pin * FALLING_EDGE_REGS_PER_PIN);
			EXTI->RTSR |= (RISING_EDGE_MSK << pin * RISING_EDGE_REGS_PER_PIN);
			break;
	}
}

static void io_set_syscfg(io_e io)
{
	uint8_t port = io_port(io);
	uint8_t pin = io_pin_idx(io);
	uint8_t syscfg_idx = pin / 4;
	uint8_t syscfg_shift = (pin % SYSCFG_REGS_PER_PIN) * SYSCFG_REGS_PER_PIN;

	SYSCFG->EXTICR[syscfg_idx] &= ~(SYSCFG_MSK << syscfg_shift);
	SYSCFG->EXTICR[syscfg_idx] |= (port << syscfg_shift);
}


static void io_set_irq_and_priority(io_e io, uint8_t priority)
{
	uint8_t pin = io_pin_idx(io);
	if (pin == 0)
	{
		NVIC_EnableIRQ(EXTI0_IRQn);
		NVIC_SetPriority(EXTI0_IRQn, priority);
	}
	else if (pin == 1)
	{
		NVIC_EnableIRQ(EXTI1_IRQn);
		NVIC_SetPriority(EXTI1_IRQn, priority);
	}
	else if (pin == 2)
	{
		NVIC_EnableIRQ(EXTI2_IRQn);
		NVIC_SetPriority(EXTI2_IRQn, priority);
	}
	else if (pin == 3)
	{
		NVIC_EnableIRQ(EXTI3_IRQn);
		NVIC_SetPriority(EXTI3_IRQn, priority);
	}
	else if (pin == 4)
	{
		NVIC_EnableIRQ(EXTI4_IRQn);
		NVIC_SetPriority(EXTI4_IRQn, priority);
	}
	else if (pin <= 9)
	{
		NVIC_EnableIRQ(EXTI9_5_IRQn);
		NVIC_SetPriority(EXTI9_5_IRQn, priority);
	}
	else
	{
		NVIC_EnableIRQ(EXTI15_10_IRQn);
		NVIC_SetPriority(EXTI15_10_IRQn, priority);
	}
}


void io_configure_interrupt(io_e io, io_trigger_e trigger, uint8_t priority, isr_function isr)
{
	uint8_t pin = io_pin_idx(io);
	io_set_trigger(io, trigger);
	io_set_irq_and_priority(io, priority);
	io_set_syscfg(io);
	io_enable_interrupt(io);
	isr_table[pin] = isr;
}


static inline void io_unregister_isr(io_e io)
{
	uint8_t pin = io_pin_idx(io);
	isr_table[pin] = NULL;
}

void io_deconfigure_interrupt(io_e io)
{
	io_unregister_isr(io);
	io_disable_interrupt(io);
}


void EXTI0_IRQHandler(void)
{
	if (EXTI->PR & (1 << 0))
	{
		EXTI->PR = (1 << 0);
		if (isr_table[0])
			isr_table[0]();
	}
}


void EXTI1_IRQHandler(void)
{
	if (EXTI->PR & (1 << 1))
	{
		EXTI->PR = (1 << 1);
		if (isr_table[1])
			isr_table[1]();
	}
}



void EXTI2_IRQHandler(void)
{
	if (EXTI->PR & (1 << 2))
	{
		EXTI->PR = (1 << 2);
		if (isr_table[2])
			isr_table[2]();
	}
}



void EXTI3_IRQHandler(void)
{
	if (EXTI->PR & (1 << 3))
	{
		EXTI->PR = (1 << 3);
		if (isr_table[3])
			isr_table[3]();
	}
}



void EXTI4_IRQHandler(void)
{
	if (EXTI->PR & (1 << 4))
	{
		EXTI->PR = (1 << 4);
		if (isr_table[4])
			isr_table[4]();
	}
}


void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & (1 << 5))
	{
		EXTI->PR = (1 << 5);
		if (isr_table[5])
			isr_table[5]();
	}

	if (EXTI->PR & (1 << 6))
	{
		EXTI->PR = (1 << 6);
		if (isr_table[6])
			isr_table[6]();
	}

	if (EXTI->PR & (1 << 7))
	{
		EXTI->PR = (1 << 7);
		if (isr_table[7])
			isr_table[7]();
	}

	if (EXTI->PR & (1 << 8))
	{
		EXTI->PR = (1 << 8);
		if (isr_table[8])
			isr_table[8]();
	}

	if (EXTI->PR & (1 << 9))
	{
		EXTI->PR = (1 << 9);
		if (isr_table[9])
			isr_table[9]();
	}

}


void EXTI15_10_IRQHandler(void)
{
	if (EXTI->PR & (1 << 10))
	{
		EXTI->PR = (1 << 10);
		if (isr_table[10])
			isr_table[10]();
	}

	if (EXTI->PR & (1 << 11))
	{
		EXTI->PR = (1 << 11);
		if (isr_table[11])
			isr_table[11]();
	}

	if (EXTI->PR & (1 << 12))
	{
		EXTI->PR = (1 << 12);
		if (isr_table[12])
			isr_table[12]();
	}

	if (EXTI->PR & (1 << 13))
	{
		EXTI->PR = (1 << 13);
		if (isr_table[13])
			isr_table[13]();
	}

	if (EXTI->PR & (1 << 14))
	{
		EXTI->PR = (1 << 14);
		if (isr_table[14])
			isr_table[14]();
	}

	if (EXTI->PR & (1 << 15))
	{
		EXTI->PR = (1 << 15);
		if (isr_table[15])
			isr_table[15]();
	}

}


void interrupt_init(void)
{
	__enable_irq();
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}
