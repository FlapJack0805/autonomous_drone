#include "adc.h"

//NOTE: This is all set up to use PA0 to sample ADC values. If I want I can change it but remember to change the other values as needed.


#define ADC_BUF_LEN 16

volatile uint16_t adc_buf[ADC_BUF_LEN];
/*
 * This function sets up the ADC to do continous readings and store them with DMA
 * Sets resolution to 12-bit
 * Prescaler is set to 2
*/
void init_adc(void)
{
	// enable ADC clock and DMA clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	
	// Set sampling time to 84 cycles
	// NOTE: If the input into the ADC is low impedence I can lower the amount of cycles between readings
	ADC1->SMPR1 = 0;
	ADC1->SMPR2 = 0b100;
	

	// set it up to sample 1 ADC value at channel 0
	ADC1->SQR1 &= ~ADC_SQR1_L;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 1; 

	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN); // must make sure this has happened before continuing
	
	DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)adc_buf;
	DMA2_Stream0->NDTR = ADC_BUF_LEN;
	
	DMA2_Stream0->CR =
        (0U << DMA_SxCR_CHSEL_Pos) |      // select channel 0
        DMA_SxCR_PL_1              |      // high priority
        DMA_SxCR_MSIZE_0           |      // memory size = 16-bit
        DMA_SxCR_PSIZE_0           |      // peripheral size = 16-bit
        DMA_SxCR_MINC              |      // increment memory
        DMA_SxCR_CIRC              |      // circular mode
        DMA_SxCR_DIR_0 * 0;               // peripheral->memory
	
	DMA2_Stream0->CR |= DMA_SxCR_EN;
 
	ADC1->CR2 = 0;
	ADC1->CR2 |= ADC_CR2_DMA; // enable DMA for ADC
	ADC1->CR2 |= ADC_CR2_DDS; // Keeps DMA continously running
	ADC1->CR2 |= ADC_CR2_ADON; // enable ADC
	ADC1->CR2 |= ADC_CR2_CONT; // enable start continous ADC readings
	
	//NOTE: I should change this over to a real delay function if I import an RTOS
	vTaskDelay(1000);

	//NOTE: Call this last
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
