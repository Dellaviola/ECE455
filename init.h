/* Includes */
#include "stm32f4xx.h"

// Initialize GPIO and ADC and SPI
void GPIO_ADC_Init(void);

void GPIOB_Init(void);

void SPI1_Init(void);

uint32_t Get_ADC_Value(void);

void SPI1_Write(uint16_t data);
