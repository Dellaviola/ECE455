
#ifndef _INIT_H_
#define  _INIT_H_
/* Includes */
#include "stm32f4xx.h"

// Initialize GPIO and ADC and SPI
void GPIO_ADC_Init(void);
/*
 *  Setup GPIOC ports for ADC
 */

void GPIOA_Init(void);
/*
 *  Setup GPIOA ports for User Button
 */

void GPIOB_Init(void);
/*
 *  Setup GPIOB ports for SPI
 */

void SPI1_Init(void);
/*
 *  Initialize the SPI
 */

uint32_t Get_ADC_Value(void);
/*
 *	This function returns the ADC value with 12b precision (0 to 4095)
 */

void SPI1_Write(uint16_t data);
/*
 *  This function ensures the SPI bus is not busy
 *	Then calls the SPI driver functions to write 8 bits
 */

#endif