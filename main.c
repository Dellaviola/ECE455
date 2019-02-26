/* Standard includes. */
#include <stdint.h>
#include <stdio.h>

/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

/* Library includes. */
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h"

// Small delay function (Testing)
void delay(int d)
{
	int j = 0;
	while(++j < d);
}

// Initialize GPIO and ADC and SPI
void GPIO_ADC_Init(void)
{
	// Structures for ADC and GPIO
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	// Enable ADC Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// Enable GPIOD Clock for ADC
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Initialize GPIO for ADC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	// Initializing GPIO
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Initialize Common ADC
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ADC1 Structure
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	// Initializing ADC
	ADC_Init(ADC1, &ADC_InitStructure);

	// Set Channel for ADC
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_144Cycles);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
}

void GPIOB_Init(void)
{
   /*
	* PB3 --> SCK
	* PB4 --> LCK
	* PB5 --> MOSI
	*/

	// Enable clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// Set up SCK & MOSI
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin	= GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Set alternate functions
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI1);

	GPIOA->BSRRL |= GPIO_Pin_4;
}

void SPI1_Init(void)
{
	// Initialize SPI1 Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitStructInfo = {
			.SPI_Direction 			= SPI_Direction_1Line_Tx,
			.SPI_Mode 				= SPI_Mode_Master,
			.SPI_DataSize 			= SPI_DataSize_8b,
			.SPI_CPOL 				= SPI_CPOL_Low,
			.SPI_CPHA 				= SPI_CPHA_1Edge,
			.SPI_NSS 				= SPI_NSS_Soft,
			.SPI_BaudRatePrescaler 	= SPI_BaudRatePrescaler_256,
			.SPI_FirstBit 			= SPI_FirstBit_MSB,
	};

	SPI_Init(SPI1, &SPI_InitStructInfo);
	SPI_Cmd(SPI1, ENABLE);
}

uint8_t Get_ADC_Value(void)
{
	ADC_SoftwareStartConv(ADC1);

	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	return ADC_GetConversionValue(ADC1);
}

void SPI1_Write(uint16_t data)
{
	// Wait until SPI1 is ready
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET){;}

	// Send data
	SPI_SendData(SPI1, data);

	//Wait until SPI1 is not busy
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET){;}

	delay(1000);
}

/*-----------------------------------------------------------*/
/* Global Variables */

// Flow Rate
#define NO_TRAFFIC 	 0
#define LOW_TRAFFIC  1
#define HIGH_TRAFFIC 2

// Light States
#define GREEN 	0
#define YELLOW	1
#define RED		2

#define BIT_0 ( 1 << 0 )

#define mainQUEUE_LENGTH 100

static void Traffic_Flow_Task( void *pvParameters );
static void Traffic_Creator_Task( uint8_t flow_rate );
static void Traffic_Light_Task( uint8_t flow_rate );
static void Traffic_Display_Task( void *pvParameters );

xQueueHandle Traffic_Flow_Queue = 0;
xQueueHandle New_Cars_Queue = 0;
xQueueHandle Light_Queue = 0;
xQueueHandle Old_Cars_Queue = 0;
SemaphoreHandle_t Flow_Semaphore;
EventGroupHandle_t Change_Light_Flag;

int main(void)
{
	// Initialize middleware
	GPIOB_Init();
	SPI1_Init();
	GPIO_ADC_Init();

	// Set up queues
	Traffic_Flow_Queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t) );
	New_Cars_Queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t) );
	Light_Queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t) );
	Old_Cars_Queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint8_t) );

	// Add queues to registry for simplicity
	vQueueAddToRegistry( Traffic_Flow_Queue, "FlowQueue");
	vQueueAddToRegistry( New_Cars_Queue, "NewCarsQueue");
	vQueueAddToRegistry( Light_Queue, "LightQueue");
	vQueueAddToRegistry( Old_Cars_Queue, "OldCarsQueue");

	// Set up semaphore
	Flow_Semaphore = xSemaphoreCreateCount( 2, 0 );

	// Set up Change Light Flag group
	Change_Light_Flag = xEventGroupCreate();

	// Start tasks
	xTaskCreate( Traffic_Flow_Task, "Traffic_Flow", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Creator_Task, "Traffic_Creator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Light_Task, "Traffic_Light", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Display_Task, "Traffic_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Start tasks and timers
	vTaskStartScheduler();

	while (1) {
//		SPI1_Write(0xFF);
//		SPI1_Write(0xED);
//		SPI1_Write(0xBF);
//		delay(10000000);
//
//		SPI1_Write(0xFF);
//		SPI1_Write(0xDC);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xFF);
//		SPI1_Write(0xBA);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xFE);
//		SPI1_Write(0xFA);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xFD);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xFB);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xF7);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xEF);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xDF);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xBF);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0x7F);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
//
//		SPI1_Write(0xFF);
//		SPI1_Write(0xF6);
//		SPI1_Write(0xFF);
//		delay(10000000);
	}
}
/*-----------------------------------------------------------*/

static void Traffic_Flow_Task( void *pvParameters )
{
	uint32_t adc_value = 0;
	uint8_t flow_rate = 0;

	while(1)
	{
		// Get potentiometer value from ADC
		adc_value = Get_ADC_Value();

		// Determine if high or low traffic
		// (2) High -> 2047 - 4095
		// (1) Low  ->    1 - 2046
		// (0) Off  ->    0

		// Determine flow rate
		if (adc_value >= 2047) {
			// High
			flow_rate = HIGH_TRAFFIC;
		} else if (adc_value > 0 && adc_value < 2047){
			// Low
			flow_rate = LOW_TRAFFIC;
		} else if (adc_value == 0) {
			// Off
			flow_rate = NO_TRAFFIC;
		}

		// Send Flow Rate to queue twice
		for (int k = 0; k < 2; k++) {
			if (!xQueueSend(Traffic_Flow_Queue, &flow_rate, 1000)) {
				printf("Issue with Flow Rate task!\n");
			}
		}

		// Use a semaphore to guard the resources (one for task 2 and one for task 3)
		xSemaphoreGive( Flow_Semaphore );
		xSemaphoreGive( Flow_Semaphore );

		// Delay task
		vTaskDelay(1000);
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Creator_Task( void *pvParameters )
{
	uint8_t flow_rate;
	uint8_t ticks = 0;

	while(1)
	{
		if ( xSemaphoreTask( Flow_Semaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			// Get flow rate from queue
			if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 500) ) {
				// Create traffic based on flow rate
				if (flow_rate == NO_TRAFFIC) {
					// Off
				} else if (flow_rate == LOW_TRAFFIC) {
					// Low
					if (ticks > 4) {
						// Add a car

						// Reset ticks
						ticks = 0;
					}
				} else if (flow_rate == HIGH_TRAFFIC) {
					// High
					if (ticks > 1) {
						// Add a car

						// Reset ticks
						ticks = 0;
					}
				}
			}
		}

		// Update ticks & delay task
		ticks++;
		vTaskDelay(1000);
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Light_Task( void *pvParameters )
{
	// Traffic Light states
	// Sending (xx, xT, xx), T = TL bits
	// Red 	  -> 011(0/1) -> 6/7
	// Yellow -> 101(0/1) -> A/B
	// Green  -> 110(0/1) -> C/D

	uint8_t flow_rate;
	uint16_t light_delay = 100;
	uint8_t ticks = 0;

	while(1)
	{
		if ( xSemaphoreTask( Flow_Semaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			// Get flow rate from queue
			if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 500) ) {
				// Create traffic based on flow rate
				if (flow_rate == NO_TRAFFIC) {
					// Off
					// Do nothing?
				} else if (flow_rate == LOW_TRAFFIC) {
					// Low
					if (ticks > 5) {
						// Update light to next state
						xEventGroupSetBits( Change_Light_Flag, BIT_0);
					}
				} else if (flow_rate == HIGH_TRAFFIC) {
					// High
					if (ticks > 3) {
						// Update light to next state
						xEventGroupSetBits( Change_Light_Flag, BIT_0);
					}
				}
			}
		}

		// Update counter
		ticks++;

		// Delay task
		vTaskDelay(light_delay);
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Display_Task( void *pvParameters )
{
	uint16_t light_state;
	EventBits_t light_state_change;

	// Test code for detecting traffic light needs to change
	light_state_change = xEventGroupGetBits( Change_Light_Flag );

	// if BIT_0 of light_state_change is 1, change. otherwise nothing
	// Clear bits if changed
	// xEventGroupClearBits( Change_Light_Flag, BIT_0 );

}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}

