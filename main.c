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
#include "../FreeRTOS_Source/include/event_groups.h"

/* Library includes. */
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h"
#include "init.h"

/*-----------------------------------------------------------*/
/* Global Variables */

// Flow Rate
#define NO_TRAFFIC 	 0
#define LOW_TRAFFIC  1
#define HIGH_TRAFFIC 2

// Event Bits
#define CHANGE_LIGHT ( 1 << 0 )
#define ADD_CAR ( 1 << 1 )
#define UPDATE_LIGHTS ( 1 << 2 )

// Light States
#define GREEN 	( 1 << 3 )
#define YELLOW	( 1 << 4 )
#define RED		( 1 << 5 )

//MASKS
#define high_last 0b00000010
#define mid_first 0b10000000
#define mid_last 0b00000010
#define mid_low 0b00000111
#define green_tl 0b00110000
#define yellow_tl 0b01010000
#define red_tl 0b01100000

static void Traffic_Flow_Task( void *pvParameters );
static void Traffic_Creator_Task( void *pvParameters );
static void Traffic_Light_Task( void *pvParameters );
static void Traffic_Display_Task( void *pvParameters );

void vTrafficLightCallback( void* );
void vCarCallback( void* );
void vUpdateLightsCallback( void* );

xQueueHandle Traffic_Flow_Queue = 0;
SemaphoreHandle_t Flow_Semaphore;
EventGroupHandle_t Event_Flags;
TimerHandle_t xTimers[ 3 ];

int main(void)
{
	// Initialize middleware
	GPIOB_Init();					//GPIOB pins are configured for SPI
	SPI1_Init();					//SPI configured for sending data to shift registers
	GPIO_ADC_Init();				//ADC uses GPIOC for potentiometer

	// Set up queues
	Traffic_Flow_Queue = xQueueCreate( 2, sizeof( uint8_t) );

	// Add queues to registry for simplicity
	vQueueAddToRegistry( Traffic_Flow_Queue, "FlowQueue");

	// Set up semaphore
	Flow_Semaphore = xSemaphoreCreateCounting( 2, 0 );

	// Set up Change Light Flag group
	Event_Flags = xEventGroupCreate();		//Event Flags are used to 
		 	
		//	Event Triggers
		//	BIT:0 CHANGE_LIGHT		This event signals that the finite statemachine will change states
		//	BIT:1 ADD_CAR 			This event signals that a car will be added to the end of the display
		//	BIT:2 UPDATE_LIGHTS 	This event signals the LEDs to update the display

		//	Finite States			The finite states are programmed such that only one is active at a time
		//	BIT:3 GREEN				Free flowing traffic with a green light and long delay between state updates
		//	BIT:4 YELLOW			Free flowing traffic with a yellow light and short delay between state updates
		//	BIT:5 RED				Interrupted traffic flow with a red light and long delay between state updates		
		

														
	// Set initial state to GREEN
	xEventGroupSetBits( Event_Flags, GREEN );

	// Set up Software Timers
	xTimers[0] = xTimerCreate( "Car_Timer", pdMS_TO_TICKS( 1000 ), pdFALSE, ( void * ) 0, vCarCallback);
	xTimers[1] = xTimerCreate( "TL_Timer", pdMS_TO_TICKS( 1000 ), pdFALSE, ( void * ) 0, vTrafficLightCallback);
	xTimers[2] = xTimerCreate( "Update_Timer", pdMS_TO_TICKS( 750 ), pdTRUE, ( void * ) 0, vUpdateLightsCallback);

	// Start tasks
	xTaskCreate( Traffic_Flow_Task, "Traffic_Flow", configMINIMAL_STACK_SIZE, NULL, 1, NULL); 		
	xTaskCreate( Traffic_Creator_Task, "Traffic_Creator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Light_Task, "Traffic_Light", configMINIMAL_STACK_SIZE, NULL, 1, NULL);		
	xTaskCreate( Traffic_Display_Task, "Traffic_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);	

		// The traffic flow task handles ADC conversions, and placing the converted values onto the traffic flow queue for use by other tasks
		// The traffic creator task takes the flow value and calculates the period between car send events.
		// The traffic light takes the flow value and calculates the time between state changes
		// The traffic display task handles the statemachine and led output logic
	
	
	// Start timers
	for (int i = 0; i < 3; i++) {
		xTimerStart( xTimers[i], 100);
	}

	// Start tasks and timers
	vTaskStartScheduler();

	while(1)
	{
		//program should not enter here...
	}
}
/*-----------------------------------------------------------*/

static void Traffic_Flow_Task( void *pvParameters )
{
	// This task gets the value from the ADC and puts it on a queue to be used by other tasks

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
		//TODO: semaphores do not operate as intended, consider removing.
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Creator_Task( void *pvParameters )
{

	//This task changes the period between car additions

	//TODO: add more flow rates, possibly using a direct calculation from the flow_rate

	uint8_t flow_rate;

	while(1)
	{
		if ( xSemaphoreTake( Flow_Semaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			// Get flow rate from queue
			if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 0) ) {
				// Create traffic based on flow rate
				if (flow_rate == NO_TRAFFIC) {
					// Off
				} else if (flow_rate == LOW_TRAFFIC) {
					// Low
					if ( xTimerIsTimerActive( xTimers[0] ) == pdFALSE ) {
						xTimerChangePeriod( xTimers[0], pdMS_TO_TICKS( 4500 ), 0);
							// Change traffic add rate to 4500 ms
					}
				} else if (flow_rate == HIGH_TRAFFIC) {
					// High
					if ( xTimerIsTimerActive( xTimers[0] ) == pdFALSE ) {
						xTimerChangePeriod( xTimers[0], pdMS_TO_TICKS( 2250 ), 0);
							// Change traffic add rate to 2250 ms
					}
				}
			}
			taskYIELD();	// Yield the task. Necessary because semaphore is not functioning as intended
							// Yielding forces scheduler to move onto other tasks
							// Without the yield this task will empty the queue and cause unexpected behaviour when trying to change the traffic flow
							// TODO: check preemption and time slicing options
		}
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Light_Task( void *pvParameters )
{
	// This task changes the period between state change events

	uint8_t flow_rate;
	uint32_t new_period;
	EventBits_t flags;
	uint16_t tick_constant = 750;	//same as refresh rate TODO: move to define

	while(1)
	{
		if ( xSemaphoreTake( Flow_Semaphore, ( TickType_t ) 10 ) == pdTRUE ) {
			// Get flow rate from queue
			if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 0) ) {
				// Get event flag bits
				flags = xEventGroupGetBits( Event_Flags );

				// Create traffic based on flow rate
				if (flow_rate == NO_TRAFFIC) {
					// Off
					// Do nothing?
				} else if (flow_rate == LOW_TRAFFIC) {
					// Low
					if ( xTimerIsTimerActive( xTimers[1] ) == pdFALSE ) {
						// Calculate new period
						new_period = ((flags & GREEN) != 0) ? (tick_constant * 3) : (tick_constant * 13);

						xTimerChangePeriod( xTimers[1], pdMS_TO_TICKS( new_period ), 0);
					}
				} else if (flow_rate == HIGH_TRAFFIC) {
					// High
					if ( xTimerIsTimerActive( xTimers[1] ) == pdFALSE ) {
						// Calculate new period
						new_period = ((flags & GREEN) != 0) ? (tick_constant * 3) : (tick_constant * 8);

						xTimerChangePeriod( xTimers[1], pdMS_TO_TICKS( new_period ), 0);
					}
				}
			}
			taskYIELD();	// Yield the task. Necessary because semaphore is not functioning as intended
							// Yielding forces scheduler to move onto other tasks
							// Without the yield this task will empty the queue and cause unexpected behaviour when trying to change the traffic flow.
		}
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Display_Task( void *pvParameters )
{
	// This task updates the led display, and handles state machine transitions

	EventBits_t flags;
									// SPI output is separated into 3 Bytes for serial out
	uint8_t high = 0b11111111;		// High byte contains 7 of the pre-traffic light leds
	uint8_t mid = 0b11111111;		// Middle byte contains the last byte before the traffic light, the traffic light, and 3 of the post traffic light leds
	uint8_t low = 0b11111111;		// Low byte contains 8 leds after the traffic light

	uint8_t red_tick = 0;			//tracks number of refresh cycles with queued 'cars'
	uint8_t mid_temp = 0;			//temp space for middle byte shifting logic
	uint8_t shift_temp = 0;			//temp space used for high byte logic

	while (1)
	{
		// Get event flag bits
		flags = xEventGroupGetBits( Event_Flags );

		if ((flags & UPDATE_LIGHTS) != 0) {			//wait for refresh event to enter the GREEN YELLOW RED finite state machine

			if ((flags & GREEN) != 0)
			{
				// Low
				// Shifted towards the right, MSB replaced with bit 1 of middle byte

				red_tick = 0;		// FSM always goes from red state to green state
				shift_temp = 0;		// Reset red_tick and shift_temp here
				low = low>>1;						
				low |= ((mid & mid_last) << 6);		

				// Middle
				// Shifted right, msb replaced with bit 1 of high byte, bit 3 replaced by bit 7

				mid_temp = (mid & mid_first);	//store bit 7
				mid = mid >> 1;					
				mid &= mid_low;		//delete the 5 msbits				    	
				mid |= (((high & high_last) << 6) | (green_tl) | (mid_temp >> 4)); 

				// High
				// Shifted right one, new car handler

				high = high >> 1;
				if ((flags & ADD_CAR) == 0) high |= mid_first;

				// Change State Handler:
				// Current State is Green, if change light event occurs, got to yellow state

				if ((flags & CHANGE_LIGHT) != 0) {
					xEventGroupClearBits( Event_Flags, GREEN );
					xEventGroupSetBits( Event_Flags, YELLOW );

					// Clear change bit
					xEventGroupClearBits( Event_Flags, CHANGE_LIGHT );
				}
			}


			else if ((flags & YELLOW) != 0)
			{

				low = low>>1;						//move low over
				low |= ((mid & mid_last) << 6);		//add the first bit in


				mid_temp = (mid & mid_first);
				mid = mid >> 1;						//move mid over
				mid &= mid_low;				    	//clear the top 5 bits of mid
				mid |= (((high & high_last) << 6) | (yellow_tl) | (mid_temp >> 4));

				high = high >> 1;
				if ((flags & ADD_CAR) == 0) high |= mid_first;

				// Change State Handler:
				// Current State is yellow, if change light event occurs, got to red state

				if ((flags & CHANGE_LIGHT) != 0) {
					xEventGroupClearBits( Event_Flags, YELLOW );
					xEventGroupSetBits( Event_Flags, RED );

					// Clear change bit
					xEventGroupClearBits( Event_Flags, CHANGE_LIGHT );
				}
			}


			else if ((flags & RED) != 0)
			{

				low = low>>1;						//move low over
				low |= ((mid & mid_last) << 6);		//add the first bit in


				mid = mid >> 1;						//move mid over
				mid &= mid_low;				    	//clear the top 5 bits of mid

				mid |= (1<<3);						//light immediately after traffic light is off

				if (!red_tick) mid |= ((high & high_last) << 6); //if we arent building up traffic move last bit of high over

				mid |= red_tl;									//update the traffic light colour

				if (!(mid & mid_first)) red_tick++;				//check the first bit to determine state of traffic buildup
																//if mid_7 is low then we need to build up traffic in the high byte
				high = high >> 1;								//shift high over 1
				if ((flags & ADD_CAR) == 0) high |= mid_first;	//check the add_car condition

				
				/*	the below segment is responsible for building up traffic behind a red light
					accomplished by checking that a car is queued... if(red_tick)
					shift_temp tracks the end of the traffic queue
					when the sequence 010 is detected as the high byte approaches the end of the queue
					the end of the queue is updated	and a 0 is moved onto the end
				*/ 
				if (red_tick) {									
					high &= (0b11111110 << shift_temp);
					if ((high & (1 << (shift_temp + 1))) && !(high & (1 << (shift_temp + 2)))) {
						shift_temp++;
					}
				}


				// Change State Handler:
				// Current State is red, if change light event occurs, go to green state

				if ((flags & CHANGE_LIGHT) != 0) {
					xEventGroupClearBits( Event_Flags, RED );
					xEventGroupSetBits( Event_Flags, GREEN );

					// Clear change bit
					xEventGroupClearBits( Event_Flags, CHANGE_LIGHT );
				}
			}

			// Write to the LEDs
			// SPI outputs 3 bytes of data serially to the shift registers/
			SPI1_Write(low);
			SPI1_Write(mid);
			SPI1_Write(high);

			// Clear update lights bit
			xEventGroupClearBits( Event_Flags, UPDATE_LIGHTS );
			xEventGroupClearBits( Event_Flags, ADD_CAR );
		}
	}
}

/*-----------------------------------------------------------*/

void vTrafficLightCallback( void* arg )
{
	// Set bits to signal TL needs to change
	xEventGroupSetBits( Event_Flags, CHANGE_LIGHT );
}
/*-----------------------------------------------------------*/

void vCarCallback( void* arg )
{
	// Set bits to signal a car needs to be added
	xEventGroupSetBits( Event_Flags, ADD_CAR );
}
/*-----------------------------------------------------------*/

void vUpdateLightsCallback( void* arg )
{
	// Set bits to signal LEDs need to be updated
	xEventGroupSetBits( Event_Flags, UPDATE_LIGHTS );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
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
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}



//TODO: look at eventgroup api, possibly update how the event group is accessed.
