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
#define MID_TRAFFIC  2
#define HIGH_TRAFFIC 3

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

void EXTI0_IRQHandler(void)
void PedestrianHandler(void)

xQueueHandle Traffic_Flow_Queue = 0;
EventGroupHandle_t Event_Flags;
TimerHandle_t xTimers[ 3 ];

int main(void)
{
	// Initialize middleware
	GPIOB_Init();
	SPI1_Init();
	GPIO_ADC_Init();
	GPIOA_Init();

	// Set up queues
	Traffic_Flow_Queue = xQueueCreate( 2, sizeof( uint8_t) );

	// Add queues to registry for simplicity
	vQueueAddToRegistry( Traffic_Flow_Queue, "FlowQueue");

	// Set up Change Light Flag group
	Event_Flags = xEventGroupCreate();

	// Set initial TL to RED
	xEventGroupSetBits( Event_Flags, RED );

	// Set up Software Timers
	xTimers[0] = xTimerCreate( "Car_Timer", pdMS_TO_TICKS( 750 ), pdFALSE, ( void * ) 0, vCarCallback);
	xTimers[1] = xTimerCreate( "TL_Timer", pdMS_TO_TICKS( 750 ), pdFALSE, ( void * ) 0, vTrafficLightCallback);
	xTimers[2] = xTimerCreate( "Update_Timer", pdMS_TO_TICKS( 750 ), pdTRUE, ( void * ) 0, vUpdateLightsCallback);

	// Start tasks
	xTaskCreate( Traffic_Flow_Task, "Traffic_Flow", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate( Traffic_Creator_Task, "Traffic_Creator", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( Traffic_Light_Task, "Traffic_Light", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( Traffic_Display_Task, "Traffic_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Start timers
	for (int i = 0; i < 3; i++) {
		xTimerStart( xTimers[i], 100);
	}

	// Start tasks and timers
	vTaskStartScheduler();
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
		// (3) High ->  2660 - 4095
		// (2) Mid  ->  1330 - 2660
		// (1) Low  ->  100 - 1330
		// (0) Off  ->    0 - 99

		// Determine flow rate
		if (adc_value >= 2660) {
			// High
			flow_rate = HIGH_TRAFFIC;
		} else if (adc_value >= 1330 && adc_value < 2660) {
			// Med
			flow_rate = MID_TRAFFIC;
		} else if (adc_value > 99 && adc_value < 1330){
			// Low
			flow_rate = LOW_TRAFFIC;
		} else if (adc_value <= 99) {
			// Off
			flow_rate = NO_TRAFFIC;
		}

		// Send Flow Rate to queue twice
		for (int k = 0; k < 2; k++) {
			// Wait 1000 ticks to ensure both values make it onto the queue,
			// this helps to sync with tasks 2 & 3
			if (!xQueueSend(Traffic_Flow_Queue, &flow_rate, 1000)) {
				printf("Issue with Flow Rate task!\n");
			}
		}
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Creator_Task( void *pvParameters )
{
	uint8_t flow_rate;

	while(1)
	{
		// Get flow rate from queue
		if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 1000) ) {
			// Create traffic based on flow rate
			if (flow_rate == NO_TRAFFIC) {
				// Off
			} else if (flow_rate == LOW_TRAFFIC) {
				// Low
				if ( xTimerIsTimerActive( xTimers[0] ) == pdFALSE ) {
					xTimerChangePeriod( xTimers[0], pdMS_TO_TICKS( 4500 ), 500);
				}
			} else if (flow_rate == MID_TRAFFIC) {
				// Mid
				if ( xTimerIsTimerActive( xTimers[0] ) == pdFALSE ) {
					xTimerChangePeriod( xTimers[0], pdMS_TO_TICKS( 3000 ), 500);
				}
			} else if (flow_rate == HIGH_TRAFFIC) {
				// High
				if ( xTimerIsTimerActive( xTimers[0] ) == pdFALSE ) {
					xTimerChangePeriod( xTimers[0], pdMS_TO_TICKS( 1500 ), 500);
				}
			}
		}
		// Let next task run
		taskYIELD();
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Light_Task( void *pvParameters )
{
	// Traffic Light states
	// Sending (xx, xT, xx), T = TL bits

	uint8_t flow_rate;
	uint32_t new_period;
	EventBits_t flags;
	uint16_t tick_constant = 750;

	while(1)
	{
		// Get event flag bits
		flags = xEventGroupGetBits( Event_Flags );

		// Get flow rate from queue
		if ( xQueueReceive(Traffic_Flow_Queue, &flow_rate, 1000) ) {
			// Create traffic based on flow rate
			if (flow_rate == NO_TRAFFIC) {
				// Off
				// Do nothing?
			} else if (flow_rate == LOW_TRAFFIC) {
				// Low
				if ( xTimerIsTimerActive( xTimers[1] ) == pdFALSE ) {
					// Calculate new period
					new_period = ((flags & GREEN) != 0) ? (tick_constant * 3) : (tick_constant * 13);

					xTimerChangePeriod( xTimers[1], pdMS_TO_TICKS( new_period ), 500);
				}
			} else if (flow_rate == MID_TRAFFIC) {
				// Mid
				if ( xTimerIsTimerActive( xTimers[1] ) == pdFALSE ) {
					// Calculate new period
					new_period = ((flags & GREEN) != 0) ? (tick_constant * 3) : (tick_constant * 10);

					xTimerChangePeriod( xTimers[1], pdMS_TO_TICKS( new_period ), 500);
				}
			} else if (flow_rate == HIGH_TRAFFIC) {
				// High
				if ( xTimerIsTimerActive( xTimers[1] ) == pdFALSE ) {
					// Calculate new period
					new_period = ((flags & GREEN) != 0) ? (tick_constant * 3) : (tick_constant * 7);

					xTimerChangePeriod( xTimers[1], pdMS_TO_TICKS( new_period ), 500);
				}
			}

			// Let next task run
			taskYIELD();
		}
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Display_Task( void *pvParameters )
{
	EventBits_t flags;

	uint8_t high = 0b11111111;
	uint8_t mid = 0b11111111;
	uint8_t low = 0b11111111;

	uint8_t red_tick = 0;
	uint8_t mid_temp = 0;
	uint8_t shift_temp = 0;

	while (1)
	{
		// Get event flag bits
		flags = xEventGroupGetBits( Event_Flags );

		if ((flags & UPDATE_LIGHTS) != 0) {

			if ((flags & GREEN) != 0)
			{
				red_tick = 0;
				shift_temp = 0;
				low = low>>1;					//move low over
				low |= ((mid & mid_last) << 6);		//add the first bit in


				mid_temp = (mid & mid_first);
				mid = mid >> 1;					//move mid over
				mid &= mid_low;				    //clear the top 5 bits of mid
				mid |= (((high & high_last) << 6) | (green_tl) | (mid_temp >> 4));

				high = high >> 1;
				if ((flags & ADD_CAR) == 0) high |= mid_first;

				// Set light to YELLOW
				if ((flags & CHANGE_LIGHT) != 0) {
					xEventGroupClearBits( Event_Flags, GREEN );
					xEventGroupSetBits( Event_Flags, YELLOW );

					// Clear change bit
					xEventGroupClearBits( Event_Flags, CHANGE_LIGHT );
				}
			}


			else if ((flags & YELLOW) != 0)
			{

				low = low>>1;					//move low over
				low |= ((mid & mid_last) << 6);		//add the first bit in


				mid_temp = (mid & mid_first);
				mid = mid >> 1;					//move mid over
				mid &= mid_low;				    //clear the top 5 bits of mid
				mid |= (((high & high_last) << 6) | (yellow_tl) | (mid_temp >> 4));

				high = high >> 1;
				if ((flags & ADD_CAR) == 0) high |= mid_first;

				// Set light to RED
				if ((flags & CHANGE_LIGHT) != 0) {
					xEventGroupClearBits( Event_Flags, YELLOW );
					xEventGroupSetBits( Event_Flags, RED );

					// Clear change bit
					xEventGroupClearBits( Event_Flags, CHANGE_LIGHT );
				}
			}


			else if ((flags & RED) != 0)
			{

				low = low>>1;					//move low over
				low |= ((mid & mid_last) << 6);		//add the first bit in


				mid = mid >> 1;					//move mid over
				mid &= mid_low;				    //clear the top 5 bits of mid

				mid |= (1<<3);					//light immediately after traffic light is off

				if (!red_tick) mid |= ((high & high_last) << 6); //if we arent building up traffic move last bit of high over

				mid |= red_tl;									//update the traffic light colour

				if (!(mid & mid_first)) red_tick++;				//check the first bit to determine state of traffic buildup
																//if mid_7 is low then we need to build up traffic in the high byte
				high = high >> 1;								//shift high over 1
				if ((flags & ADD_CAR) == 0) high |= mid_first;				//check the add_car condition

				if (red_tick) {
					high &= (0b11111110 << shift_temp);
					if ((high & (1 << (shift_temp + 1))) && !(high & (1 << (shift_temp + 2)))) {
						shift_temp++;
					}
				}


				// Set light to GREEN
				if ((flags & CHANGE_LIGHT) != 0) {
					xEventGroupClearBits( Event_Flags, RED );
					xEventGroupSetBits( Event_Flags, GREEN );

					// Clear change bit
					xEventGroupClearBits( Event_Flags, CHANGE_LIGHT );
				}
			}

			// Write to the LEDs
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

void EXTI0_IRQHandler(void)
{
    // Checks whether the interrupt from EXTI0 or not
    if (EXTI_GetITStatus(EXTI_Line0))
    {
		// Clears the EXTI line pending bit
		EXTI_ClearITPendingBit(EXTI_Line0);
		
        // goto the pedestrian handling function
        PedestrianHandler();
               
        
    }
}
/*-----------------------------------------------------------*/

void PedestrianHandler(void)
{
	//set pedestrian event 
	//check timer remaining and state
	//if red and remainder less than ticks*5, restart timer and extend to ticks*5
	//if yellow continue
	//if green and timer greater than ticks * 3, change timer to ticks *2
	
	uint8_t flags = 0;
	
	flags = xEventGroupGetBits( Event_Flags );
	
	if ((flags & GREEN) != 0 && pdMS_TO_TICKS(xTimerGetPeriod(xTimer[1]) > 2250)
		{

			xTimerChangePeriod(xTimer[1], pdMS_TO_TICKS(1500),	500);
			
		}
		
	if ((flags & RED) != 0 && pdMS_TO_TICKS(xTimerGetPeriod(xTimer[1]) < )
		{

			xTimerChangePeriod(xTimer[1], pdMS_TO_TICKS(1500),	3750);
			
		}
	
	
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
