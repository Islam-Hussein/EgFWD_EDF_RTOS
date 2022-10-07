 /*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

#include "semphr.h"

/*	Period Macros	*/
#define BUTTON_1_PERIOD         50
#define BUTTON_2_PERIOD         50
#define TRANSMITTER_PERIOD      100
#define UART_PERIOD             20
#define LOAD_1_PERIOD           10
#define LOAD_2_PERIOD           100
/*	Stack Size Macros	*/

#define STACK_SIZE							200

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

int Button1_TaskInTime =0 , Button1_TaskOutTime =0 , Button1_TaskTotalTime   =0 ;
int Button2_TaskInTime =0 , Button2_TaskOutTime =0 , Button2_TaskTotalTime   =0 ;
int Periodic_TaskInTime      =0 , Periodic_TaskOutTime      =0 , Periodic_TaskTotalTime  =0 ;
int UART_TaskInTime      =0 , UART_TaskOutTime      =0 , UART_TaskTotalTime        =0 ;
int Load1_TaskInTime   =0 , Load1_TaskOutTime   =0 , Load1_TaskTotalTime     =0 ;
int Load2_TaskInTime   =0 , Load2_TaskOutTime   =0 , Load2_TaskTotalTime     =0 ;

int System_Time =0 ;

int CPU_Load =0 ;
/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


TaskHandle_t Button_1_Handler = NULL;
TaskHandle_t Button_2_Handler = NULL;
TaskHandle_t Periodic_Transmitter_Handler  = NULL;
TaskHandle_t UART_Receiver_Handler = NULL;

TaskHandle_t Load_1_Simulation_Handler = NULL;
TaskHandle_t Load_2_Simulation_Handler = NULL;



								
QueueHandle_t xQueueButton_1=NULL ,xQueueButton_2=NULL , xQueuerTransmiter=NULL;



/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

typedef struct
{
	pinState_t Button_1_State;
	pinState_t Button_2_State;

}Global_State;

Global_State buttom_states , button_Prev_state;

Global_State * Buttom_States = &buttom_states ;
Global_State * Buttom_Prev_States = &button_Prev_state ;

typedef enum
{	NONE,
	RISING_EDGE,
	FALLING_EDGE,
	PERIODIC_STRING
}EDGE_SENSE;

typedef struct
{
	EDGE_SENSE BUTTOM_1_EDGE;
	EDGE_SENSE BUTTOM_2_EDGE;
	EDGE_SENSE PERIODIC;

}GLOBAL_EDGE_SENSE_STATE;

GLOBAL_EDGE_SENSE_STATE global_Edge_sense;
GLOBAL_EDGE_SENSE_STATE * Global_Edge = &global_Edge_sense;

GLOBAL_EDGE_SENSE_STATE UART_BUFF;



/*-----------------------------------------------------------*/
/* Task to be Created*/


void Button_1_Monitor( void * pvParameters )
{
	
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	
	Buttom_Prev_States->Button_1_State = GPIO_read(PORT_1 , PIN1);
	
	//volatile uint8_t Edge_Flag = 0;
	
	vTaskSetApplicationTaskTag( NULL, (void *) 1 );

	
	while(1)
	{
		/* Task code goes Here. */
		Buttom_States->Button_1_State = GPIO_read(PORT_1 , PIN1);
		
		/*	RISING EDGE	*/
		if((Buttom_States->Button_1_State == PIN_IS_HIGH) && (Buttom_Prev_States->Button_1_State == PIN_IS_LOW ) )
		{
			Global_Edge->BUTTOM_1_EDGE = RISING_EDGE;
			if(xQueueButton_1 != 0)
			{
				xQueueSend(xQueueButton_1,(void *)&Global_Edge->BUTTOM_1_EDGE ,(TickType_t)0);
			}
		}
		
		
		/*	FALLING EDGE	*/
		if((Buttom_States->Button_1_State == PIN_IS_LOW ) && (Buttom_Prev_States->Button_1_State == PIN_IS_HIGH ) )
		{
			Global_Edge->BUTTOM_1_EDGE = FALLING_EDGE;
			
			if(xQueueButton_1 != 0)
			{
				xQueueSend(xQueueButton_1,(void *)&Global_Edge->BUTTOM_1_EDGE ,(TickType_t)0);
			}
		}

		
		Buttom_Prev_States->Button_1_State = Buttom_States->Button_1_State;		//Storing the new state in the Previous State.
		
		vTaskDelayUntil(&xLastWakeTime , BUTTON_1_PERIOD);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);

		
	}
}



void Button_2_Monitor( void * pvParameters )
{

	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	
	Buttom_Prev_States->Button_2_State = GPIO_read(PORT_1 , PIN2);
	
	//volatile uint8_t Edge_Flag = 0;
	
	vTaskSetApplicationTaskTag( NULL, (void *) 2 );

	
	while(1)
	{
		/* Task code goes Here. */
		Buttom_States->Button_2_State = GPIO_read(PORT_1 , PIN2);
		
		/*	RISING EDGE	*/
		if((Buttom_States->Button_2_State == PIN_IS_HIGH) && (Buttom_Prev_States->Button_2_State == PIN_IS_LOW ) )
		{
			Global_Edge->BUTTOM_2_EDGE = RISING_EDGE;
			if(xQueueButton_2 != 0)
			{
				xQueueSend(xQueueButton_2 ,(void *)&Global_Edge->BUTTOM_2_EDGE ,(TickType_t)0);
			}
		}
		
		/*	FALLING EDGE	*/
		if((Buttom_States->Button_2_State == PIN_IS_LOW ) && (Buttom_Prev_States->Button_2_State == PIN_IS_HIGH ) )
		{
			Global_Edge->BUTTOM_2_EDGE = FALLING_EDGE;
			
			if(xQueueButton_2 != 0)
			{
				xQueueSend(xQueueButton_2 ,(void *)&Global_Edge->BUTTOM_2_EDGE ,(TickType_t)0);
			}
		}

		
		
		Buttom_Prev_States->Button_2_State = Buttom_States->Button_2_State;		//Storing the new state in the Previous State.
		
		vTaskDelayUntil(&xLastWakeTime , BUTTON_2_PERIOD);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);

		
	}
}
void Periodic_Transmitter( void * pvParameters )
{
	volatile char*	String_Periodic;
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	
	Global_Edge->PERIODIC = PERIODIC_STRING;

	vTaskSetApplicationTaskTag( NULL, (void *) 3 );

	while(1)
	{
		String_Periodic = "Periodic\n";
		
		if(xQueuerTransmiter != 0)
    {
			xQueueSend( xQueuerTransmiter, ( void * )&Global_Edge->PERIODIC, (TickType_t) 0 );
    }
		
		
		vTaskDelayUntil(&xLastWakeTime , TRANSMITTER_PERIOD);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
	}
}


void UART_Receiver( void * pvParameters )
{
	
	char* BUTTON_1_ON ="Button 1 ON\n";
	char* BUTTON_1_OFF="Button 1 OFF\n";
	char* BUTTON_2_ON ="Button 2 ON\n";
	char* BUTTON_2_OFF="Button 2 OFF\n";
	
	char* Periodic_String ="Periodic\n";
	
	char receiverBuffer;

	
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();

	vTaskSetApplicationTaskTag( NULL, (void *) 4 );
	
	

	while(1)
	{
		
		if(xQueueButton_1!=NULL)
    {
			if ((xQueueReceive(xQueueButton_1,(void *) &(receiverBuffer), (TickType_t)0)) == pdPASS )
      {
        // xSerialPutChar('\n');
				if (receiverBuffer == RISING_EDGE) vSerialPutString((const signed char * const)BUTTON_1_ON, 12);
				else if (receiverBuffer == FALLING_EDGE) vSerialPutString((const signed char * const)BUTTON_1_OFF, 13);
      }
			
		}	
		
		
		if(xQueueButton_2 !=NULL)
    {
			if ((xQueueReceive(xQueueButton_2,(void *) &(receiverBuffer), (TickType_t)0)) == pdPASS )
      {
        // xSerialPutChar('\n');
				if (receiverBuffer == RISING_EDGE) vSerialPutString((const signed char * const)BUTTON_2_ON, 12);
				else if (receiverBuffer == FALLING_EDGE) vSerialPutString((const signed char * const)BUTTON_2_OFF, 13);
      }
			
		}	


		if(xQueuerTransmiter !=NULL)
    {
			if ((xQueueReceive(xQueuerTransmiter,(void *) &(receiverBuffer), (TickType_t)0)) == pdPASS )
      {
				if (receiverBuffer == PERIODIC_STRING) vSerialPutString((const signed char * const)Periodic_String, 9);
      }
			
		}	
		
		
		vTaskDelayUntil(&xLastWakeTime , UART_PERIOD);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
	}
}

void Load_1_Simulation ( void * pvParameters )
{
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	
	uint32_t i=0, x = 12000 * 5;  /* (XTAL / 1000U) * TIME (MS)  */
	
	/* This task is going to be represented by a voltage scale of 5 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );	
	
	for( ; ; )
	{		
		/*	Dummy Loop*/
		for( i=0 ; i <= x; i++){} /*5 ms delay*/
	
		/*Periodicity  of the task 5 equal 10*/
		vTaskDelayUntil( &xLastWakeTime , LOAD_1_PERIOD);
	
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/* the Task 6 load 2 12ms  */
void Load_2_Simulation ( void * pvParameters )
{
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	
	uint32_t i=0, x = 12000 * 12;  /* (XTAL / 1000U)* TIME (MS) */
	
	/* This task is going to be represented by a voltage scale of 6 . */
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for( ; ; )
	{		
		/*	Dummy Loop	*/
		for( i=0 ; i <= x; i++){} /*12 ms delay*/
			
		/*Periodicity  of the task 6 equal 100*/
		vTaskDelayUntil( &xLastWakeTime , LOAD_2_PERIOD);
			
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}






/* Implement Tick Hook*/

void vApplicationIdleHook( void )
{
  GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
}
void vApplicationTickHook( void )
{
  GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	
	xQueueButton_1    = xQueueCreate( 1,sizeof(char*) );
	xQueueButton_2    = xQueueCreate( 1,sizeof(char*) );	
	xQueuerTransmiter = xQueueCreate( 1,sizeof(char*) );
							


		xTaskPeriodicCreate(
								Button_1_Monitor,       /*  Function that implements the task  */
								"Button_1_Monitor",		 /*  Text name for the Task   */
								STACK_SIZE,							 /* Stack size in words not bytes */
								(void * ) 0, 			 /* Parameter to be passes into the task*/
								1, 								 /* Priority at which task is created  */
								&Button_1_Handler,
								BUTTON_1_PERIOD);/* Used to pass ot the created task Handler */
								
								
		xTaskPeriodicCreate(
							  Button_2_Monitor,       /*  Function that implements the task  */
								"Button_2_Monitor",		 /*  Text name for the Task   */
								STACK_SIZE,							 /* Stack size in words not bytes */
								(void * ) 0, 			 /* Parameter to be passes into the task*/
								1, 								 /* Priority at which task is created  */
								&Button_2_Handler,
								BUTTON_2_PERIOD);/* Used to pass ot the created task Handler */
								

		xTaskPeriodicCreate(
							  Periodic_Transmitter,       /*  Function that implements the task  */
								"Periodic_Transmitter",		 /*  Text name for the Task   */
								STACK_SIZE,							 /* Stack size in words not bytes */
								(void * ) 0, 			 /* Parameter to be passes into the task*/
								1, 								 /* Priority at which task is created  */
								&Periodic_Transmitter_Handler,
								TRANSMITTER_PERIOD);/* Used to pass ot the created task Handler */
								
								
		xTaskPeriodicCreate(
							  UART_Receiver,       /*  Function that implements the task  */
								"UART_Receiver",		 /*  Text name for the Task   */
								STACK_SIZE,							 /* Stack size in words not bytes */
								(void * ) 0, 			 /* Parameter to be passes into the task*/
								1, 								 /* Priority at which task is created  */
								&UART_Receiver_Handler,
								UART_PERIOD);/* Used to pass ot the created task Handler */	
		
		xTaskPeriodicCreate(
							  Load_1_Simulation,       /*  Function that implements the task  */
								"Load 1",		 /*  Text name for the Task   */
								STACK_SIZE,							 /* Stack size in words not bytes */
								(void * ) 0, 			 /* Parameter to be passes into the task*/
								1, 								 /* Priority at which task is created  */
								&Load_1_Simulation_Handler,
								LOAD_1_PERIOD);/* Used to pass ot the created task Handler */	

		xTaskPeriodicCreate(
							  Load_2_Simulation,       /*  Function that implements the task  */
								"Load 1",		 /*  Text name for the Task   */
								STACK_SIZE,							 /* Stack size in words not bytes */
								(void * ) 0, 			 /* Parameter to be passes into the task*/
								1, 								 /* Priority at which task is created  */
								&Load_2_Simulation_Handler,
								LOAD_2_PERIOD);/* Used to pass ot the created task Handler */	
								
								
    /* Create Tasks here */


	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();
 
	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/






