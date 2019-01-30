/*
/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os.h"

#include "serial.h"

#include "usart.h"
/*---------------------------------------------------------------------------*/

//#define serialSTRING_DELAY_TICKS		( portMAX_DELAY )
#define serialSTRING_DELAY_TICKS		( 0 )

#define SERIAL_RX_BUF_SIZE   8
#define SERIAL_TX_BUF_SIZE 1024

volatile uint32_t UartReady = 0;
/*---------------------------------------------------------------------------*/

extern QueueHandle_t SerialTxQueueHandle;
extern QueueHandle_t SerialRxQueueHandle;

volatile char espRxChar[SERIAL_RX_BUF_SIZE] = {0};
volatile char espTxBuf [SERIAL_TX_BUF_SIZE] = {0};


//signed portBASE_TYPE xSerialPutChar( char cOutChar, TickType_t xBlockTime );
/*---------------------------------------------------------------------------*/

void vSerialPutString( const char * const pcString )
{
unsigned short usIndex = 0;


	for( usIndex = 0; usIndex < 2048; usIndex++ )
	{
		/* Check for pre-mature end of line. */
		if( '\0' == pcString[ usIndex ] )
		{
			break;
		}

		/* Send out, one character at a time. */
		if( pdTRUE != xSerialPutChar( pcString[ usIndex ], serialSTRING_DELAY_TICKS ) )
		{
			/* Failed to send, this will be picked up in the receive comtest task. */
		}
	}
}
/*---------------------------------------------------------------------------*/

void vSerialPutStringLn( const char * const pcString )
{
    vSerialPutString(pcString);
    vSerialPutString("\r\n");
}

signed portBASE_TYPE xSerialGetChar( char *pcRxedChar, TickType_t xBlockTime )
{
portBASE_TYPE xReturn = pdFALSE;

	if( pdTRUE == xQueueReceive( SerialRxQueueHandle, pcRxedChar, xBlockTime ) )
	{
		/* Picked up a character. */
		xReturn = pdTRUE;
	}
	return xReturn;
}
/*---------------------------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( char cOutChar, TickType_t xBlockTime )
{
	portBASE_TYPE xReturn = pdFALSE;

	/* The ISR is processing characters is so just add to the end of the queue. */
	if( pdTRUE == xQueueSend( SerialTxQueueHandle, &cOutChar, xBlockTime ) )
	{
		xReturn = pdTRUE;
	}
	else
	{
		/* The queue is probably full. */
		xReturn = pdFALSE;
	}

	return xReturn;
}
/*---------------------------------------------------------------------------*/

void vSerialPutChar( char cOutChar )
{
    xSerialPutChar( cOutChar, serialSTRING_DELAY_TICKS );
}


void vSerialPutCharLn( char cOutChar )
{
    xSerialPutChar( cOutChar, serialSTRING_DELAY_TICKS );
    xSerialPutChar( '\r', serialSTRING_DELAY_TICKS );
    xSerialPutChar( '\n', serialSTRING_DELAY_TICKS );
}


//extern void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart);
//extern void HAL_UART1_TxCpltCallback(UART_HandleTypeDef *huart);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == &huart1 )
	{
		BaseType_t pxHigherPriorityTaskWoken;
		uint32_t i;
		for ( i = 0; i < huart->RxXferSize ; i++ )
		{
			xQueueSendFromISR ( SerialRxQueueHandle, (uint8_t*)&espRxChar[i], &pxHigherPriorityTaskWoken );
		}
	}
//	else if( huart == &huart5 )
//	{
//		HAL_UART5_RxCpltCallback(huart);
//	}
}
/*---------------------------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == &huart1 )
	{
		UartReady = 1;
	}
//	else if( huart == &huart5 )
//	{
//		HAL_UART5_TxCpltCallback(huart);
//	}
}
/*---------------------------------------------------------------------------*/


void espUartTxWorker(void const * argument)
{
	UartReady = 1;

	for (;;)
	{
		uint32_t bytesToSend = 0;

		while ( ( UartReady == 1 ) && ( uxQueueMessagesWaiting( SerialTxQueueHandle ) > 0 ) && ( bytesToSend < SERIAL_TX_BUF_SIZE ) )
		{
			uint8_t pcRxedChar;

			if ( xQueueReceive( SerialTxQueueHandle, &pcRxedChar, 0 ) )
				espTxBuf[bytesToSend++] = pcRxedChar;
		}

		if ( bytesToSend > 0 )
		{
			HAL_UART_Transmit_IT( &huart1, (uint8_t*)espTxBuf, bytesToSend);
			UartReady = 0;
			bytesToSend = 0;
		}

		osDelay(1);

	}

}

