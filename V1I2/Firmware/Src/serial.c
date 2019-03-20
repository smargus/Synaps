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

