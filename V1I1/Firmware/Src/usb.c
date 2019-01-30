/*
 * usb.c
 *
 *  Created on: 3 окт. 2018 г.
 *      Author: ai
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "cmsis_os.h"

#include "usbd_cdc_if.h"

#include "usb.h"

extern osMessageQId USB_Tx_QueueHandle;
extern osMessageQId USB_Rx_QueueHandle;

void usb_rx_process_data( uint8_t * buffer, uint32_t count);
void usb_tx_data_from_queue (void);

void usb_send_str( char * str )
{
    size_t len = strlen(str);
    size_t i;

    for ( i = 0; i < len; i++)
    	xQueueSendToBack( USB_Tx_QueueHandle, &str[i] , 0);

}

void usb_send_char( char chr )
{
	xQueueSendToBack( USB_Tx_QueueHandle, &chr , 0);
}

void usbTaskWorker(void const * argument)
{

    for(;;)
    {
	    usb_tx_data_from_queue();
	    osThreadYield();
	    //osDelay(1);
    }
}

void usb_rx_process_data( uint8_t * buffer, uint32_t count)
{
	uint32_t i;

	if ( USB_Rx_QueueHandle !=0 )
	{
		for ( i=0; i < count; i++)
		{
			xQueueSendToBackFromISR( USB_Rx_QueueHandle, &buffer[i], 0);
			//xQueueSendToBackFromISR( USB_Tx_QueueHandle, &buffer[i], 0);
		}
	}
}

void usb_tx_data_from_queue (void)
{

	if ( USB_Tx_QueueHandle !=0 )
	{
		uint8_t buffer [CDC_DATA_HS_MAX_PACKET_SIZE];
		size_t bytesToSend = 0;
		uint8_t status = 0;

		if (uxQueueMessagesWaiting(USB_Tx_QueueHandle) == 0)
			return;

		do
		{
			xQueueReceive( USB_Tx_QueueHandle, &buffer[bytesToSend++], ( TickType_t ) 0 );
		}
		while ( ( uxQueueMessagesWaiting(USB_Tx_QueueHandle) > 0 ) && ( bytesToSend < CDC_DATA_HS_MAX_PACKET_SIZE ) );
//		while ( ( uxQueueMessagesWaiting(USB_Tx_QueueHandle) > 0 ) && ( bytesToSend < 0x33 ) );

		/* Wait until component is ready to send data to host. */
		do
		{
			status = CDC_Transmit_HS(buffer, bytesToSend);
			//osThreadYield();
			//osDelay(1);

		} while ( ( status != USBD_OK ) & ( status != USBD_FAIL ) );

		if ( ( uxQueueMessagesWaiting(USB_Tx_QueueHandle) == 0 ) && ( bytesToSend == CDC_DATA_HS_MAX_PACKET_SIZE ))
		{
			do
			{
				status = CDC_Transmit_HS( NULL, 0 );

				osThreadYield();
				//osDelay(1);

			} while ( ( status != USBD_OK ) & ( status != USBD_FAIL ) );

		}
		osThreadYield();

	}

}

uint8_t usb_get_rx_byte( TickType_t xTicksToWait )
{
    char b=0;

    xQueueReceive( USB_Rx_QueueHandle, &b, xTicksToWait );

    return b;
}

uint8_t usb_check_get_rx_byte( char * byte )
{
    return xQueueReceive( USB_Rx_QueueHandle, byte, 0 );
}


int _write(int FD, char * outstr, int len)
{

    int i;

    for ( i = 0; i < len; i++)
    	xQueueSendToBack( USB_Tx_QueueHandle, &outstr[i] , 0);

	return len;
}

//void usbRxTask(void const * argument)
//{
//
//    for(;;)
//    {
//	osDelay(200);
//    }
//
//}
//
//
//void usbTxTask(void const * argument)
//{
//    uint8_t str [] = "TEST\n\r";
//
//    for(;;)
//    {
//	CDC_Transmit_HS(str,(uint16_t)strlen(str));
//	osDelay(200);
//    }
//
//}
