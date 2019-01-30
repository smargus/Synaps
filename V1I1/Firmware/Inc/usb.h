/*
 * usb.h
 *
 *  Created on: 3 окт. 2018 г.
 *      Author: ai
 */

#ifndef USB_H_
#define USB_H_

void usbTaskWorker(void const * argument);

void usb_send_str( char * str );

void usb_send_char( char chr );

uint8_t usb_get_rx_byte( TickType_t xTicksToWait );

uint8_t usb_check_get_rx_byte( char * byte );

int _write(int FD, char * outstr, int len);


#endif /* USB_H_ */
