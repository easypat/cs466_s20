#ifndef __UARTSERIAL_H_INCLUDED__
#define __UARTSERIAL_H_INCLUDED__

/*
** Allows calls to 
**
** 	UARTprintf("Your test here\n");
**
** to use this module:
**     1) include this header file.
**     2) include the header "utils/uartstdio.h" in any other 
**        module you want to print from
**     3) Be sure to call configureUART() during your hardware 
**        setup section before creating the tasks.
**     4) You need to add build line for uartSerial.c in the 
**        Makefile.  (hint: look at how assert.c get's built)
*/

/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Tiva Hardware includes. */
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

/* 
** Optional includes for USB serial output 
*/

#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"

void configureUART(void);

#else

/*
** If USB_SERIAL_OUTPUT is not defined, Make all these calls vaporize
*/
#define _configureUART() ((void *) 0)
#define UARTprintf() ((void *) 0)
#define UARTprintf(x) ((void *) 0)

#endif


#endif // __UARTSERIAL_H_INCLUDED__