/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <inttypes.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Hardware includes. */
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"

/*local includes*/
#include "assert.h"

/* Optional includes for USB serial output */
#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif

#define LED_R (1<<1)
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)

#define LOW 0
#define HIGH 3.3

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

uint32_t SystemCoreClock;

static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    // This is a TiveDriver library function
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1); // Output pin for Signal Generator

    //
    // Set weak pull-up for switchs
    // This is a TiveDriver library function
    //
    //GPIOPadConfigSet(GPIO_PORTF_BASE, (SW1|SW2), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


    //
    // Set the clocking to run at (SYSDIV_2_5) 80.0 MHz from the PLL.
    //                            (SYSDIV_3) 66.6 MHz
    //                            (SYSDIV_4) 50.0 MHz
    //                            (SYSDIV_5) 40.0 MHz
    //                            (SYSDIV_6) 33.3 MHz
    //                            (SYSDIV_8) 25.0 MHz
    //                            (SYSDIV_10) 20.0 MHz
    //
    SystemCoreClock = 80000000;  // Required for FreeRTOS.

    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

#ifdef USB_SERIAL_OUTPUT

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
	_assert_failed ("__error__", pcFilename, ui32Line);
}
#endif

//*****************************************************************************
//
//: Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
static void
_configureUART(void)
{
    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}
#endif

static void
_heartbeat( void *notUsed )
{
    uint32_t greenRate = 500; // 1 second
    uint32_t ledOn = 0;

    IntPrioritySet(INT_GPIOF, 255);  // Required with FreeRTOS 10.1.1, 

    while(true)
    {
        ledOn = !ledOn;
        LED(LED_G, ledOn);
	    UARTprintf("---- 500ms Heartbeat ----\n");
        vTaskDelay(greenRate / portTICK_RATE_MS);
    }
}

static void
_sigGen( void *notUsed )
{
    uint32_t genDelay = 10;
    int counter = 0;

    IntPrioritySet(INT_GPIOB, 255);  // Required with FreeRTOS 10.1.1, 

    while(true)
    {
        UARTprintf("Signal sent %d\n", counter);
        counter++;
        GPIO_PORTB_DATA_R ^= GPIO_PIN_1;
        vTaskDelay(genDelay / portTICK_RATE_MS);
    }
}

int main( void )
{
    //QueueHandle_t queue1;
    _setupHardware();
#ifdef USB_SERIAL_OUTPUT
    void spinDelayMs(uint32_t ms);
    _configureUART();
    spinDelayMs(1000);  // Allow UART to setup
#endif

    SysCtlPeripheralEnable(GPIO_PCTL_PB4_SSI2CLK);

    
    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                NULL );
    xTaskCreate(_sigGen,
                "signal",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                NULL );

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
