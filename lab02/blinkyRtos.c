/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>

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

#define LED_R (1<<1)
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

static SemaphoreHandle_t _semBtn = NULL;
static SemaphoreHandle_t _redBtn = NULL;
static SemaphoreHandle_t _blueBtn = NULL;

uint32_t SystemCoreClock;

static void
_interruptHandlerPortF(void)
{
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    uint32_t mask = GPIOIntStatus(GPIO_PORTF_BASE, 1);

    if (mask & SW1)
    {
        xSemaphoreGiveFromISR(_redBtn, &xHigherPriorityTaskWoken);
    }

    if (mask & SW2)
    {
        xSemaphoreGiveFromISR(_blueBtn, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken)
    {
        // should request a schedule update....  
        // ....stay tuned
    }    

    GPIOIntClear(GPIO_PORTF_BASE, mask);
}


static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    // This is a TiveDriver library function
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Unlock SW2
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R = 0xFF;

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW1 );
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW2 );

    //
    // Set weak pull-up for switchs
    // This is a TiveDriver library function
    //
    GPIOPadConfigSet(GPIO_PORTF_BASE, (SW1|SW2), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


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

static void
_heartbeat( void *notUsed )
{
    uint32_t green500ms = 500; // 1 second
    uint32_t ledOn = 0;
    uint32_t led[] = {LED_G, LED_R, LED_B};
    uint32_t ledLen = sizeof(led) / sizeof(uint32_t);
    BaseType_t semRes;
    int ii = 0;

    //
    // Using TiveDriver Library,
    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //
    // Make pin B1 rising edge triggered interrupts.
    // Enable the B pin interrupts.
    //
    // This is put in the low priority task startup code because we want
    // to be very sure that the OS scheduler has started before we receive
    // any interrupts.
    //
    _semBtn = xSemaphoreCreateBinary();

    GPIOIntRegister(GPIO_PORTF_BASE, _interruptHandlerPortF);
    GPIOIntTypeSet(GPIO_PORTF_BASE, SW1, GPIO_FALLING_EDGE);

    IntPrioritySet(INT_GPIOF, 255);  // Required with FreeRTOS 10.1.1, 
    GPIOIntEnable(GPIO_PORTF_BASE, SW1);

    while(true)
    {
        semRes = xSemaphoreTake( _semBtn, 0);   // Won't block but will return
                                                // successful if the sem was given
                                                // for every button press the LED
                                                // should change color.
        if (semRes == pdPASS)
        {
            LED(led[ii], 0);
            ii = (ii+1)%ledLen;
        }

        ledOn = !ledOn;
        LED(led[ii], ledOn);
        
        vTaskDelay(green500ms / portTICK_RATE_MS);
    }
}

int main( void )
{
    _setupHardware();

    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,  // higher numbers are higher priority..
                NULL );

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
