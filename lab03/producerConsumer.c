/* Standard includes. */
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

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

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))

uint32_t SystemCoreClock;

typedef struct {
    QueueHandle_t qHandle;
    int messageID;
    char message[20];
} queueStruct_t;

static void
_setupHardware(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    // This is a TiveDriver library function
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    // These are TiveDriver library functions
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));

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
_consumer( void *conParams )
{
    uint32_t blinkRate = 59; // blink every 40ms
    uint32_t ledOn = 0;
    BaseType_t queueRes;
    queueStruct_t conQueueHandle = *((queueStruct_t*)conParams);

    while(true)
    {
        queueRes = xQueueReceive(conQueueHandle.qHandle, &(conQueueHandle.messageID), portMAX_DELAY);

        UARTprintf("C-Receive\n");
    	ledOn = !ledOn;
        if(conQueueHandle.messageID == 1) LED(LED_B, ledOn);
        if(conQueueHandle.messageID == 2) LED(LED_R, ledOn);
        vTaskDelay(blinkRate / portTICK_RATE_MS);
        ledOn = !ledOn;
        LED(LED_B|LED_R, ledOn);
    }
}

static void
_producer( void *prodParams )
{
    BaseType_t queueRes; // Response for full/not-full queue
    queueStruct_t prodQueueHandle = *((queueStruct_t*)prodParams);
    prodQueueHandle.messageID = 1;
    int r = 1191;

    IntPrioritySet(INT_GPIOF, 255);  // Required with FreeRTOS 10.1.1

    while(true)
    {
        // Create random number for delay
        r = (95+(((r<<3)+93)*53)%11);

        queueRes = xQueueSend(prodQueueHandle.qHandle, (void*)&prodQueueHandle.messageID, (TickType_t) 0);

        if(queueRes == pdPASS) {
            UARTprintf("C-Produce prod1\n");
            vTaskDelay(r / portTICK_RATE_MS);
        } else {
            assert(0);
        }
    }
}

static void
_producerToo( void *prodParams )
{
    BaseType_t queueRes; // Response for full/not-full queue
    queueStruct_t prodQueueHandle = *((queueStruct_t*)prodParams);
    prodQueueHandle.messageID = 2;
    int r = 1023;

    IntPrioritySet(INT_GPIOF, 255);  // Required with FreeRTOS 10.1.1

    while(true)
    {
        // Create random number for delay
        r = (95+(((r<<3)+101)*79)%10);

        queueRes = xQueueSend(prodQueueHandle.qHandle, (void*)&prodQueueHandle.messageID, (TickType_t) 0);

        if(queueRes == pdPASS) {
            UARTprintf("C-Produce prod2\n");
            vTaskDelay(r / portTICK_RATE_MS);
        } else {
            assert(0);
        }
    }
}

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

int main( void )
{
    //QueueHandle_t queue1;
    _setupHardware();
#ifdef USB_SERIAL_OUTPUT
    void spinDelayMs(uint32_t ms);
    _configureUART();
    spinDelayMs(1000);  // Allow UART to setup
#endif

    /* Create queue structure. */
    queueStruct_t qStuff;
    qStuff.qHandle = xQueueCreate(20, sizeof(uint32_t));

    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                NULL );

    /* Start producer and consumer with the queue handle. */
    xTaskCreate(_producer,
		"producer",
		configMINIMAL_STACK_SIZE,
		(void *)&qStuff,
		tskIDLE_PRIORITY + 1,
		NULL );

    xTaskCreate(_producerToo,
		"producer2",
		configMINIMAL_STACK_SIZE,
		(void *)&qStuff,
		tskIDLE_PRIORITY + 1,
		NULL );

    xTaskCreate(_consumer,
		"consumer",
		configMINIMAL_STACK_SIZE,
		(void *)&qStuff,
		tskIDLE_PRIORITY,
		NULL );

    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
