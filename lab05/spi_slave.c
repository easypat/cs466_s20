/* Standard includes. */
#include <stdbool.h>
//#include <stdio.h>

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

#ifdef USB_SERIAL_OUTPUT
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#endif

/*local includes*/
#include "assert.h"

#define LED_R (1<<1)
#define LED_G (1<<3)
#define LED_B (1<<2)
#define SW1   (1<<4)
#define SW2   (1<<0)

#define RESET_A GPIO_PIN_5
#define CS_A GPIO_PIN_7
#define SCK_A GPIO_PIN_2
#define SI_A GPIO_PIN_4

#define SO_A GPIO_PIN_3

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))
#define HIGH 1
#define LOW 0

void spinDelayMs(uint32_t ms);
uint32_t SystemCoreClock;
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
// Set System/Master clock
//  Falling edge: set data bits on SI
//  Rising edge: read data bits on SI
void setSCK(int state){
    if(state){
        GPIO_PORTA_DATA_R |= SCK_A;
    }else{
        GPIO_PORTA_DATA_R &= ~(SCK_A);
    }
}

// Set Master Out/Slave In; for writing
void setSI(int state){
    if(state){
        GPIO_PORTA_DATA_R |= SI_A;
    }else{
        GPIO_PORTA_DATA_R &= ~(SI_A);
    }
}

// Set Chip Select state; pulled low to tx/rx data
void setCS(int state){
    if(state){
        GPIO_PORTA_DATA_R |= CS_A;
    }else{
        GPIO_PORTA_DATA_R &= ~(CS_A);
    }
}

// Expander RESET pin, loading default register values
void setRESET(int state){
    if(state){
        GPIO_PORTA_DATA_R |= RESET_A;
    }else{
        GPIO_PORTA_DATA_R &= ~(RESET_A);
    }
}

// Set Master In/Slave Out pin; for reading
uint8_t getSO(){
    return (GPIO_PORTA_DATA_R&SO_A)>0;
}

uint8_t transfer(uint8_t out){
    uint8_t in = 0;
    uint8_t mask = 0x80;
    //setSCK(LOW);
    for(int i = 0; i < 8; i++){
        in <<= 1;
        in += getSO();
        setSI(out & mask);
        vTaskDelay(.01/portTICK_RATE_MS);
        setSCK(HIGH);
        vTaskDelay(.01/portTICK_RATE_MS);
        //UARTprintf("in: %d\n",in);
        //UARTprintf("out: %d\n",in);
        setSCK(LOW);
        out <<= 1;
    }
    setSI(LOW);
    return in;
}

uint8_t expanderReadByte(uint8_t address){
    uint8_t value, preRead = 0x41;

    setCS(LOW);
    transfer(preRead);
    transfer(address);
    //vTaskDelay(1/portTICK_RATE_MS);
    value = transfer(0);
    setCS(HIGH);
    return value;
}



uint8_t expanderWrite(uint8_t address, uint8_t data){
    uint8_t preWrite = 0x40;
    setCS(LOW);
    transfer(preWrite);
    transfer(address);
    //vTaskDelay(1/portTICK_RATE_MS);
    uint8_t value = transfer(data);
    setCS(HIGH);
    return value;
}

uint8_t expanderInvert(uint8_t address, uint8_t bit){
    uint8_t val = expanderReadByte(address);
    uint8_t mask = val&bit;
    if(mask){
        val &= ~bit;
    }else{
        val |= bit;
    }
    expanderWrite(address,val);
}

void expanderInit(void){
    expanderWrite(0,0x55);
    expanderWrite(1,0x3F);
    //expanderWrite(0x06,1);
    return;
}
static void
_configureUART(void)
{
    //
    // Enable UART0
    //
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
_setupHardware(void)
{
    SystemCoreClock = 80000000;  // Required for FreeRTOS.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7));
    GPIO_PORTA_DATA_R |= (CS_A);
    GPIO_PORTA_DATA_R |= (SI_A);
    setRESET(LOW);
    spinDelayMs(1000);  // Allow UART to setup
    setRESET(HIGH);
    setCS(HIGH);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, SO_A);
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
    int ii = 0;

    while(true)
    {

        ledOn = !ledOn;
        LED(led[ii], ledOn);

                
        vTaskDelay(green500ms / portTICK_RATE_MS);
        UARTprintf("Toggle the LED\n");
    }
}



static void
_sck( void *notUsed )
{
    uint32_t clock1ms = 1; // 1 second
    uint32_t ledOn = 0;
    uint32_t led[] = {LED_G, LED_R, LED_B};
    int ii = 0;
    setSCK(LOW);
    uint64_t counter = 0;
    uint8_t address1 = 0x00;
    uint8_t address2 = 0x01;
    uint8_t led_addr = 0x09;
    expanderInit();
    while(true)
    {
        expanderInvert(led_addr,0x01); 
        for(int i = 0; i < 11; i++){
            UARTprintf("%d Counter: %d\n",expanderReadByte(i), i);
        }
        vTaskDelay(500/ portTICK_RATE_MS);
    }
}





int main( void )
{
    _setupHardware();
    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                NULL );
    xTaskCreate(_sck,
                "clock",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                NULL );
#ifdef USB_SERIAL_OUTPUT
    _configureUART();
    spinDelayMs(1000);  // Allow UART to setup
#endif
    
    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    assert(0); // we should never get here..

    return 0;
}
