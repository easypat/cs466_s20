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
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

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

#define READ 0x0a
#define WRITE 0x0b
#define SPECIAL 0x0c

/* Slave LED registers */
#define LED_REG 0x01 // Add register info later
#define LED_REG_BLUE 1
#define LED_REG_RED 2
#define LED_REG_GREEN 4

/* Slave Switch registers */
#define SW_REG 0x02 // Add register info later
#define SW1_DEPRESSED 1
#define SW2_DEPRESSED 2
#define INTERRUPT_SOURCE_SW1 4
#define INTERRUPT_SOURCE_SW2 8

/* Slave Interrupt register */
#define INT_REG 0x03 // Add register info later
#define INT_ACK_SW1 1
#define INT_ACK_SW2 2
#define INT_ENABLE_SW1 4
#define INT_ENABLE_SW2 8

// B, C, D are interrupt ports.
// B is for reset B6
// C is for clock C7
// D is for CS D6

#define RESET_B GPIO_PIN_6
#define CLOCK_C GPIO_PIN_7
#define CS_D GPIO_PIN_6

#define SI_A GPIO_PIN_4
#define SO_A GPIO_PIN_3

#define LED_ON(x) (GPIO_PORTF_DATA_R |= (x))
#define LED_OFF(x) (GPIO_PORTF_DATA_R &= ~(x))
#define LED_TOGGLE(x) (GPIO_PORTF_DATA_R ^= x)
#define LED(led,on) ((on)?LED_ON(led):LED_OFF(led))
#define HIGH 1
#define LOW 0
void spinDelayMs(uint32_t ms);
void SET_LED(int on);
uint32_t SystemCoreClock;

uint8_t read_byte = 0;
int read_count = 0;

static SemaphoreHandle_t _enable = NULL;
static SemaphoreHandle_t _reset = NULL;
static SemaphoreHandle_t _soft_reset = NULL;
static SemaphoreHandle_t _sw1 = NULL;
static SemaphoreHandle_t _sw2 = NULL;

int s1 = 0;
int s2 = 0;
int resetInts1 = 0;
int resetInts2 = 0;
uint8_t LED_register = 0x00;
uint8_t SW_register = 0x00;
uint8_t INT_register = 0x00;

QueueHandle_t state_q = NULL;

TaskHandle_t readTaskHandle;
TaskHandle_t writeTaskHandle;
int falling_cs = 0;
int falling_clock = 0;
int reset = 0;
enum states
{
    S_IDLE,
    S_CMD,
    S_READ,
    S_WRITE,
    S_EATBYTE,
    S_ERROR
} state;

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

struct qhandles{
    QueueHandle_t read;
    QueueHandle_t write;
}qHandle;

struct message_data {
    QueueHandle_t q;
    uint32_t qsize;
    uint32_t elementsize;
    uint8_t id;
};

struct message {
    uint8_t id;
    uint32_t data;
};

int readbit(){
    return !!(GPIO_PORTA_DATA_R&SI_A);
}
void writebit(int data){
    if(data){
        GPIO_PORTA_DATA_R |= SO_A;
    }else{
        GPIO_PORTA_DATA_R &= ~SO_A;
    }
}
#define INT_B GPIO_PIN_4
static void
_setupHardware(void)
{
    SystemCoreClock = 80000000;  // Required for FreeRTOS.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, SO_A);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, INT_B);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (LED_G|LED_R|LED_B));

    GPIO_PORTF_LOCK_R = 0x4C4F434B; // Make sure PORTF is unlocked for GPIO
    GPIO_PORTF_CR_R = 0xFF;

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, SI_A);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, RESET_B);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, CLOCK_C);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, CS_D);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, SW1|SW2 );
    // TODO Set up Pull up resistors on interrupt pins. 
    GPIOPadConfigSet(GPIO_PORTB_BASE, RESET_B, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTC_BASE, CLOCK_C, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); 
    GPIOPadConfigSet(GPIO_PORTD_BASE, CS_D, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); 
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOPadConfigSet(GPIO_PORTF_BASE, SW2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    spinDelayMs(1000);  // Allow UART to setup
    SysCtlClockSet( SYSCTL_SYSDIV_2_5 |
                    SYSCTL_USE_PLL |
                    SYSCTL_XTAL_16MHZ |
                    SYSCTL_OSC_MAIN);

}

static void _interruptHandlerSW(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t mask = GPIOIntStatus(GPIO_PORTF_BASE, 1);
    if ((mask & SW1)&&(INT_register&(1<<2))&&(!(SW_register&(1<<2))))
    {
        xSemaphoreGiveFromISR(_sw1,&xHigherPriorityTaskWoken);
        GPIO_PORTB_DATA_R |= INT_B;
    }
    if ((mask & SW2)&&(INT_register&(1<<3))&&(!(SW_register&(1<<3))))
    {
        xSemaphoreGiveFromISR(_sw2,&xHigherPriorityTaskWoken);
        GPIO_PORTB_DATA_R |= INT_B;
    } 
    GPIOIntClear(GPIO_PORTF_BASE, mask);
}

static void
_interruptHandlerClock(void)
{
    //
    // We have not woken a task at the start of the ISR.
    //
    //LED_TOGGLE(LED_R);
    // Read on low
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);
    falling_clock = !falling_clock;
    uint8_t low = 0x04, high= 0x03;
    if (!falling_clock)
    {
        // Read, low
        xQueueSendFromISR(state_q,(void*)(&low),&xHigherPriorityTaskWoken);
    }        
    else
    {
        // Write, high
        xQueueSendFromISR(state_q,(void*)(&high),&xHigherPriorityTaskWoken);
    }
    GPIOIntClear(GPIO_PORTC_BASE, mask);
}

static void
_interruptHandlerCS(void)
{
    //
    // We have not woken a task at the start of the ISR.
    //
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t mask = GPIOIntStatus(GPIO_PORTD_BASE, 1);
    uint8_t low = 0x01, high = 0;
    falling_cs = !falling_cs;
    if (falling_cs)
    {
        xQueueSendFromISR(state_q,(void*)(&low),&xHigherPriorityTaskWoken);
    }
    else
    {
        xQueueSendFromISR(state_q,(void*)(&high),&xHigherPriorityTaskWoken);
    }
    GPIOIntClear(GPIO_PORTD_BASE, mask);
}

static void
_interruptHandlerReset(void)
{
    //
    // We have not woken a task at the start of the ISR.
    //
    //LED_TOGGLE(LED_B);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t mask = GPIOIntStatus(GPIO_PORTB_BASE, 1);
    //UARTprintf("RESETTING!?!?!?\n");
    //HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    uint8_t reset = 0x05;
    //xQueueSendFromISR(state_q,(void*)(&reset),&xHigherPriorityTaskWoken);
    GPIOIntClear(GPIO_PORTB_BASE, mask);
}



static void
_heartbeat( void *notUsed )
{
    GPIOIntRegister(GPIO_PORTD_BASE, _interruptHandlerCS);
    GPIOIntTypeSet(GPIO_PORTD_BASE, CS_D, GPIO_BOTH_EDGES);
    IntPrioritySet(INT_GPIOD, 255);  // Required with FreeRTOS 10.1.1,
    GPIOIntEnable(GPIO_PORTD_BASE, CS_D);

    GPIOIntRegister(GPIO_PORTB_BASE, _interruptHandlerReset);
    GPIOIntTypeSet(GPIO_PORTB_BASE, RESET_B, GPIO_FALLING_EDGE);
    IntPrioritySet(INT_GPIOB, 255);  
    GPIOIntEnable(GPIO_PORTB_BASE, RESET_B);

    GPIOIntRegister(GPIO_PORTC_BASE, _interruptHandlerClock);
    GPIOIntTypeSet(GPIO_PORTC_BASE, CLOCK_C, GPIO_BOTH_EDGES);
    IntPrioritySet(INT_GPIOC, 255);  
    GPIOIntEnable(GPIO_PORTC_BASE, CLOCK_C);

    GPIOIntRegister(GPIO_PORTF_BASE, _interruptHandlerSW);
    GPIOIntTypeSet(GPIO_PORTF_BASE, SW2, GPIO_FALLING_EDGE);
    IntPrioritySet(INT_GPIOF, 255);   
    GPIOIntEnable(GPIO_PORTF_BASE, SW2);

    GPIOIntRegister(GPIO_PORTF_BASE, _interruptHandlerSW);
    GPIOIntTypeSet(GPIO_PORTF_BASE, SW1, GPIO_FALLING_EDGE);
    IntPrioritySet(INT_GPIOF, 255);   
    GPIOIntEnable(GPIO_PORTF_BASE, SW1);

    uint32_t green500ms = 500; // 1 second
    uint32_t ledOn = 0;
    uint32_t led[] = {LED_G, LED_R, LED_B};
    int ii = 0;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    WatchdogReloadSet(WATCHDOG0_BASE,SysCtlClockGet()); // 1 sec watchdog timer
    WatchdogIntEnable(WATCHDOG0_BASE);
    IntEnable(INT_WATCHDOG);
    WatchdogResetEnable(WATCHDOG0_BASE);
    

    while(true)
    {

        WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet()); // 1 sec watchdog timer
        ledOn = !ledOn;
        //LED(led[ii], ledOn);
        vTaskDelay(10/portTICK_RATE_MS);
    }
}

BaseType_t readByte(QueueHandle_t queue, uint8_t* buffer){

    return xQueueReceive(queue,(void*)(buffer),portMAX_DELAY);

}
// readByte(read_q,&input_byte);

void LED_update(uint8_t reg){
    LED(LED_B, reg&LED_REG_BLUE);
    LED(LED_R, reg&LED_REG_RED);
    LED(LED_G, reg&LED_REG_GREEN);
}
void SW_update(uint8_t reg){

}
void INT_update(uint8_t reg){

}
void setState(int s){
    state = s;
}

void setRegister(uint8_t *reg, uint8_t val){
    *reg = val;
}

static void
_statemachine( void *queueHandles )
{
    BaseType_t resetTriggered;
    BaseType_t readAvailable;
    struct qhandles q = *((struct qhandles*)queueHandles);
    QueueHandle_t read_q = q.read; // Queue for sending read in bytes to statemachine
    QueueHandle_t write_q = q.write; // Queue for reading in bytes to be written to master device.
    uint8_t input_byte;
    uint8_t cmd, addr, data;

    // Bits 0-3 are address
    // Bits 4-7 are command.
    uint8_t newreg;
    setState(S_IDLE);
    UARTprintf("STARTING STATEMACHINE %d\n",SysCtlClockGet());
    while(true)
    {
        switch(state) {
            case S_IDLE:
                if((readAvailable = readByte(read_q,&input_byte)) == pdTRUE) {
                    cmd = (input_byte&(0xF0))>>4;
                    addr = (input_byte&(0x0F));
                    if (xSemaphoreTake(_sw1,0))
                    {
                        setRegister(&SW_register,SW_register|(1 << 2));
                        GPIOIntDisable(GPIO_PORTF_BASE, SW1);
                    }if (xSemaphoreTake(_sw2,0))
                    {
                        setRegister(&SW_register,SW_register|(1 << 3));
                        GPIOIntDisable(GPIO_PORTF_BASE, SW2);
                    }
                }
                if(cmd == READ) {
                    setState(S_READ);
                } else if(cmd == WRITE) {
                    setState(S_WRITE);
                } else if(cmd == SPECIAL) {
                    setState(S_ERROR);
                    while(1);
                } else {
                    setState(S_ERROR);
                }
                break;
            case S_READ:
                //UARTprintf("Entering READ state");
                switch(addr) {
                    case LED_REG:
                        //UARTprintf("sending  LED_REG %x\n",addr);
                        xQueueSend(write_q, (void*)(&LED_register),0);
                        setState(S_EATBYTE);
                        break;
                    case SW_REG:
                        newreg = (((SW1 & GPIO_PORTF_DATA_R)>>4) | SW_register) | ((((SW2 & GPIO_PORTF_DATA_R)<<1) | SW_register));
                        setRegister(&SW_register,(newreg));
                        xQueueSend(write_q, (void*)(&SW_register), 0);
                        setState(S_EATBYTE);
                        break;
                    case INT_REG:
                        //UARTprintf("sending  Int REG %x\n",addr);
                        xQueueSend(write_q, (void*)(&INT_register), 0);
                        setState(S_EATBYTE);
                        break;
                    default:
                        assert(("DEFAULT CASE", 0));
                        setState(S_ERROR);
                        break;
                }
                break;
            case S_WRITE:
                //UARTprintf("Entering Write state\n");
                switch(addr) {
                    case LED_REG:
                        readAvailable = readByte(read_q,&input_byte);
                        //UARTprintf("Writing to LED Register %x value %x\n",addr,input_byte);
                        setRegister(&LED_register, input_byte);
                        LED_update(LED_register);
                        setState(S_IDLE);
                        break;
                    case SW_REG:
                        readAvailable = readByte(read_q,&input_byte);
                        SW_register = input_byte;
                        setRegister(&SW_register, input_byte);
                        setState(S_IDLE);
                        break;
                    case INT_REG:
                        //UARTprintf("writing to Int address %x\n",addr);
                        readAvailable = readByte(read_q,&input_byte);
                        setRegister(&INT_register, input_byte);
                        if(INT_register&1){
                            setRegister(&SW_register,SW_register&(~(1<<2)));
                            GPIO_PORTB_DATA_R &= ~INT_B;
                            GPIOIntEnable(GPIO_PORTF_BASE, SW1);
                        }
                        if(INT_register&2){
                            setRegister(&SW_register,SW_register&(~(1<<3)));
                            GPIO_PORTB_DATA_R &= ~INT_B;
                            GPIOIntEnable(GPIO_PORTF_BASE, SW2);
                        }
                        setRegister(&INT_register,INT_register&(~(3)));
                        setState(S_IDLE);
                        break;
                    default:
                        assert(("DEFAULT CASE", 0));
                        setState(S_ERROR);
                        break;
                }
                break;
            case S_ERROR: // Error/Complete/Done state
                //UARTprintf("Entering Error state\n");
                if(falling_cs && !falling_clock) {
                    setState(S_IDLE);
                    //UARTprintf("Error/Done state.\n"); 
                }
                break;
            case S_EATBYTE:
                readAvailable = readByte(read_q,&input_byte);
                setState(S_IDLE);
                break;
            default:
                // Hope we don't get here...
                assert(("DEFAULT CASE", 0));
                setState(S_ERROR);
                break;
        }
        //UARTprintf("READ IN %x\n",input_byte);
    }
}


void soft_reset(){
    read_count = 0;
    read_byte = 0;
}

void read_bit(){
    read_byte <<= 1;
    read_byte += readbit();
    read_count++;
    /*if(!falling_cs && read_count == 1){
        soft_reset();
    }*/
}
static void
_read( void *queueHandles)
{
    

    struct qhandles q = *((struct qhandles*)queueHandles);
    QueueHandle_t read_q = q.read; // Queue for sending read in bytes to statemachine
    QueueHandle_t write_q = q.write; // Queue for reading in bytes to be written to master device.
    int down = 1;
    uint8_t write_byte = 0;
    uint8_t write_count = 0;
    uint32_t status;
    uint8_t bit;
    uint8_t cs = 1, clock = 0;
    uint8_t state_change;
    writebit(0);
    while(true)
    {
        //UARTprintf("Waiting for notification\n");
        //UARTprintf("Notified\n");
        //UARTprintf("Tick Registered %d \n",read_count);
        xQueueReceive(state_q,(void*)(&state_change),portMAX_DELAY);
        switch(state_change){
            case 0:
                cs = 1;
                break;
            case 1:
                cs = 0;
                break;
            case 3:
                clock = 1;
                break;
            case 4:
                clock = 0;
                break;
            case 5:
                soft_reset();
                write_byte = 0;
                write_count = 0;
                break;
            default:
                assert(0);
                break;

        }
        if(!cs && !clock){
            read_bit();
            if (read_count == 8)
            {
                xQueueSend(read_q, (void *)(&read_byte), 0);
                soft_reset();
            }
        }else if(!cs && clock){
            if (!write_count && xQueueReceive(write_q, (void *)(&write_byte), 0) == pdTRUE)
            {
                write_count = 8;
            }

            if (write_count)
            {
                writebit(write_byte & 0x80);
                write_count--;
                write_byte <<= 1;
            }
            else
            {
                writebit(0);
            }
        }else{
            if(read_count==1){
                assert(0);
            }
            write_count = 0;
            write_byte = 0;
            soft_reset();
        }
    }
}

int main( void )
{
    _setupHardware();

    

    struct message_data data3;
    QueueHandle_t readQueue  = xQueueCreate(20, sizeof(uint8_t));
    QueueHandle_t writeQueue = xQueueCreate(20, sizeof(uint8_t));
    state_q = xQueueCreate(20, sizeof(uint8_t));
    struct qhandles q;
    q.read = readQueue;
    q.write = writeQueue;
    _enable = xSemaphoreCreateBinary();
    _reset = xSemaphoreCreateBinary();
    _soft_reset = xSemaphoreCreateBinary();
    _sw1 = xSemaphoreCreateBinary();
    _sw2 = xSemaphoreCreateBinary();
    xSemaphoreGive(_reset);
    xTaskCreate(_heartbeat,
                "green",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                NULL );
    xTaskCreate(_read,
                "clock",
                configMINIMAL_STACK_SIZE,
                (void*)(&q),
                tskIDLE_PRIORITY,  // higher numbers are higher priority..
                &readTaskHandle );
    xTaskCreate(_statemachine,
                "state machine",
                configMINIMAL_STACK_SIZE,
                (void*)(&q),
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
