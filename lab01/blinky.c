//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// .. Leveraged from the TI Blinky example but no external dependencies
//
//*****************************************************************************

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#define RED 2
#define BLUE 4
#define GREEN 8
#define SW1 0x01
#define SW2 0x10

// Red:  20 x 15Hz
// Blue: 10 x 13Hz
//int red_time, blue_time, green_counter;
static int blink_red = 0, blink_blue = 0;

//
// This is a poor delay method.. Will delay appx 1us,
// Probably inaccurate
//
void
delayUs(uint32_t us)
{
    while(us--)
    {
        __asm("    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n"
              "    nop\n");
    }
}

// This is a Very poor delay method.. Will delay appx 1ms.
// Probably inaccurate
void
delayMs(uint32_t ms)
{
	// Check for Switch 1 input
	if((GPIO_PORTF_DATA_R & SW1) == 0){
		blink_red = 1; // Set red LED flag
		//if(*red_counter == 0) *red_counter = 40;
		//GPIO_PORTF_DATA_R |= (RED);
	}
	// Check for Switch 2 input
	if((GPIO_PORTF_DATA_R & SW2) == 0){
		blink_blue = 1; // Set blue LED flag
		//if(*blue_counter == 0) *blue_counter = 20;
		//GPIO_PORTF_DATA_R |= (BLUE);
	}

	while(ms--){
		delayUs(1000);
	}
}

int
main(void)
{
    volatile uint32_t ui32Loop;
    int red_counter = 0, blue_counter = 0, green_counter = 0;
    int red_time = 0, blue_time = 0;

    // Enable the GPIO port that is used for the on-board LED.
    // Pause for a few moments afterwards to let the chip settle..
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
    delayUs(10);

    // setup the main loop delay for 500ms on then 500ms off...
    //ui32Loop = 500;
    ui32Loop = 1;

    // Unlock GPIOLOCK register, allowing writes and "enabling" SW2
    // 0x00000001 -> GPIOCR is locked/can't be modified
    // 0x00000000 -> GPIOCR is unlocked/can be modified
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // 0x4C4F434B

    // Set the pins for the GPIO Commit register, allowing writes to 
    // GPIO_PORTF_PUR_R and GPIO_PORTF_DEN_R
    GPIO_PORTF_CR_R = 0xFF;
    //
    // Enable the GPIO pin for all LEDs (PF1, PF2, PF3).
    // Enable the GPIO pin for digital function.
    // PF0 and PF4 	 -> input
    // PF1, PF2, and PF3 -> output
    GPIO_PORTF_DIR_R = 0x0E; // 0b01110 Enable RGBs
    GPIO_PORTF_PUR_R = 0x11; // 0b10001 Enable pull up resistors on SW1(PF4) and SW2(PF0)
    GPIO_PORTF_DEN_R = 0x1F; // 0b11111 Enable all pins PF0 thru PF4

    // Counter values

    //
    // Loop forever.
    //
    while(1)
    {
	green_counter++; // Inc. green LED counter by 1ms
	if(blink_red == 1) red_counter++;
	if(blink_blue == 1) blue_counter++;
        // Turn on the LED.
	if(green_counter >= 500){
	        GPIO_PORTF_DATA_R ^= (GREEN);
		green_counter = 0;
	}
	if(red_counter >= 66) {
		GPIO_PORTF_DATA_R ^= (RED);
		red_counter = 0;
		if(++red_time >= 40){
			blink_red = 0;
			red_time = 0;
		}
	}
	if(blue_counter >= 76) {
		GPIO_PORTF_DATA_R ^= (BLUE);
		blue_counter = 0;
		if(++blue_time >= 20){
			blink_blue = 0;
			blue_time = 0;
		}
	}
	/*
	if(blink_red == 1) red_counter = 1320;
	if(red_counter >= 0){
		red_counter--;
		if(red_counter%33 == 0) GPIO_PORTF_DATA_R ^= (RED);
		if(red_counter == 0) GPIO_PORTF_DATA_R &= ~(RED);
	}

	if(blink_blue == 1) blue_counter = 760;
	if(blue_counter >= 0){
		blue_counter--;
		if(blue_counter%38 == 0) GPIO_PORTF_DATA_R ^= (BLUE);
		if(blue_counter == 0) GPIO_PORTF_DATA_R &= ~(BLUE);
	}*/
        // Delay for a bit.
        delayMs(ui32Loop);

        // Turn off the LED.
        //GPIO_PORTF_DATA_R ^= (GREEN);

        // Delay for a bit.
        //delayMs(ui32Loop);
    }
}
