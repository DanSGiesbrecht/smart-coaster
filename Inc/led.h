/*
 * led.h
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

typedef enum
{
    LED0 = 1 << 0,
    LED1 = 1 << 1,
    LED2 = 1 << 2,
    LED3 = 1 << 3,
    LED4 = 1 << 4,
    LED5 = 1 << 5,
    LED6 = 1 << 6,
    LED7 = 1 << 8,
    number_leds
} led_number;


/*
 * LED_Init:
 *
 * Set up LED_rgb_array to be empty.
 * Set LED anodes to be off.
 * Start the six PWM channels.
 */
void LED_Init();

/*
 * LED_SetColor:
 *
 * Assign 8-bit values for the RGB elements of LED_rgb_array for the specified LED.
 */
void LED_SetColor( uint32_t _hex_color, led_number _led );

/*
 * LED_RefreshMatrixTask:
 *
 * Task to raster the 8 LED matrix, at a set interval.
 * Fetches LED RGB values from LED_rgb_array.
 * Two LEDs are set at a time.
 */
void LED_RefreshMatrixTask();

#endif /* LED_H_ */
