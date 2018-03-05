/*
 * led.h
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include "cmsis_os.h"

#define LED0    0
#define LED1    1
#define LED2    2
#define LED3    3
#define LED4    4
#define LED5    5
#define LED6    6
#define LED7    7

SemaphoreHandle_t led_mutex;

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
void LED_SetColor( uint32_t _hex_color, uint8_t _led );

/*
 * LED_RefreshMatrixTask:
 *
 * Task to raster the 8 LED matrix, at a set interval.
 * Fetches LED RGB values from LED_rgb_array.
 * Two LEDs are set at a time.
 */
void LED_RefreshMatrixTask();

void LED_AnimationTask();

#endif /* LED_H_ */
