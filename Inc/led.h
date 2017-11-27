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



void LED_Init();

void LED_SetColor( uint32_t _hex_color, led_number _led );

void LED_RefreshMatrixTask();

void LED_SetPWM( uint8_t _anode );

#endif /* LED_H_ */
