/*
 * led.c
 */

#include "led.h"
#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

#define CATH0_TIM_CHANNEL   TIM_CHANNEL_1
#define CATH1_TIM_CHANNEL   TIM_CHANNEL_2
#define CATH2_TIM_CHANNEL   TIM_CHANNEL_3
#define CATH3_TIM_CHANNEL   TIM_CHANNEL_4
#define CATH4_TIM_CHANNEL   TIM_CHANNEL_1
#define CATH5_TIM_CHANNEL   TIM_CHANNEL_2

// local variables
static uint8_t led_array[4][6];
static TIM_HandleTypeDef LED_tim1;
static TIM_HandleTypeDef LED_tim2;


// local function prototypes
//static void LED_SetPWM( uint8_t _anode );

// public functions

void LED_Init( TIM_HandleTypeDef * _tim1, TIM_HandleTypeDef * _tim2 )
{
    if( _tim1 != NULL && _tim2 != NULL )
    {
        LED_tim1 = *_tim1;
        LED_tim2 = *_tim2;
    }

    int i,j;
    for( i = 0; i < 4; i++ )
    {
        for( j = 0; j < 6; j++ )
            led_array[i][j] = 0;
    }

    HAL_TIM_PWM_Start( &LED_tim1, CATH0_TIM_CHANNEL );
    HAL_TIM_PWM_Start( &LED_tim1, CATH1_TIM_CHANNEL );
    HAL_TIM_PWM_Start( &LED_tim1, CATH2_TIM_CHANNEL );
    HAL_TIM_PWM_Start( &LED_tim1, CATH3_TIM_CHANNEL );
    HAL_TIM_PWM_Start( &LED_tim2, CATH4_TIM_CHANNEL );
    HAL_TIM_PWM_Start( &LED_tim2, CATH5_TIM_CHANNEL );
}

void LED_SetColor( uint32_t _hex_color, led_number _led )
{
    switch( _led )
    {
        case LED0:
        led_array[0][0] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[0][1] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[0][2] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        case LED1:
        led_array[2][0] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[2][1] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[2][2] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        case LED2:
        led_array[1][3] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[1][4] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[1][5] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        case LED3:
        led_array[3][3] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[3][4] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[3][5] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        case LED4:
        led_array[0][3] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[0][4] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[0][5] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        case LED5:
        led_array[2][3] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[2][4] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[2][5] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        case LED6:
        led_array[1][0] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[1][1] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[1][2] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        case LED7:
        led_array[3][0] = ( _hex_color >> 0  ) & 0xff;      // Set Blue  LED
        led_array[3][1] = ( _hex_color >> 8  ) & 0xff;      // Set Green LED
        led_array[3][2] = ( _hex_color >> 16 ) & 0xff;      // Set Red   LED
        break;

        default:
        break;
    }
}

void LED_RefreshMatrixTask()
{
    TickType_t begin_awake;

    // Initialize tick time before entering for-loop.
    // Only needs to be done manually once.
    begin_awake = xTaskGetTickCount();

    for( ;; )
    {
        static uint8_t i = 0;

        switch( i )
        {
            case 0:
            HAL_GPIO_WritePin( ANODE3_GPIO_Port, ANODE3_Pin, GPIO_PIN_RESET );
            LED_SetPWM( i );
            HAL_GPIO_WritePin( ANODE0_GPIO_Port, ANODE0_Pin, GPIO_PIN_SET );
            i = ( i + 1) % 4;
            break;

            case 1:
            HAL_GPIO_WritePin( ANODE0_GPIO_Port, ANODE0_Pin, GPIO_PIN_RESET );
            LED_SetPWM( i );
            HAL_GPIO_WritePin( ANODE1_GPIO_Port, ANODE1_Pin, GPIO_PIN_SET );
            i = ( i + 1) % 4;
            break;

            case 2:
            HAL_GPIO_WritePin( ANODE1_GPIO_Port, ANODE1_Pin, GPIO_PIN_RESET );
            LED_SetPWM( i );
            HAL_GPIO_WritePin( ANODE2_GPIO_Port, ANODE2_Pin, GPIO_PIN_SET );
            i = ( i + 1) % 4;
            break;

            case 3:
            HAL_GPIO_WritePin( ANODE2_GPIO_Port, ANODE2_Pin, GPIO_PIN_RESET );
            LED_SetPWM( i );
            HAL_GPIO_WritePin( ANODE3_GPIO_Port, ANODE3_Pin, GPIO_PIN_SET );
            i = ( i + 1) % 4;
            break;
        }
        vTaskDelayUntil( &begin_awake, pdMS_TO_TICKS( 4 ) );
    }
}

/*static*/ void LED_SetPWM( uint8_t _anode )
{
    __HAL_TIM_SET_COMPARE( &LED_tim1, CATH0_TIM_CHANNEL, led_array[_anode][0] );
    __HAL_TIM_SET_COMPARE( &LED_tim1, CATH1_TIM_CHANNEL, led_array[_anode][1] );
    __HAL_TIM_SET_COMPARE( &LED_tim1, CATH2_TIM_CHANNEL, led_array[_anode][2] );
    __HAL_TIM_SET_COMPARE( &LED_tim1, CATH3_TIM_CHANNEL, led_array[_anode][3] );
    __HAL_TIM_SET_COMPARE( &LED_tim2, CATH4_TIM_CHANNEL, led_array[_anode][4] );
    __HAL_TIM_SET_COMPARE( &LED_tim2, CATH5_TIM_CHANNEL, led_array[_anode][5] );
}
