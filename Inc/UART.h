/* UART.h
 * Simplifies use of the UART peripheral
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define UART_MAX_BUFFER_LENGTH  256

/*
 * Initializes the UART peripheral
 */
void UART_Init( UART_HandleTypeDef* _uartHandle );

/*
 * Sends a message with the UART peripheral
 */
void UART_Send( uint8_t* _message, uint16_t _length );

/*
 * Checks if a message is being sent, returns result (boolean)
 */
bool UART_CheckSending( void );

#endif
