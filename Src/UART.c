/* UART.c
 * Simplifies use of the UART peripheral
 */

#include "UART.h"

#include <string.h>
#include <stdbool.h>

static uint8_t              sendBuffer[UART_MAX_BUFFER_LENGTH] = { 0 };
static UART_HandleTypeDef*  uartHandle = NULL;
static bool                 isInitialized = false;
static bool                 isSending = false;

/*
 * Initializes the UART peripheral
 */
void UART_Init( UART_HandleTypeDef* _uartHandle )
{
    uartHandle = _uartHandle;
    isInitialized = true;
}

/*
 * Sends a message with the UART peripheral
 */
void UART_Send( uint8_t* _message, uint16_t _length )
{
    if( !isInitialized || isSending )
        return;

    if( _length > UART_MAX_BUFFER_LENGTH )
        return;

    isSending = true;

    memcpy( sendBuffer, _message, _length );

    HAL_UART_Transmit_IT( uartHandle, sendBuffer, _length );
}

/*
 * Checks if a message is being sent, returns result (boolean)
 */
bool UART_CheckSending( void )
{
    return isSending;
}

/*
 * Callback for UART transmission being complete
 */
void HAL_UART_TxCpltCallback( UART_HandleTypeDef* _uartHandle )
{
    isSending = false;
}
