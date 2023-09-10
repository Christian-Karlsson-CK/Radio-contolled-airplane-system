#include <avr/io.h>
#include <stdio.h>

#include <util/setbaud.h>
#include <util/delay.h>

#include "RX.c"

volatile uint8_t dataReceived = 0;
extern uint8_t TxData[32];
volatile char receivedData;

ISR(USART_RX_vect) {
    // Data received, read it and set a flag
    uint8_t receivedData = UDR0;

    if (receivedData == '$') {
        // for later testing
    }

    updateTxDataWithGps(receivedData);

    dataReceived = 1;
}


void init_uart(void) 
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    /*
    #if USE_2X
        UCSR0A |= _BV(U2X0);
    #else
        UCSR0A &= ~(_BV(U2X0));
    #endif
    */

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */

    // Enable USART receive interrupt
    UCSR0B |= (1 << RXCIE0); 
}

int uart_putchar(char c) 
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return(0);
}

int uart_getchar() 
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}
























