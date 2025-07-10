#include "USART.h"
#include <stdlib.h>  // Para itoa()

void USART_init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

unsigned char USART_receive(void) {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

void USART_send_string(const char* str) {
    while (*str) {
        USART_transmit(*str++);
    }
}

void USART_send_int(int value) {
    char buffer[10]; // Buffer para armazenar o inteiro convertido (suficiente para -32768 a 32767)
    itoa(value, buffer, 10); // Converte inteiro para string decimal
    USART_send_string(buffer);
}
