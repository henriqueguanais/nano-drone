#ifndef USART_H
#define USART_H

#include <avr/io.h>

#define BAUD 57600
#define MYUBRR ((F_CPU / (16UL * BAUD)) - 1)

// Inicializa a USART com o valor do registrador UBRR
void USART_init(unsigned int ubrr);

// Transmite um byte pela USART
void USART_transmit(unsigned char data);

// Recebe um byte pela USART
unsigned char USART_receive(void);

// Envia uma string pela USART
void USART_send_string(const char* str);

// Envia um número inteiro como texto
void USART_send_int(int value);

#endif
