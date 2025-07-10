#define F_CPU 16000000UL
#include <avr/io.h>
#include "USART.h"
#include <util/delay.h>

int main(void) {
    USART_init(MYUBRR);

    USART_send_string("Contador:\r\n");

    int contador = 0;

    while (1) {
        USART_send_int(contador++);
        USART_send_string("\r\n");

        _delay_ms(1000); // Espera 1 segundo
    }
}
