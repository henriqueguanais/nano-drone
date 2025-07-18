/*
 * Nano Drone - Blink LED
 * Projeto: Sistema básico de controle para nano drone
 * Microcontrolador: ATmega328P @ 16MHz
 * 
 * Este código faz um LED piscar conectado ao pino PB5 (pino 13 do Arduino)
 * que é comumente usado como LED built-in em placas Arduino Uno/Nano
 */

#include <avr/io.h>
#include <util/delay.h>

// Definições de pinos
#define LED_PIN PB5  // Pino 13 do Arduino (LED built-in)
#define LED_PORT PORTB
#define LED_DDR DDRB

// Função para inicializar o sistema
void init_system(void) {
    // Configura o pino do LED como saída
    LED_DDR |= (1 << LED_PIN);
    
    // Garante que o LED inicie apagado
    LED_PORT &= ~(1 << LED_PIN);
}

// Função para ligar o LED
void led_on(void) {
    LED_PORT |= (1 << LED_PIN);
}

// Função para desligar o LED
void led_off(void) {
    LED_PORT &= ~(1 << LED_PIN);
}

// Função para alternar o estado do LED
void led_toggle(void) {
    LED_PORT ^= (1 << LED_PIN);
}

int main(void) {
    // Inicializa o sistema
    init_system();
    
    // Loop principal
    while (1) {
        led_on();           // Liga o LED
        _delay_ms(500);     // Espera 500ms
        
        led_off();          // Desliga o LED
        _delay_ms(500);     // Espera 500ms
        
        // Alternativa usando toggle:
        // led_toggle();
        // _delay_ms(1000);
    }
    
    return 0;  // Nunca será executado
}