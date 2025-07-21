#ifndef __AVR_ATmega2560__
#define __AVR_ATmega2560__
#endif


#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Pinos PWM para ESCs (Mega 2560):
// Motor 1: OC1A  - Pin 11 (PB5)
// Motor 2: OC3A  - Pin 5  (PE3)
// Motor 3: OC4A  - Pin 6  (PH3)
// Motor 4: OC5A  - Pin 46 (PL3)

#define ESC_MIN_PULSE 1000  // em us
#define ESC_MAX_PULSE 2000  // em us
#define ESC_FREQ      50    // Hz (20ms)

// Função para inicializar todos os timers em modo Fast PWM, TOP=ICRn, freq=50Hz
void esc_pwm_init() {
    // Timer1 - Motor 1 (OC1A - PB5 - Pin 11)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    ICR1 = 39999; // 16MHz/8/50Hz - 1 ciclo = 0.5us, 40000-1 = 20ms
    OCR1A = 3000; // 1500us inicial (neutro)
    DDRB |= (1 << PB5);

    // Timer3 - Motor 2 (OC3A - PE3 - Pin 5)
    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
    ICR3 = 39999;
    OCR3A = 3000;
    DDRE |= (1 << PE3);

    // Timer4 - Motor 3 (OC4A - PH3 - Pin 6)
    TCCR4A = (1 << COM4A1) | (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);
    ICR4 = 39999;
    OCR4A = 3000;
    DDRH |= (1 << PH3);

    // Timer5 - Motor 4 (OC5A - PL3 - Pin 46)
    TCCR5A = (1 << COM5A1) | (1 << WGM51);
    TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS51);
    ICR5 = 39999;
    OCR5A = 3000;
    DDRL |= (1 << PL3);
}

// Função para definir o pulso de cada motor em microssegundos (1000~2000us)
void esc_set_pulse_us(uint8_t motor, uint16_t pulse_us) {
    if (pulse_us < ESC_MIN_PULSE) pulse_us = ESC_MIN_PULSE;
    if (pulse_us > ESC_MAX_PULSE) pulse_us = ESC_MAX_PULSE;
    uint16_t ocr = (pulse_us * 2); // 1us = 2 contagens (prescaler 8, 16MHz)
    switch (motor) {
        case 1: OCR1A = ocr; break;
        case 2: OCR3A = ocr; break;
        case 3: OCR4A = ocr; break;
        case 4: OCR5A = ocr; break;
        default: break;
    }
}

int main(void) {
    esc_pwm_init();

    // Exemplo: inicializa todos os motores em 1000us (desligado)
    for (uint8_t i = 1; i <= 4; i++) {
        esc_set_pulse_us(i, 1000);
    }

    // Exemplo de sweep: aumenta e diminui o pulso de cada motor
    while (1) {
        for (uint16_t p = 1000; p <= 2000; p += 10) {
            for (uint8_t i = 1; i <= 4; i++) esc_set_pulse_us(i, p);
            _delay_ms(10);
        }
        for (uint16_t p = 2000; p >= 1000; p -= 10) {
            for (uint8_t i = 1; i <= 4; i++) esc_set_pulse_us(i, p);
            _delay_ms(10);
        }
    }
}
