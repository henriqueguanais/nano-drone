#ifndef __AVR_ATmega2560__
#define __AVR_ATmega2560__
#endif

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h> // Para abs()

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "USART.h"
#include "mpu6050.h"

// Pinos PWM para ESCs (Mega 2560):
// Motor 1: OC3A  - Pin 5  (PE3)
// Motor 2: OC4A  - Pin 6  (PH3)
// Motor 3: OC4B  - Pin 7  (PH4)
// Motor 4: OC4C  - Pin 8  (PH5)

#define ESC_MIN_PULSE 850  // em us
#define ESC_MAX_PULSE 2000 // em us
#define ESC_FREQ      50    // Hz (20ms)

#define THROTTLE_SAFE 950

#define RC_CHANNELS 6
#define PPM_SYNC_TIME 3000
#define THROTTLE_THRESHOLD 15

// PID
#define KP_ROLL 1.0f
#define KI_ROLL 0.1f
#define KD_ROLL 0.05f
#define KP_PITCH 1.0f
#define KI_PITCH 0.1f
#define KD_PITCH 0.05f
#define MAX_ANGLE_CORRECTION 200

QueueHandle_t xQueueRC;
volatile int16_t current_roll_angle = 0;
volatile int16_t current_pitch_angle = 0;
volatile int16_t roll_correction = 0;
volatile int16_t pitch_correction = 0;
static int32_t roll_integral = 0;
static int16_t roll_last_error = 0;
static int32_t pitch_integral = 0;
static int16_t pitch_last_error = 0;
volatile uint8_t motors_armed = 0; // 0 = desarmado, 1 = armado

// Lookup para atan2
const int16_t atan_lookup[51] = {
    0, 57, 114, 171, 228, 284, 340, 395, 449, 503,
    557, 610, 662, 714, 765, 816, 866, 916, 965, 1013,
    1061, 1107, 1154, 1199, 1244, 1288, 1332, 1376, 1419, 1462,
    1504, 1546, 1588, 1629, 1670, 1711, 1751, 1791, 1831, 1871,
    1910, 1949, 1988, 2027, 2065, 2103, 2141, 2179, 2217, 2254,
    2291
};

int16_t fast_atan2_degrees(int16_t y, int16_t z) {
    if (z == 0) return (y > 0) ? 900 : -900;
    int32_t ratio_abs = (y >= 0 ? y : -y) * 10L;
    int32_t z_abs = (z >= 0 ? z : -z);
    ratio_abs = ratio_abs / z_abs;
    int16_t angle = atan_lookup[ratio_abs];
    if (z < 0) angle = 1800 - angle;
    if (y < 0) angle = -angle;
    return angle;
}

int16_t calculate_pid(int16_t setpoint, int16_t current_value, int32_t *integral, int16_t *last_error, float kp, float ki, float kd) {
    int16_t error = setpoint - current_value;
    int32_t proportional = (int32_t)(kp * 10.0f) * error / 10;
    *integral += error;
    if (*integral > 1000) *integral = 1000;
    if (*integral < -1000) *integral = -1000;
    int32_t integral_term = (int32_t)(ki * 10.0f) * (*integral) / 10;
    int16_t derivative = error - *last_error;
    *last_error = error;
    int32_t derivative_term = (int32_t)(kd * 10.0f) * derivative / 10;
    int32_t output = proportional + integral_term + derivative_term;
    if (output > MAX_ANGLE_CORRECTION) output = MAX_ANGLE_CORRECTION;
    if (output < -MAX_ANGLE_CORRECTION) output = -MAX_ANGLE_CORRECTION;
    return (int16_t)output;
}

// Função para inicializar PWM usando Timer3 (OC3A) e Timer4 (OC4A, OC4B, OC4C)
void esc_pwm_init() {
    // Timer3 - Motor 1 (OC3A - PE3 - Pin 5)
    TCCR3A = (1 << COM3A1) | (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS11); // Prescaler 8
    ICR3 = 39999;
    OCR3A = ESC_MIN_PULSE * 2; // 850us inicial
    DDRE |= (1 << PE3);

    // Timer4 - Motor 2 (OC4A - PH3 - Pin 6), Motor 3 (OC4B - PH4), Motor 4 (OC4C - PH5)
    TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS11); // Prescaler 8
    ICR4 = 39999;
    OCR4A = ESC_MIN_PULSE * 2; // Motor 2 (PH3)
    OCR4B = ESC_MIN_PULSE * 2; // Motor 3 (PH4)
    OCR4C = ESC_MIN_PULSE * 2; // Motor 4 (PH5)
    DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5);

    DDRL |= (1 << PL7); // led externo
}

// Função para definir o pulso de cada motor em microssegundos (850~2000us)
void esc_set_pulse_us(uint8_t motor, uint16_t pulse_us) {
    if (pulse_us < ESC_MIN_PULSE) pulse_us = ESC_MIN_PULSE;
    if (pulse_us > ESC_MAX_PULSE) pulse_us = ESC_MAX_PULSE;
    uint16_t ocr = (pulse_us * 2); // 1us = 2 contagens (prescaler 8, 16MHz)
    switch (motor) {
        case 1: OCR3A = ocr; break; // Motor 1 - Timer3 OC3A
        case 2: OCR4A = ocr; break; // Motor 2 - Timer4 OC4A
        case 3: OCR4B = ocr; break; // Motor 3 - Timer4 OC4B
        case 4: OCR4C = ocr; break; // Motor 4 - Timer4 OC4C
        default: break;
    }
}

static void vtask_mpu6050(void *pvParameters) {
    mpu6050_data_t mpu_data;
    uint32_t sample_count = 0;
    uint8_t led_state = 0;
    if (!mpu6050_test_connection()) {
        // USART_send_string("ERRO: MPU6050 nao encontrado!\r\n");
        for (;;) {
            // Pisca LED Mega: PB7
            PORTB ^= (1 << PB7);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    for (;;) {
        led_state = !led_state;
        if (led_state) PORTB |= (1 << PB7);
        else PORTB &= ~(1 << PB7);
        mpu6050_read_all(&mpu_data);
        sample_count++;
        int16_t roll_tenths = fast_atan2_degrees(mpu_data.accel_x, mpu_data.accel_z);
        int16_t pitch_tenths = fast_atan2_degrees(-mpu_data.accel_y, mpu_data.accel_z);
        current_roll_angle = roll_tenths;
        current_pitch_angle = pitch_tenths;
        roll_correction = calculate_pid(0, roll_tenths, &roll_integral, &roll_last_error, KP_ROLL, KI_ROLL, KD_ROLL);
        pitch_correction = calculate_pid(0, pitch_tenths, &pitch_integral, &pitch_last_error, KP_PITCH, KI_PITCH, KD_PITCH);
        // if (sample_count % 10 == 0) {
        //     USART_send_string("[STAB] Roll:"); USART_send_int(roll_tenths / 10);
        //     USART_send_string("."); USART_send_int(abs(roll_tenths % 10));
        //     USART_send_string(" Pitch:"); USART_send_int(pitch_tenths / 10);
        //     USART_send_string("."); USART_send_int(abs(pitch_tenths % 10));
        //     USART_send_string(" RollCorr:"); USART_send_int(roll_correction);
        //     USART_send_string(" PitchCorr:"); USART_send_int(pitch_correction);
        //     USART_send_string("\r\n");
        // }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void vtask_rc(void *pvParameters) {
    BaseType_t xStatus;
    uint16_t rc_local_values[RC_CHANNELS] = {0};
    static uint16_t previous_throttle = ESC_MIN_PULSE;
    for (;;) {
        xStatus = xQueueReceive(xQueueRC, &rc_local_values, portMAX_DELAY);
        if (xStatus == pdPASS) {
            uint16_t throttle_raw = rc_local_values[2];
            int16_t pitch_cmd = rc_local_values[0] - 1500;
            int16_t roll_cmd  = rc_local_values[1] - 1500;
            uint16_t throttle = previous_throttle;
            if (abs((int16_t)throttle_raw - (int16_t)previous_throttle) > THROTTLE_THRESHOLD) {
                throttle = throttle_raw;
                previous_throttle = throttle;
            }
            if (rc_local_values[4] <= 1500) {
                // Desarmado: todos os motores em 850us
                esc_set_pulse_us(1, ESC_MIN_PULSE);
                esc_set_pulse_us(2, ESC_MIN_PULSE);
                esc_set_pulse_us(3, ESC_MIN_PULSE);
                esc_set_pulse_us(4, ESC_MIN_PULSE);
                motors_armed = 0;
            } else if (throttle >= THROTTLE_SAFE && throttle <= 2900) {
                int16_t pitch_total = (pitch_cmd / 4) + pitch_correction;
                int16_t roll_total = -((roll_cmd / 4) + roll_correction);
                // Correção da ordem dos motores conforme solicitado:
                // Frente: acelera 3 e 4
                // Trás: acelera 1 e 2
                // Esquerda: acelera 2 e 4
                // Direita: acelera 1 e 3
                int16_t m1 = throttle - pitch_total - roll_total; // Frente Direita
                int16_t m2 = throttle + pitch_total - roll_total; // Traseira Esquerda
                int16_t m3 = throttle - pitch_total + roll_total; // Frente Esquerda
                int16_t m4 = throttle + pitch_total + roll_total; // Traseira Direita
                esc_set_pulse_us(1, m1);
                esc_set_pulse_us(2, m2);
                esc_set_pulse_us(3, m3);
                esc_set_pulse_us(4, m4);
                motors_armed = 1;
            } else {
                motors_armed = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void vtask_blink_led_pl7(void *pvParameters) {
    for (;;) {
        if (motors_armed) {
            PORTL |= (1 << PL7); // LED aceso
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            PORTL ^= (1 << PL7); // Pisca LED
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// --- Função micros() usando Timer2 para ATmega2560 ---
volatile uint32_t timer2_overflow_count = 0;
ISR(TIMER2_OVF_vect) {
    timer2_overflow_count++;
}

void timer2_init_for_micros() {
    TCCR2A = 0x00; // Normal mode
    TCCR2B = (1 << CS22); // Prescaler 64 (1 tick = 4us)
    TIMSK2 = (1 << TOIE2); // Habilita interrupção de overflow
}

uint32_t micros() {
    uint8_t tcnt2_val;
    uint32_t overflow_count;
    cli();
    tcnt2_val = TCNT2;
    overflow_count = timer2_overflow_count;
    sei();
    // Cada tick = 4us, Timer2 8-bit
    return ((overflow_count << 8) + tcnt2_val) * 4;
}

// Interrupção externa INT4 para capturar PPM no Mega (PE4 - Digital 2)
ISR(INT4_vect) {
    static uint32_t last_time = 0;
    static uint8_t current_channel = 0;
    static uint16_t rc_values[RC_CHANNELS] = {0};
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t current_time = micros();
    uint32_t pulse_width = current_time - last_time;
    last_time = current_time;
    if (pulse_width > PPM_SYNC_TIME) {
        if (current_channel > 0) {
            xQueueSendFromISR(xQueueRC, &rc_values, &xHigherPriorityTaskWoken);
        }
        current_channel = 0;
    } else if (current_channel < RC_CHANNELS && pulse_width > 500 && pulse_width < 2500) {
        rc_values[current_channel] = (uint16_t)pulse_width;
        current_channel++;
    }
    if (xHigherPriorityTaskWoken == pdTRUE) {
        taskYIELD();
    }
}

int main(void) {
    esc_pwm_init();
    timer2_init_for_micros();
    USART_init(MYUBRR);
    mpu6050_init();
    xQueueRC = xQueueCreate(1, sizeof(uint16_t) * RC_CHANNELS);
    xTaskCreate(vtask_rc, (const char *)"serial", 128, NULL, 1, NULL);
    xTaskCreate(vtask_mpu6050, (const char *)"mpu6050", 192, NULL, 1, NULL);
    xTaskCreate(vtask_blink_led_pl7, (const char *)"blink_led", 64, NULL, 1, NULL); // Task para piscar LED PL7
    // Configura INT4 para borda de subida (PE4 - Digital 2)
    EICRB |= (1 << ISC41) | (1 << ISC40); // INT4 rising edge
    EIMSK |= (1 << INT4); // Enable INT4
    sei();
    vTaskStartScheduler();
    for (;;);
}
