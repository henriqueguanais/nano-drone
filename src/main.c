#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>  // Para abs()

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "USART.h"
#include "mpu6050.h"

#define RC_CHANNELS 6
#define PPM_SYNC_TIME 3000 // Tempo de sincronização do PPM em microssegundos

#define T1DUTY_MIN 50
#define T1DUTY_MAX 125 // 2000μs = 2ms = 10% duty cycle (125/1250 * 100)
#define T2DUTY_MIN 16
#define T2DUTY_MAX 31 // 2000μs = 2ms = 12% duty cycle (31/256 * 100)

#define THROTTLE_THRESHOLD 15 // Valor mínimo para considerar mudança (ajuste conforme necessário)


void setup(void);
static void vtask_rc(void *pvParameters);
static void vtask_mpu6050(void *pvParameters);
uint16_t map_rc_to_pwm(uint16_t rc_value);
uint8_t map_rc_to_pwm_8bit(uint16_t rc_value);
uint32_t get_timer0_microseconds(void);

const int16_t atan_lookup[51] = {
	0,    57,   114,  171,  228,  284,  340,  395,  449,  503,  // 0.0-0.9
	557,  610,  662,  714,  765,  816,  866,  916,  965,  1013, // 1.0-1.9
	1061, 1107, 1154, 1199, 1244, 1288, 1332, 1376, 1419, 1462, // 2.0-2.9
	1504, 1546, 1588, 1629, 1670, 1711, 1751, 1791, 1831, 1871, // 3.0-3.9
	1910, 1949, 1988, 2027, 2065, 2103, 2141, 2179, 2217, 2254, // 4.0-4.9
	2291  // 5.0
};

int16_t fast_atan2_degrees(int16_t y, int16_t z) {
	if (z == 0) return (y > 0) ? 900 : -900; // ±90°
	
	// Calcula a razão absoluta
	int32_t ratio_abs = (y >= 0 ? y : -y) * 10L;
	int32_t z_abs = (z >= 0 ? z : -z);
	ratio_abs = ratio_abs / z_abs;
	
	// Limita o índice da tabela
	if (ratio_abs > 50) ratio_abs = 50;
	
	int16_t angle = atan_lookup[ratio_abs];
	
	// Ajusta o sinal baseado nos quadrantes
	if (z < 0) angle = 1800 - angle; // 180° - angle
	if (y < 0) angle = -angle;
	
	return angle; // Retorna em décimos de grau
}

QueueHandle_t xQueueRC;

// Variável para criar timer virtual de 16 bits com Timer0 de 8 bits
volatile uint16_t timer0_overflow_count = 0;

int main(void)
{
	setup();

	xQueueRC = xQueueCreate(1, sizeof(uint16_t) * RC_CHANNELS);

	xTaskCreate(vtask_rc, (const char *)"serial", 128, NULL, 1, NULL);
	xTaskCreate(vtask_mpu6050, (const char *)"mpu6050", 192, NULL, 1, NULL);
	vTaskStartScheduler();
	for (;;);
}

// Função que define os pinos de entrada do receptor RC e configura as interrupções
void setup()
{
	// Configura direção dos pinos
	DDRD = 0x00;           // Configura PORTD como entrada

	DDRD &= ~(1 << PD2);   // Garante que PD2 seja entrada (receptor PPM)
	DDRD |= (1 << PD3);    // Define PD3 (OC2B) como saída para PWM - Motor 4
	DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB5); // Define PB1 (OC1A), PB2 (OC1B), PB3 (OC2A) como saída para PWM e PB5 para LED

	// Configura Timer1 para Fast PWM (TOP = ICR1), 50 Hz
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Fast PWM, modo 14, habilita OC1A e OC1B
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12); // Prescaler 256
	ICR1 = 1249; // 50 Hz (16MHz / (256 * 1250) = 50Hz)
	OCR1A = 62;  // 1000μs (1ms) - Motor 1 (PB1) - posição de armamento ESC
	OCR1B = 62;  // 1000μs (1ms) - Motor 2 (PB2) - posição de armamento ESC
	TIMSK1 = 0x00; // Desabilita interrupções do Timer1
	
	// Configura Timer2 para Fast PWM, ~61Hz (mais próximo possível de 50Hz com Timer 8-bit)
	// Frequência = 16MHz / (prescaler × 256) = 16MHz / (1024 × 256) = ~61Hz
	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM mode
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
	OCR2A = 15;  // Motor 3 (PB3) - ~1000μs (1ms) - posição de armamento ESC
	OCR2B = 15;  // Motor 4 (PD3) - ~1000μs (1ms) - posição de armamento ESC
	TIMSK2 = 0x00; // Desabilita interrupções do Timer2
	
	// Configura Timer0 para contagem de tempo (PPM timing)
	TCCR0A = 0x00; // Normal mode
	TCCR0B = 0x02; // Prescaler 8 (1 tick = 0.5 us)
	TIMSK0 = (1 << TOIE0); // Habilita interrupção de overflow do Timer0
	
	// Configura INT0 (PD2) para borda de subida
	EICRA |= (1 << ISC01) | (1 << ISC00); // INT0 na borda de subida
	EIMSK |= (1 << INT0);                 // Habilita INT0


	sei();

	USART_init(MYUBRR);
	mpu6050_init();
}

static void vtask_mpu6050(void *pvParameters)
{
	mpu6050_data_t mpu_data;
	uint32_t sample_count = 0;
	uint8_t led_state = 0;
	
	// Debug: Task iniciada
	
	if (!mpu6050_test_connection()) {
		// Se não conseguir conectar, envia erro pela serial
		USART_send_string("ERRO: MPU6050 nao encontrado!\r\n");
		USART_send_string("Verifique as conexoes I2C (SDA/SCL)\r\n");
		USART_send_string("LED piscando rapidamente...\r\n");
		// E pisca o LED rapidamente
		for (;;) {
			PORTB ^= (1 << PB5);
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
	
	
	for (;;)
	{
		// Controle do LED (pisca a cada leitura)
		led_state = !led_state;
		if (led_state) {
			PORTB |= (1 << PB5);  // Liga LED
		} else {
			PORTB &= ~(1 << PB5); // Desliga LED
		}
		// Lê todos os dados do MPU6050
		mpu6050_read_all(&mpu_data);
		sample_count++;

		// Calcula ângulos usando tabela de lookup (sem math.h)
		// int16_t roll_tenths = fast_atan2_degrees(mpu_data.accel_y, mpu_data.accel_z);
		// int16_t pitch_tenths = fast_atan2_degrees(-mpu_data.accel_x, mpu_data.accel_z);

		// Envia dados do MPU6050 pela USART (apenas aceleração e ângulos)
		// USART_send_string("[MPU6050] ");
		// USART_send_string(" | Roll (°):"); USART_send_int(roll_tenths / 10);
		// USART_send_string("."); USART_send_int(abs(roll_tenths % 10));
		// USART_send_string(" Pitch (°):"); USART_send_int(pitch_tenths / 10);
		// USART_send_string("."); USART_send_int(abs(pitch_tenths % 10));
		// USART_send_string("\r\n");

		// Delay de 1 segundo entre leituras para melhor visualização
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

// Função de envio dos valores do receptor RC via USART
static void vtask_rc(void *pvParameters)
{
	BaseType_t xStatus;
	uint16_t rc_local_values[RC_CHANNELS] = {0};

	for (;;)
	{
		xStatus = xQueueReceive(xQueueRC, &rc_local_values, portMAX_DELAY);

		if (xStatus == pdPASS)
		{
			// Aplica o valor do canal 3 (throttle) a todos os 4 motores
			uint16_t throttle_value = rc_local_values[2];
			if (throttle_value >= 990 && throttle_value <= 2900) {
				uint16_t pwm_value = map_rc_to_pwm(throttle_value);
				uint8_t pwm_value_8bit = map_rc_to_pwm_8bit(throttle_value);

				USART_send_string("Motor 1 e 2 (PB1/PB2) - Timer1 OC1AB: ");
				USART_send_int(pwm_value);
				USART_send_string("\r\n");

				uint8_t m1a2_fineTunning = pwm_value+1;
				
				// Motor 1 (PB1) - Timer1 OC1A
				OCR1A = m1a2_fineTunning;
				
				// Motor 2 (PB2) - Timer1 OC1B
				OCR1B = m1a2_fineTunning;

				// Motor 3 (PD3) - Timer2 OC2B
				OCR2B = pwm_value_8bit;
				
				// Motor 4 (PB3) - Timer2 OC2A
				uint8_t m4_fineTunning = pwm_value_8bit+6;
				if (m4_fineTunning > 34) {
		uint16_t last_throttle = 0;
		uint16_t current_throttle = rc_local_values[0];
		if (abs((int)current_throttle - (int)last_throttle) > THROTTLE_THRESHOLD) {
			last_throttle = current_throttle;
		}
		vTaskDelay(pdMS_TO_TICKS(20)); // Ajuste o tempo conforme necessário
					m4_fineTunning = 34; // Limita o valor máximo para evitar overflow
				}
				OCR2A = m4_fineTunning;

				
			}
			
			// Exibe todos os canais via USART
			USART_send_string("CH1: ");
			USART_send_int(rc_local_values[0]);
			USART_send_string(" | CH2: ");
			USART_send_int(rc_local_values[1]);
			USART_send_string(" | CH3: ");
			USART_send_int(rc_local_values[2]);
			USART_send_string(" | CH4: ");
			USART_send_int(rc_local_values[3]);
			USART_send_string(" | M1: ");
			USART_send_int(OCR1A);
			USART_send_string(" | M2: ");
			USART_send_int(OCR1B);
			USART_send_string(" | M3: ");
			USART_send_int(OCR2A);
			USART_send_string(" | M4: ");
			USART_send_int(OCR2B);
			USART_send_string("\r\n");
		}

		vTaskDelay(pdMS_TO_TICKS(50)); // Atualiza a cada 50ms para resposta mais rápida
	}
}

// ISR do overflow do Timer0 para criar um timer virtual de 16 bits
ISR(TIMER0_OVF_vect)
{
	timer0_overflow_count++;
}

// Função para obter o tempo atual em microssegundos (resolução de 0.5 us)
uint32_t get_timer0_microseconds(void)
{
	uint8_t tcnt0_val;
	uint16_t overflow_count;
	
	// Leitura atômica para evitar problemas de concorrência
	cli();
	tcnt0_val = TCNT0;
	overflow_count = timer0_overflow_count;
	sei();
	
	// Calcula o tempo total em ticks (cada tick = 0.5 us)
	uint32_t total_ticks = ((uint32_t)overflow_count << 8) + tcnt0_val;
	
	// Converte para microssegundos (divide por 2, pois cada tick = 0.5 us)
	return total_ticks >> 1;
}

// Função para mapear valores RC (1000-2000us) para valores PWM (OCR1A)
uint16_t map_rc_to_pwm(uint16_t rc_value) {
	// Limita os valores de entrada
	if (rc_value < 1000) rc_value = 1000;
	if (rc_value > 2000) rc_value = 2000;
	
	// Mapeia 1000-2000μs para valores OCR apropriados no Timer1 (50Hz)
	// ICR1 = 1249, período = 20ms
	// 1000μs → 1ms = 5% duty cycle = T1DUTY_MIN (1000/20000 * 1250 = T1DUTY_MIN)
	// 2000μs → 2ms = 10% duty cycle = T1DUTY_MAX (2000/20000 * 1250 = T1DUTY_MAX)
	uint16_t pwm_value = ((uint32_t)(rc_value - 1000) * T1DUTY_MIN) / 1000 + T1DUTY_MIN;

	// Garante limites seguros (1-2ms para ESCs)
	if (pwm_value > T1DUTY_MAX) {
		pwm_value = T1DUTY_MAX;
	}
	if (pwm_value < T1DUTY_MIN) {
		pwm_value = T1DUTY_MIN;
	}
	
	return pwm_value;
}

// Função para mapear valores RC para PWM de 8 bits (Timer2)
uint8_t map_rc_to_pwm_8bit(uint16_t rc_value) {
	// Limita os valores de entrada
	if (rc_value < 1000) rc_value = 1000;
	if (rc_value > 2000) rc_value = 2000;
	
	// Mapeia 1000-2000μs para valores OCR apropriados no Timer2 (~61Hz)
	// Timer2 8-bit: TOP = 255, período ≈ 16.384ms
	// 1000μs → ~6% duty cycle = T2DUTY_MIN (1000/16384 * 256 ≈ T2DUTY_MIN)
	// 2000μs → ~12% duty cycle = T2DUTY_MAX (2000/16384 * 256 ≈ T2DUTY_MAX)
	uint8_t pwm_value = ((uint32_t)(rc_value - 1000) * 16) / 1000 + T2DUTY_MIN;

	// Garante limites seguros
	if (pwm_value > T2DUTY_MAX) {
		pwm_value = T2DUTY_MAX;
	}
	if (pwm_value < T2DUTY_MIN) {
		pwm_value = T2DUTY_MIN;
	}
	
	return pwm_value;
}

// Interrupção externa INT0 para capturar o sinal PPM do receptor RC no PD2
ISR(INT0_vect)
{
	static uint32_t last_time = 0;
	static uint8_t current_channel = 0;
	static uint16_t rc_values[RC_CHANNELS] = {0};

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint32_t current_time = get_timer0_microseconds(); // Obtém tempo atual em microssegundos
	uint32_t pulse_width = current_time - last_time;   // Calcula a largura do pulso
	last_time = current_time;                          // Atualiza o último tempo

	// Verifica se é um pulso de sincronização (> 3ms)
	if (pulse_width > PPM_SYNC_TIME)
	{
		// Envia os valores dos canais para a fila apenas se temos dados válidos
		if (current_channel > 0) {
			xQueueSendFromISR(xQueueRC, &rc_values, &xHigherPriorityTaskWoken);
		}
		current_channel = 0; // Reseta para o canal 0
	}
	else if (current_channel < RC_CHANNELS && pulse_width > 500 && pulse_width < 2500)
	{
		// Filtra apenas pulsos válidos (entre 0.5ms e 2.5ms)
		rc_values[current_channel] = (uint16_t)pulse_width;
		current_channel++;
	}
	
	// Chama o scheduler do FreeRTOS se necessário
	if (xHigherPriorityTaskWoken == pdTRUE) {
		taskYIELD();
	}
}

