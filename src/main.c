#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h> // Para abs()

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

// Constantes para estabilização PID
#define KP_ROLL 1.0f   // Ganho proporcional para roll
#define KI_ROLL 0.1f   // Ganho integral para roll  
#define KD_ROLL 0.05f  // Ganho derivativo para roll
#define KP_PITCH 1.0f  // Ganho proporcional para pitch
#define KI_PITCH 0.1f  // Ganho integral para pitch
#define KD_PITCH 0.05f // Ganho derivativo para pitch

#define MAX_ANGLE_CORRECTION 200 // Máxima correção em unidades PWM

void setup(void);
static void vtask_rc(void *pvParameters);
static void vtask_mpu6050(void *pvParameters);
uint16_t map_rc_to_pwm(uint16_t rc_value);
uint8_t map_rc_to_pwm_8bit(uint16_t rc_value);
uint32_t get_timer0_microseconds(void);
QueueHandle_t xQueueRC;

// Variáveis globais para estabilização
volatile int16_t current_roll_angle = 0;   // Ângulo atual de roll em décimos de grau
volatile int16_t current_pitch_angle = 0;  // Ângulo atual de pitch em décimos de grau
volatile int16_t roll_correction = 0;      // Correção PID para roll
volatile int16_t pitch_correction = 0;     // Correção PID para pitch

// Variáveis PID para roll
static int32_t roll_integral = 0;
static int16_t roll_last_error = 0;

// Variáveis PID para pitch  
static int32_t pitch_integral = 0;
static int16_t pitch_last_error = 0;

// Variável para criar timer virtual de 16 bits com Timer0 de 8 bits
volatile uint16_t timer0_overflow_count = 0;

// Função que define os pinos de entrada do receptor RC e configura as interrupções
void setup()
{
	// Configura direção dos pinos
	DDRD = 0x00; // Configura PORTD como entrada

	DDRD &= ~(1 << PD2);									   // Garante que PD2 seja entrada (receptor PPM)
	DDRD |= (1 << PD3);										   // Define PD3 (OC2B) como saída para PWM - Motor 4
	DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB5); // Define PB1 (OC1A), PB2 (OC1B), PB3 (OC2A) como saída para PWM e PB5 para LED

	// Configura Timer1 para Fast PWM (TOP = ICR1), 50 Hz
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Fast PWM, modo 14, habilita OC1A e OC1B
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS12);	   // Prescaler 256
	ICR1 = 1249;										   // 50 Hz (16MHz / (256 * 1250) = 50Hz)
	OCR1A = 62;											   // 1000μs (1ms) - Motor 1 (PB1) - posição de armamento ESC
	OCR1B = 62;											   // 1000μs (1ms) - Motor 2 (PB2) - posição de armamento ESC
	TIMSK1 = 0x00;										   // Desabilita interrupções do Timer1

	// Configura Timer2 para Fast PWM, ~61Hz (mais próximo possível de 50Hz com Timer 8-bit)
	// Frequência = 16MHz / (prescaler × 256) = 16MHz / (1024 × 256) = ~61Hz
	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM mode
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);					  // Prescaler 1024
	OCR2A = 15;															  // Motor 3 (PB3) - ~1000μs (1ms) - posição de armamento ESC
	OCR2B = 15;															  // Motor 4 (PD3) - ~1000μs (1ms) - posição de armamento ESC
	TIMSK2 = 0x00;														  // Desabilita interrupções do Timer2

	// Configura Timer0 para contagem de tempo (PPM timing)
	TCCR0A = 0x00;		   // Normal mode
	TCCR0B = 0x02;		   // Prescaler 8 (1 tick = 0.5 us)
	TIMSK0 = (1 << TOIE0); // Habilita interrupção de overflow do Timer0

	// Configura INT0 (PD2) para borda de subida
	EICRA |= (1 << ISC01) | (1 << ISC00); // INT0 na borda de subida
	EIMSK |= (1 << INT0);				  // Habilita INT0

	sei();

	USART_init(MYUBRR);
	mpu6050_init();
}

int main(void)
{
	setup();

	xQueueRC = xQueueCreate(1, sizeof(uint16_t) * RC_CHANNELS);

	xTaskCreate(vtask_rc, (const char *)"serial", 128, NULL, 1, NULL);
	xTaskCreate(vtask_mpu6050, (const char *)"mpu6050", 192, NULL, 1, NULL);
	vTaskStartScheduler();
	for (;;)
		;
}

const int16_t atan_lookup[51] = {
	0, 57, 114, 171, 228, 284, 340, 395, 449, 503,				// 0.0-0.9
	557, 610, 662, 714, 765, 816, 866, 916, 965, 1013,			// 1.0-1.9
	1061, 1107, 1154, 1199, 1244, 1288, 1332, 1376, 1419, 1462, // 2.0-2.9
	1504, 1546, 1588, 1629, 1670, 1711, 1751, 1791, 1831, 1871, // 3.0-3.9
	1910, 1949, 1988, 2027, 2065, 2103, 2141, 2179, 2217, 2254, // 4.0-4.9
	2291														// 5.0
};

int16_t fast_atan2_degrees(int16_t y, int16_t z)
{
	if (z == 0)
		return (y > 0) ? 900 : -900; // ±90°

	// Calcula a razão absoluta
	int32_t ratio_abs = (y >= 0 ? y : -y) * 10L;
	int32_t z_abs = (z >= 0 ? z : -z);
	ratio_abs = ratio_abs / z_abs;

	int16_t angle = atan_lookup[ratio_abs];

	// Ajusta o sinal baseado nos quadrantes
	if (z < 0)
		angle = 1800 - angle; // 180° - angle
	if (y < 0)
		angle = -angle;

	return angle; // Retorna em décimos de grau
}

// Função para calcular controle PID para estabilização
int16_t calculate_pid(int16_t setpoint, int16_t current_value, int32_t *integral, int16_t *last_error, float kp, float ki, float kd)
{
	// Calcula o erro (setpoint - valor atual)
	int16_t error = setpoint - current_value;
	
	// Termo proporcional
	int32_t proportional = (int32_t)(kp * 10.0f) * error / 10;
	
	// Termo integral (com limite para evitar windup)
	*integral += error;
	if (*integral > 1000) *integral = 1000;
	if (*integral < -1000) *integral = -1000;
	int32_t integral_term = (int32_t)(ki * 10.0f) * (*integral) / 10;
	
	// Termo derivativo
	int16_t derivative = error - *last_error;
	*last_error = error;
	int32_t derivative_term = (int32_t)(kd * 10.0f) * derivative / 10;
	
	// Saída PID
	int32_t output = proportional + integral_term + derivative_term;
	
	// Limita a saída
	if (output > MAX_ANGLE_CORRECTION) output = MAX_ANGLE_CORRECTION;
	if (output < -MAX_ANGLE_CORRECTION) output = -MAX_ANGLE_CORRECTION;
	
	return (int16_t)output;
}

static void vtask_mpu6050(void *pvParameters)
{
	mpu6050_data_t mpu_data;
	uint32_t sample_count = 0;
	uint8_t led_state = 0;

	// Debug: Task iniciada

	if (!mpu6050_test_connection())
	{
		// Se não conseguir conectar, envia erro pela serial
		USART_send_string("ERRO: MPU6050 nao encontrado!\r\n");
		USART_send_string("Verifique as conexoes I2C (SDA/SCL)\r\n");
		USART_send_string("LED piscando rapidamente...\r\n");
		// E pisca o LED rapidamente
		for (;;)
		{
			PORTB ^= (1 << PB5);
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}

	for (;;)
	{
		// Controle do LED (pisca a cada leitura)
		led_state = !led_state;
		if (led_state)
		{
			PORTB |= (1 << PB5); // Liga LED
		}
		else
		{
			PORTB &= ~(1 << PB5); // Desliga LED
		}
		
		// Lê todos os dados do MPU6050
		mpu6050_read_all(&mpu_data);
		sample_count++;

		// Calcula ângulos usando tabela de lookup (sem math.h)
		int16_t roll_tenths = fast_atan2_degrees(mpu_data.accel_x, mpu_data.accel_z);
		int16_t pitch_tenths = fast_atan2_degrees(-mpu_data.accel_y, mpu_data.accel_z);

		// Atualiza ângulos globais para a task de controle
		current_roll_angle = roll_tenths;
		current_pitch_angle = pitch_tenths;

		// Calcula correções PID (setpoint = 0 para nível)
		roll_correction = calculate_pid(0, roll_tenths, &roll_integral, &roll_last_error, KP_ROLL, KI_ROLL, KD_ROLL);
		pitch_correction = calculate_pid(0, pitch_tenths, &pitch_integral, &pitch_last_error, KP_PITCH, KI_PITCH, KD_PITCH);

		// Debug: Envia dados do MPU6050 pela USART a cada 10 amostras
		if (sample_count % 10 == 0) {
			USART_send_string("[STAB] Roll:"); USART_send_int(roll_tenths / 10);
			USART_send_string("."); USART_send_int(abs(roll_tenths % 10));
			USART_send_string(" Pitch:"); USART_send_int(pitch_tenths / 10);
			USART_send_string("."); USART_send_int(abs(pitch_tenths % 10));
			USART_send_string(" RollCorr:"); USART_send_int(roll_correction);
			USART_send_string(" PitchCorr:"); USART_send_int(pitch_correction);
			USART_send_string("\r\n");
		}

		// Executa a 100Hz para boa resposta de estabilização
		vTaskDelay(pdMS_TO_TICKS(10));
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
			// Controle dos eixos PITCH (CH1) e ROLL (CH2) com estabilização
			uint16_t throttle = rc_local_values[2]; // CH3
			int16_t pitch_cmd = rc_local_values[0] - 1500; // CH1 (centro em 1500)
			int16_t roll_cmd  = rc_local_values[1] - 1500; // CH2 (centro em 1500)
			
			if (rc_local_values[4] <= 1500) {
				throttle = 990; // Se canal auxiliar desligado, desliga motores
			}

			if (throttle >= 990 && throttle <= 2900)
			{
				// Combina comando do piloto com correção de estabilização
				// Reduz a escala dos comandos do piloto para permitir estabilização
				int16_t pitch_total = (pitch_cmd / 4) + pitch_correction;
				int16_t roll_total = -((roll_cmd / 4) + roll_correction); // Roll invertido

				// Aplica correções nos motores (configuração X)
				// Motor 1 (PB1): +pitch -roll
				// Motor 2 (PB2): +pitch +roll  
				// Motor 3 (PD3): -pitch +roll
				// Motor 4 (PB3): -pitch -roll
				int16_t m1 = throttle + pitch_total - roll_total;
				int16_t m2 = throttle + pitch_total + roll_total;
				int16_t m3 = throttle - pitch_total + roll_total;
				int16_t m4 = throttle - pitch_total - roll_total;

				// Mapeia para PWM seguro
				uint16_t pwm_m1 = map_rc_to_pwm(m1);
				uint16_t pwm_m2 = map_rc_to_pwm(m2);
				uint8_t  pwm_m3 = map_rc_to_pwm_8bit(m3);
				uint8_t  pwm_m4 = map_rc_to_pwm_8bit(m4);

				// Fine tuning para compensar diferenças dos motores
				uint16_t m1_fineTuning = pwm_m1 + 1; // Motor 1 ajuste
				uint16_t m2_fineTuning = pwm_m2 + 1; // Motor 2 ajuste
				
				uint8_t m4_fineTuning = pwm_m4 + 6; // Motor 4 ajuste
				if (m4_fineTuning > 34)
				{
					m4_fineTuning = 34; // Limita o valor máximo para evitar overflow
				}

				// Aplica PWM nos motores com fine tuning
				OCR1A = m1_fineTuning; // Motor 1 (PB1)
				OCR1B = m2_fineTuning; // Motor 2 (PB2)
				OCR2B = pwm_m3;        // Motor 3 (PD3) - sem ajuste
				OCR2A = m4_fineTuning; // Motor 4 (PB3)

			}

			
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
uint16_t map_rc_to_pwm(uint16_t rc_value)
{
	// Limita os valores de entrada
	if (rc_value < 1000)
		rc_value = 1000;
	if (rc_value > 2000)
		rc_value = 2000;

	// Mapeia 1000-2000μs para valores OCR apropriados no Timer1 (50Hz)
	// ICR1 = 1249, período = 20ms
	// 1000μs → 1ms = 5% duty cycle = T1DUTY_MIN (1000/20000 * 1250 = T1DUTY_MIN)
	// 2000μs → 2ms = 10% duty cycle = T1DUTY_MAX (2000/20000 * 1250 = T1DUTY_MAX)
	uint16_t pwm_value = ((uint32_t)(rc_value - 1000) * T1DUTY_MIN) / 1000 + T1DUTY_MIN;

	// Garante limites seguros (1-2ms para ESCs)
	if (pwm_value > T1DUTY_MAX)
	{
		pwm_value = T1DUTY_MAX;
	}
	if (pwm_value < T1DUTY_MIN)
	{
		pwm_value = T1DUTY_MIN;
	}

	return pwm_value;
}

// Função para mapear valores RC para PWM de 8 bits (Timer2)
uint8_t map_rc_to_pwm_8bit(uint16_t rc_value)
{
	// Limita os valores de entrada
	if (rc_value < 1000)
		rc_value = 1000;
	if (rc_value > 2000)
		rc_value = 2000;

	// Mapeia 1000-2000μs para valores OCR apropriados no Timer2 (~61Hz)
	// Timer2 8-bit: TOP = 255, período ≈ 16.384ms
	// 1000μs → ~6% duty cycle = T2DUTY_MIN (1000/16384 * 256 ≈ T2DUTY_MIN)
	// 2000μs → ~12% duty cycle = T2DUTY_MAX (2000/16384 * 256 ≈ T2DUTY_MAX)
	uint8_t pwm_value = ((uint32_t)(rc_value - 1000) * 16) / 1000 + T2DUTY_MIN;

	// Garante limites seguros
	if (pwm_value > T2DUTY_MAX)
	{
		pwm_value = T2DUTY_MAX;
	}
	if (pwm_value < T2DUTY_MIN)
	{
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
	last_time = current_time;						   // Atualiza o último tempo

	// Verifica se é um pulso de sincronização (> 3ms)
	if (pulse_width > PPM_SYNC_TIME)
	{
		// Envia os valores dos canais para a fila apenas se temos dados válidos
		if (current_channel > 0)
		{
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
	if (xHigherPriorityTaskWoken == pdTRUE)
	{
		taskYIELD();
	}
}
