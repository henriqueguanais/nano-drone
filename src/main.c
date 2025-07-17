#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "USART.h"
#include "mpu6050.h"

#define RC_CHANNELS 6
#define PPM_SYNC_TIME 3000 // Tempo de sincronização do PPM em microssegundos

void setup(void);
static void vtask_rc(void *pvParameters);

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

static void vtask_mpu6050(void *pvParameters);

QueueHandle_t xQueueRC;

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
	DDRD = 0x00;
	DDRD |= (1 << PD6); // Define PD6 como saida esc

	TCCR1A = 0x00; // Normal mode
	TCCR1B = 0x02; // Prescaler 8 (1 tick = 0.5 us)
	TIMSK1 = 0x00; // Desabilita inter
	EICRA = 0x03;  // Aciona a interrupção INT0 na borda de subida
	EIMSK = 0x01;  // Habilita a interrupção INT0	PD2

	TCCR0A = 0x02; // ctc mode
	TCCR0B = 0x02; // prescaler8
	TIMSK0 = (1 << OCIE0A);     // Habilita interrupção de comparação
	OCR0A = 249;

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
		int16_t roll_tenths = fast_atan2_degrees(mpu_data.accel_y, mpu_data.accel_z);
		int16_t pitch_tenths = fast_atan2_degrees(-mpu_data.accel_x, mpu_data.accel_z);

		// Envia dados do MPU6050 pela USART
		USART_send_string("MPU6050: ");
		USART_send_string("AX:"); USART_send_int(mpu_data.accel_x);
		USART_send_string(" AY:"); USART_send_int(mpu_data.accel_y);
		USART_send_string(" AZ:"); USART_send_int(mpu_data.accel_z);
		USART_send_string(" | GX:"); USART_send_int(mpu_data.gyro_x);
		USART_send_string(" GY:"); USART_send_int(mpu_data.gyro_y);
		USART_send_string(" GZ:"); USART_send_int(mpu_data.gyro_z);
		USART_send_string(" | Roll:"); USART_send_int(roll_tenths);
		USART_send_string(" Pitch:"); USART_send_int(pitch_tenths);
		USART_send_string("\r\n");

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
			// USART_send_string("CH1: ");
			// USART_send_int(rc_local_values[0]);
			// USART_send_string("|CH2: ");
			// USART_send_int(rc_local_values[1]);
			USART_send_string("|CH3: ");
			USART_send_int(rc_local_values[2]);
			// USART_send_string("|CH4: ");
			// USART_send_int(rc_local_values[3]);
			// USART_send_string("|CH5: ");
			// USART_send_int(rc_local_values[4]);
			// USART_send_string("|CH6: ");
			// USART_send_int(rc_local_values[5]);
			USART_send_string("\r\n");
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

// Interrupção externa INT0 para capturar o sinal PPM do receptor RC
ISR(INT0_vect)
{
	static uint16_t last_time = 0;
	static uint8_t current_channel = 0;
	static uint16_t rc_values[RC_CHANNELS] = {0};

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint16_t current_time = TCNT1;					 // Captura o tempo atual do timer
	uint16_t pulse_width = current_time - last_time; // Calcula a largura do pulso
	last_time = current_time;						 // Atualiza o último tempo

	uint16_t pulse_width_us = pulse_width * 0.5; // Converte o tempo do timer para us

	if (pulse_width_us > PPM_SYNC_TIME)
	{
		xQueueSendFromISR(xQueueRC, &rc_values, &xHigherPriorityTaskWoken); // envia para fila apos ler todos os canais
		current_channel = 0;												// Reseta para o canal 0, se for pulso de sincronização
	}
	else if (current_channel < RC_CHANNELS)
	{
		rc_values[current_channel] = pulse_width_us;
		current_channel++;
	}
}

ISR(TIMER0_COMPA_vect) {
	static uint8_t count = 0;
	count++;
	if (count >= 100) {
		PORTD |= (1 << PD6);
	}
	if (count >= 200) {
		PORTD &= ~(1 << PD6);
	}

	if (count >= 2000) {
		count = 0;
	}

}