#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "USART.h"

#define RC_CHANNELS 6
#define PPM_SYNC_TIME 3000 // Tempo de sincronização do PPM em microssegundos

void setup(void);
static void vtask_rc(void *pvParameters);

QueueHandle_t xQueueRC;

int main(void)
{
	setup();

	xQueueRC = xQueueCreate(1, sizeof(uint16_t) * RC_CHANNELS);

	xTaskCreate(vtask_rc, (const char *)"serial", 128, NULL, 1, NULL);
	vTaskStartScheduler();
	for (;;);
}

// Função que define os pinos de entrada do receptor RC e configura as interrupções
void setup() {
	DDRD = 0x00;

	TCCR1A = 0x00; // Normal mode
	TCCR1B = 0x02; // Prescaler 8 (1 tick = 0.5 us)
	TIMSK1 = 0x00; // Desabilita inter
	EICRA = 0x03; // Aciona a interrupção INT0 na borda de subida
	EIMSK = 0x01; // Habilita a interrupção INT0

	sei();

	USART_init(MYUBRR);
}

// Função de envio dos valores do receptor RC via USART
static void vtask_rc(void *pvParameters)
{
	BaseType_t xStatus;
	uint16_t rc_local_values[RC_CHANNELS] = {0};

	for (;;)
	{
		xStatus = xQueueReceive(xQueueRC, &rc_local_values, portMAX_DELAY);

		if (xStatus == pdPASS) {
			USART_send_string("CH1: ");
			USART_send_int(rc_local_values[0]);
			USART_send_string("|CH2: ");
			USART_send_int(rc_local_values[1]);
			USART_send_string("|CH3: ");
			USART_send_int(rc_local_values[2]);
			USART_send_string("|CH4: ");
			USART_send_int(rc_local_values[3]);
			USART_send_string("|CH5: ");
			USART_send_int(rc_local_values[4]);
			USART_send_string("|CH6: ");
			USART_send_int(rc_local_values[5]);
			USART_send_string("\r\n");
		}
		
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

// Interrupção externa INT0 para capturar o sinal PPM do receptor RC
ISR(INT0_vect) {
	static uint16_t last_time = 0;
	static uint8_t current_channel = 0;
	static uint16_t rc_values[RC_CHANNELS] = {0};

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint16_t current_time = TCNT1; // Captura o tempo atual do timer
	uint16_t pulse_width = current_time - last_time; // Calcula a largura do pulso
	last_time = current_time; // Atualiza o último tempo

	uint16_t pulse_width_us = pulse_width * 0.5; // Converte o tempo do timer para us

	if (pulse_width_us > PPM_SYNC_TIME) {
		xQueueSendFromISR(xQueueRC, &rc_values, &xHigherPriorityTaskWoken);	// envia para fila apos ler todos os canais
		current_channel = 0; // Reseta para o canal 0, se for pulso de sincronização
	}
	else if (current_channel < RC_CHANNELS) {
		rc_values[current_channel] = pulse_width_us;
		current_channel++;
	}
}