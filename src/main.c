#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"
#include "task.h"
#include "USART.h"

void setup(void);
static void vtask_led(void *pvParameters);
static void vtask_serial(void *pvParameters);

int main(void)
{
	setup();

	xTaskCreate(vtask_led, (const char *)"led", 256, NULL, 1, NULL);
	xTaskCreate(vtask_serial, (const char *)"serial", 256, NULL, 2, NULL);
	vTaskStartScheduler();
	for (;;);
}

void setup() {
	PORTB = 0x00;
	USART_init(MYUBRR);
}

static void vtask_led(void *pvParameters)
{
	for (;;)
	{
		PORTB ^= (1 << PB5); 
		vTaskDelay(10);
	}
}

static void vtask_serial(void *pvParameters)
{
	for (;;)
	{
		USART_send_string("Pisquei\r\n");
		vTaskDelay(20);
	}
}
