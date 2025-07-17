#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>  // Para abs()

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "USART.h"
#include "mpu6050.h"
#include "flight_config.h"

#define RC_CHANNELS 6
#define PPM_SYNC_TIME 3000 // Tempo de sincronização do PPM em microssegundos

// Estrutura para controle PID
typedef struct {
    float kp, ki, kd;          // Ganhos PID
    float previous_error;      // Erro anterior para derivativo
    float integral;            // Soma dos erros para integral
    float output_min, output_max; // Limites de saída
    uint32_t last_time;        // Timestamp da última execução
} pid_controller_t;

// Estrutura para dados de controle de voo
typedef struct {
    int16_t throttle;          // Throttle do RC
    int16_t pitch_setpoint;    // Setpoint de pitch (normalmente 0)
    int16_t roll_setpoint;     // Setpoint de roll (normalmente 0) 
    int16_t yaw_rate_setpoint; // Setpoint de taxa de yaw
    uint8_t armed;             // Status de armamento
} flight_control_t;

void setup(void);
static void vtask_rc(void *pvParameters);
static void vtask_mpu6050(void *pvParameters);
static void vtask_flight_control(void *pvParameters);
uint16_t map_rc_to_pwm(uint16_t rc_value);
uint8_t map_rc_to_pwm_8bit(uint16_t rc_value);
uint32_t get_timer0_microseconds(void);
float pid_calculate(pid_controller_t *pid, float setpoint, float measured_value, uint32_t current_time);
void update_motor_control(flight_control_t *flight_ctrl, float pitch_correction, float roll_correction, float yaw_correction);
void init_pid_controllers(void);

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
QueueHandle_t xQueueIMU;        // Fila para dados do IMU
QueueHandle_t xQueueFlightCtrl; // Fila para dados de controle de voo

// Variável para criar timer virtual de 16 bits com Timer0 de 8 bits
volatile uint16_t timer0_overflow_count = 0;

// Controladores PID globais
pid_controller_t pid_pitch;
pid_controller_t pid_roll;
pid_controller_t pid_yaw;

int main(void)
{
	setup();

	// Inicializa controladores PID
	init_pid_controllers();

	// Cria filas
	xQueueRC = xQueueCreate(1, sizeof(uint16_t) * RC_CHANNELS);
	xQueueIMU = xQueueCreate(1, sizeof(mpu6050_data_t));
	xQueueFlightCtrl = xQueueCreate(1, sizeof(flight_control_t));

	// Cria tasks
	xTaskCreate(vtask_rc, (const char *)"rc", 128, NULL, 1, NULL);
	xTaskCreate(vtask_mpu6050, (const char *)"mpu6050", 192, NULL, 2, NULL);
	xTaskCreate(vtask_flight_control, (const char *)"flight", 256, NULL, 3, NULL);
	
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

// Inicializa os controladores PID com valores conservadores
void init_pid_controllers(void) {
	// PID para Pitch (eixo Y)
	pid_pitch.kp = PID_PITCH_KP;
	pid_pitch.ki = PID_PITCH_KI;
	pid_pitch.kd = PID_PITCH_KD;
	pid_pitch.previous_error = 0;
	pid_pitch.integral = 0;
	pid_pitch.output_min = PID_PITCH_MIN;
	pid_pitch.output_max = PID_PITCH_MAX;
	pid_pitch.last_time = 0;
	
	// PID para Roll (eixo X)
	pid_roll.kp = PID_ROLL_KP;
	pid_roll.ki = PID_ROLL_KI;
	pid_roll.kd = PID_ROLL_KD;
	pid_roll.previous_error = 0;
	pid_roll.integral = 0;
	pid_roll.output_min = PID_ROLL_MIN;
	pid_roll.output_max = PID_ROLL_MAX;
	pid_roll.last_time = 0;
	
	// PID para Yaw (taxa de rotação Z)
	pid_yaw.kp = PID_YAW_KP;
	pid_yaw.ki = PID_YAW_KI;
	pid_yaw.kd = PID_YAW_KD;
	pid_yaw.previous_error = 0;
	pid_yaw.integral = 0;
	pid_yaw.output_min = PID_YAW_MIN;
	pid_yaw.output_max = PID_YAW_MAX;
	pid_yaw.last_time = 0;
}

static void vtask_mpu6050(void *pvParameters)
{
	mpu6050_data_t mpu_data;
	complementary_filter_t filter;
	uint32_t sample_count = 0;
	uint8_t led_state = 0;
	
	// Inicializa filtro complementar
	complementary_filter_init(&filter);
	
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
	
	USART_send_string("IMU inicializado. Calibrando...\r\n");
	
	for (;;)
	{
		// Controle do LED (pisca a cada 25 leituras = ~0.5s)
		if (++sample_count % 25 == 0) {
			led_state = !led_state;
			if (led_state) {
				PORTB |= (1 << PB5);  // Liga LED
			} else {
				PORTB &= ~(1 << PB5); // Desliga LED
			}
		}
		
		// Lê todos os dados do MPU6050
		if (mpu6050_read_all(&mpu_data)) {
			// Fase de calibração
			if (!filter.calibrated) {
				complementary_filter_calibrate(&filter, &mpu_data);
				if (filter.calibrated) {
					USART_send_string("Calibracao concluida! Pronto para voo.\r\n");
				}
			} else {
				// Filtro complementar para ângulos precisos
				complementary_filter_update(&filter, &mpu_data);
			}
			
			// Envia dados para fila do controle de voo
			xQueueSend(xQueueIMU, &mpu_data, 0);
			
			// Debug a cada 1 segundo (50 samples)
			if (sample_count % DEBUG_IMU_DIVIDER == 0) {
				USART_send_string("Roll: ");
				USART_send_int((int16_t)mpu_data.angle_x);
				USART_send_string("° | Pitch: ");
				USART_send_int((int16_t)mpu_data.angle_y);
				USART_send_string("° | YawRate: ");
				USART_send_int((int16_t)mpu_data.angle_z);
				USART_send_string("°/s");
				
				// Indicador de calibração
				if (!filter.calibrated) {
					USART_send_string(" [CAL]");
				}
				USART_send_string("\r\n");
			}
		}

		// Task roda a 50Hz (20ms) para controle preciso
		vTaskDelay(pdMS_TO_TICKS(1000/TASK_IMU_FREQ));
	}
}

// Função de envio dos valores do receptor RC via USART
static void vtask_rc(void *pvParameters)
{
	BaseType_t xStatus;
	uint16_t rc_local_values[RC_CHANNELS] = {0};
	flight_control_t flight_ctrl = {0};
	
	// Inicializa estrutura de controle de voo
	flight_ctrl.throttle = 1000;
	flight_ctrl.pitch_setpoint = 0;
	flight_ctrl.roll_setpoint = 0;
	flight_ctrl.yaw_rate_setpoint = 0;
	flight_ctrl.armed = 0;

	for (;;)
	{
		xStatus = xQueueReceive(xQueueRC, &rc_local_values, portMAX_DELAY);

		if (xStatus == pdPASS)
		{
			// Mapeia canais RC para controle de voo
			flight_ctrl.throttle = rc_local_values[2];           // Canal 3 (throttle)
			flight_ctrl.pitch_setpoint = (rc_local_values[1] - RC_CENTER_VALUE) / RC_PITCH_SCALE;
			flight_ctrl.roll_setpoint = (rc_local_values[0] - RC_CENTER_VALUE) / RC_ROLL_SCALE;
			flight_ctrl.yaw_rate_setpoint = (rc_local_values[3] - RC_CENTER_VALUE) / RC_YAW_SCALE;
			
			// Lógica de armamento
			if (rc_local_values[ARM_CHANNEL] > ARM_THRESHOLD && flight_ctrl.throttle < ARM_THROTTLE_MAX) {
				flight_ctrl.armed = 1;
			} else if (rc_local_values[ARM_CHANNEL] < ARM_THRESHOLD) {
				flight_ctrl.armed = 0;
			}
			
			// Limita setpoints para segurança
			if (flight_ctrl.pitch_setpoint > MAX_PITCH_ANGLE) flight_ctrl.pitch_setpoint = MAX_PITCH_ANGLE;
			if (flight_ctrl.pitch_setpoint < -MAX_PITCH_ANGLE) flight_ctrl.pitch_setpoint = -MAX_PITCH_ANGLE;
			if (flight_ctrl.roll_setpoint > MAX_ROLL_ANGLE) flight_ctrl.roll_setpoint = MAX_ROLL_ANGLE;
			if (flight_ctrl.roll_setpoint < -MAX_ROLL_ANGLE) flight_ctrl.roll_setpoint = -MAX_ROLL_ANGLE;
			if (flight_ctrl.yaw_rate_setpoint > MAX_YAW_RATE) flight_ctrl.yaw_rate_setpoint = MAX_YAW_RATE;
			if (flight_ctrl.yaw_rate_setpoint < -MAX_YAW_RATE) flight_ctrl.yaw_rate_setpoint = -MAX_YAW_RATE;
			
			// Envia dados para task de controle de voo
			xQueueSend(xQueueFlightCtrl, &flight_ctrl, 0);
			
			// Debug via USART
			USART_send_string("RC: T=");
			USART_send_int(flight_ctrl.throttle);
			USART_send_string(" P=");
			USART_send_int(flight_ctrl.pitch_setpoint);
			USART_send_string(" R=");
			USART_send_int(flight_ctrl.roll_setpoint);
			USART_send_string(" Y=");
			USART_send_int(flight_ctrl.yaw_rate_setpoint);
			USART_send_string(" ARM=");
			USART_send_int(flight_ctrl.armed);
			USART_send_string("\r\n");
		}

		vTaskDelay(pdMS_TO_TICKS(1000/TASK_RC_FREQ)); // Atualiza a cada 50ms para resposta mais rápida
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
	// 1000μs → 1ms = 5% duty cycle = 62 (1000/20000 * 1250 = 62.5)
	// 2000μs → 2ms = 10% duty cycle = 125 (2000/20000 * 1250 = 125)
	uint16_t pwm_value = ((uint32_t)(rc_value - 1000) * 62) / 1000 + 62;

	// Garante limites seguros (1-2ms para ESCs)
	if (pwm_value > 125) {
		pwm_value = 125;
	}
	if (pwm_value < 62) {
		pwm_value = 62;
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
	// 1000μs → ~6% duty cycle = 15 (1000/16384 * 256 ≈ 15.6)
	// 2000μs → ~12% duty cycle = 31 (2000/16384 * 256 ≈ 31.2)
	uint8_t pwm_value = ((uint32_t)(rc_value - 1000) * 16) / 1000 + 15;
	
	// Garante limites seguros
	if (pwm_value > 31) {
		pwm_value = 31;
	}
	if (pwm_value < 15) {
		pwm_value = 15;
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

// Função PID para cálculo de correção
float pid_calculate(pid_controller_t *pid, float setpoint, float measured_value, uint32_t current_time) {
	float error = setpoint - measured_value;
	float dt = (current_time - pid->last_time) / 1000.0; // Converte para segundos
	
	// Evita divisão por zero na primeira execução
	if (pid->last_time == 0 || dt <= 0) {
		pid->last_time = current_time;
		pid->previous_error = error;
		return 0;
	}
	
	// Termo proporcional
	float p_term = pid->kp * error;
	
	// Termo integral (com limitação anti-windup)
	pid->integral += error * dt;
	float i_term = pid->ki * pid->integral;
	
	// Termo derivativo
	float d_term = pid->kd * (error - pid->previous_error) / dt;
	
	// Saída total
	float output = p_term + i_term + d_term;
	
	// Limitação da saída
	if (output > pid->output_max) {
		output = pid->output_max;
		// Anti-windup: reduz integral se saída saturou
		if (pid->ki > 0) {
			pid->integral = (pid->output_max - p_term - d_term) / pid->ki;
		}
	}
	if (output < pid->output_min) {
		output = pid->output_min;
		// Anti-windup: reduz integral se saída saturou
		if (pid->ki > 0) {
			pid->integral = (pid->output_min - p_term - d_term) / pid->ki;
		}
	}
	
	// Atualiza valores para próxima iteração
	pid->previous_error = error;
	pid->last_time = current_time;
	
	return output;
}

// Função para atualizar controle dos motores com PID
void update_motor_control(flight_control_t *flight_ctrl, float pitch_correction, float roll_correction, float yaw_correction) {
	// Converte throttle para faixa base (1000-2000)
	int16_t base_throttle = flight_ctrl->throttle;
	
	// Cálculo individual para cada motor baseado no layout X:
	//   M1    M2
	//     \  /
	//      ><
	//     /  \
	//   M4    M3
	
	int16_t motor1 = base_throttle - (int16_t)pitch_correction - (int16_t)roll_correction - (int16_t)yaw_correction;
	int16_t motor2 = base_throttle - (int16_t)pitch_correction + (int16_t)roll_correction + (int16_t)yaw_correction;
	int16_t motor3 = base_throttle + (int16_t)pitch_correction + (int16_t)roll_correction - (int16_t)yaw_correction;
	int16_t motor4 = base_throttle + (int16_t)pitch_correction - (int16_t)roll_correction + (int16_t)yaw_correction;
	
	// Limitação de segurança
	if (motor1 < 1000) motor1 = 1000;
	if (motor1 > 2000) motor1 = 2000;
	if (motor2 < 1000) motor2 = 1000;
	if (motor2 > 2000) motor2 = 2000;
	if (motor3 < 1000) motor3 = 1000;
	if (motor3 > 2000) motor3 = 2000;
	if (motor4 < 1000) motor4 = 1000;
	if (motor4 > 2000) motor4 = 2000;
	
	// Aplicação dos valores PWM
	OCR1A = map_rc_to_pwm(motor1);         // Motor 1 (Timer1)
	OCR1B = map_rc_to_pwm(motor2);         // Motor 2 (Timer1)
	OCR2A = map_rc_to_pwm_8bit(motor3);    // Motor 3 (Timer2)
	OCR2B = map_rc_to_pwm_8bit(motor4);    // Motor 4 (Timer2)
}

// Task principal de controle de voo
static void vtask_flight_control(void *pvParameters) {
	mpu6050_data_t imu_data;
	flight_control_t flight_ctrl = {0};
	uint32_t current_time;
	
	// Valores iniciais
	flight_ctrl.throttle = 1000;          // Throttle mínimo
	flight_ctrl.pitch_setpoint = 0;       // Nível
	flight_ctrl.roll_setpoint = 0;        // Nível
	flight_ctrl.yaw_rate_setpoint = 0;    // Sem rotação
	flight_ctrl.armed = 0;                // Desarmado
	
	for (;;) {
		// Recebe dados do IMU (non-blocking)
		if (xQueueReceive(xQueueIMU, &imu_data, 0) == pdPASS) {
			// Recebe dados de controle de voo (non-blocking)
			xQueueReceive(xQueueFlightCtrl, &flight_ctrl, 0);
			
			// Só executa PID se armado e com throttle > mínimo
			if (flight_ctrl.armed && flight_ctrl.throttle > 1050) {
				current_time = get_timer0_microseconds() / 1000; // Converte para ms
				
				// Calcula correções PID
				float pitch_correction = pid_calculate(&pid_pitch, flight_ctrl.pitch_setpoint, imu_data.angle_y, current_time);
				float roll_correction = pid_calculate(&pid_roll, flight_ctrl.roll_setpoint, imu_data.angle_x, current_time);
				float yaw_correction = pid_calculate(&pid_yaw, flight_ctrl.yaw_rate_setpoint, imu_data.angle_z, current_time);
				
				// Aplica controle aos motores
				update_motor_control(&flight_ctrl, pitch_correction, roll_correction, yaw_correction);
				
				// Debug a cada 10 ciclos (~200ms)
				static uint8_t debug_counter = 0;
				if (++debug_counter >= DEBUG_FLIGHT_DIVIDER) {
					debug_counter = 0;
					USART_send_string("PID: P=");
					USART_send_int((int16_t)pitch_correction);
					USART_send_string(" R=");
					USART_send_int((int16_t)roll_correction);
					USART_send_string(" Y=");
					USART_send_int((int16_t)yaw_correction);
					USART_send_string(" T=");
					USART_send_int(flight_ctrl.throttle);
					USART_send_string("\r\n");
				}
			} else {
				// Modo desarmado: todos os motores em mínimo
				OCR1A = 62;  // 1000μs - Motor 1
				OCR1B = 62;  // 1000μs - Motor 2
				OCR2A = 15;  // 1000μs - Motor 3
				OCR2B = 15;  // 1000μs - Motor 4
			}
		}
		
		// Task roda a 50Hz (20ms) para controle preciso
		vTaskDelay(pdMS_TO_TICKS(1000/TASK_FLIGHT_FREQ));
	}
}

