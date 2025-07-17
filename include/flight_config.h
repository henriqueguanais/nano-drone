#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

// Configurações dos Controladores PID
#define PID_PITCH_KP    0.8
#define PID_PITCH_KI    0.0    // Começar com 0 para evitar windup
#define PID_PITCH_KD    0.08
#define PID_PITCH_MIN   -400.0
#define PID_PITCH_MAX   400.0

#define PID_ROLL_KP     0.8
#define PID_ROLL_KI     0.0    // Começar com 0 para evitar windup
#define PID_ROLL_KD     0.08
#define PID_ROLL_MIN    -400.0
#define PID_ROLL_MAX    400.0

#define PID_YAW_KP      0.3
#define PID_YAW_KI      0.0    // Começar com 0 para evitar windup
#define PID_YAW_KD      0.03
#define PID_YAW_MIN     -200.0
#define PID_YAW_MAX     200.0

// Limites de Setpoints de Segurança
#define MAX_PITCH_ANGLE     30      // ±30 graus
#define MAX_ROLL_ANGLE      30      // ±30 graus
#define MAX_YAW_RATE        100     // ±100 graus/s

// Configurações de Armamento
#define ARM_CHANNEL         4       // Canal 5 (array index 4)
#define ARM_THRESHOLD       1500    // Valor RC para armamento
#define ARM_THROTTLE_MAX    1100    // Throttle máximo para permitir armamento

// Configurações de Mapeamento RC
#define RC_CENTER_VALUE     1500    // Valor central do RC (neutro)
#define RC_PITCH_SCALE      10      // Divisor para conversão (1500±500)/10 = ±50°
#define RC_ROLL_SCALE       10      // Divisor para conversão (1500±500)/10 = ±50°
#define RC_YAW_SCALE        5       // Divisor para conversão (1500±500)/5 = ±100°/s

// Configurações de Task
#define TASK_FLIGHT_FREQ    50      // 50Hz = 20ms
#define TASK_IMU_FREQ       50      // 50Hz = 20ms
#define TASK_RC_FREQ        20      // 20Hz = 50ms

// Configurações de Debug
#define DEBUG_FLIGHT_DIVIDER    10  // Debug a cada 10 ciclos (200ms)
#define DEBUG_IMU_DIVIDER       50  // Debug a cada 50 ciclos (1s)

// Configurações de Segurança
#define THROTTLE_MIN        1000    // Valor mínimo de throttle
#define THROTTLE_MAX        2000    // Valor máximo de throttle
#define MOTOR_IDLE_VALUE    1000    // Valor PWM para motor parado

// Configurações do Filtro Complementar
#define COMPLEMENTARY_FILTER_ALPHA  0.95    // Peso do giroscópio (0.9-0.99)
#define GYRO_SENSITIVITY            131.0   // LSB/°/s para ±250°/s
#define ACCEL_SENSITIVITY           16384.0 // LSB/g para ±2g
#define FILTER_DT                   0.02    // Delta time em segundos (50Hz)

// Configurações de Calibração
#define CALIBRATION_SAMPLES         100     // Amostras para calibração
#define GYRO_BIAS_THRESHOLD         10      // Limite para detectar drone parado

#endif // FLIGHT_CONFIG_H
