#ifndef MPU6050_H
#define MPU6050_H

#include <avr/io.h>
#include <stdint.h>

// Endereço I2C do MPU6050
#define MPU6050_ADDR 0x68

// Registradores do MPU6050
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_WHO_AM_I     0x75

// Estrutura para dados do MPU6050
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    // Ângulos calculados (adicionados para PID)
    float angle_x;  // Roll em graus
    float angle_y;  // Pitch em graus  
    float angle_z;  // Yaw rate em graus/s
} mpu6050_data_t;

// Estrutura para filtro complementar
typedef struct {
    float angle_x;          // Ângulo X filtrado
    float angle_y;          // Ângulo Y filtrado
    float gyro_x_bias;      // Offset do giroscópio X
    float gyro_y_bias;      // Offset do giroscópio Y
    float gyro_z_bias;      // Offset do giroscópio Z
    uint8_t calibrated;     // Flag de calibração
} complementary_filter_t;

// Funções do MPU6050
void mpu6050_init(void);
uint8_t mpu6050_read_register(uint8_t reg);
void mpu6050_write_register(uint8_t reg, uint8_t data);
uint8_t mpu6050_read_all(mpu6050_data_t *data);
uint8_t mpu6050_test_connection(void);  // Função de teste

// Funções do filtro complementar
void complementary_filter_init(complementary_filter_t *filter);
void complementary_filter_update(complementary_filter_t *filter, mpu6050_data_t *data);
void complementary_filter_calibrate(complementary_filter_t *filter, mpu6050_data_t *data);

// Funções I2C
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);

#endif