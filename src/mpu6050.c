#include "mpu6050.h"
#include <util/delay.h>

// Inicialização do I2C
void i2c_init(void) {
    // Configurar velocidade do I2C para 100kHz com F_CPU = 16MHz
    TWSR = 0x00;  // Prescaler = 1
    TWBR = 0x48;  // Bit rate register para 100kHz
    TWCR = (1 << TWEN);  // Habilitar TWI
}

// Inicia condição START no barramento I2C
void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

// Para o barramento I2C
void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

// Escreve um byte no barramento I2C
uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return (TWSR & 0xF8);
}

// Lê um byte com ACK
uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// Lê um byte com NACK
uint8_t i2c_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

// Inicializa o MPU6050
void mpu6050_init(void) {
    i2c_init();
    _delay_ms(100);  // Aguarda o MPU6050 estabilizar
    
    // Sair do modo sleep
    mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);
    _delay_ms(10);
    
    // Configurar sample rate (1kHz)
    mpu6050_write_register(MPU6050_SMPLRT_DIV, 0x07);
    
    // Configurar filtro passa-baixa
    mpu6050_write_register(MPU6050_CONFIG, 0x06);
    
    // Configurar giroscópio (±250°/s)
    mpu6050_write_register(MPU6050_GYRO_CONFIG, 0x00);
    
    // Configurar acelerômetro (±2g)
    mpu6050_write_register(MPU6050_ACCEL_CONFIG, 0x00);
}

// Escreve em um registrador do MPU6050
void mpu6050_write_register(uint8_t reg, uint8_t data) {
    i2c_start();
    i2c_write((MPU6050_ADDR << 1) | 0);  // Endereço + write
    i2c_write(reg);                      // Registrador
    i2c_write(data);                     // Dados
    i2c_stop();
}

// Lê um registrador do MPU6050
uint8_t mpu6050_read_register(uint8_t reg) {
    uint8_t data;
    
    i2c_start();
    i2c_write((MPU6050_ADDR << 1) | 0);  // Endereço + write
    i2c_write(reg);                      // Registrador
    
    i2c_start();                         // Restart
    i2c_write((MPU6050_ADDR << 1) | 1);  // Endereço + read
    data = i2c_read_nack();              // Lê dados
    i2c_stop();
    
    return data;
}

// Lê todos os dados do MPU6050
void mpu6050_read_all(mpu6050_data_t *data) {
    uint8_t raw_data[14];
    
    i2c_start();
    i2c_write((MPU6050_ADDR << 1) | 0);  // Endereço + write
    i2c_write(MPU6050_ACCEL_XOUT_H);     // Primeiro registrador
    
    i2c_start();                         // Restart
    i2c_write((MPU6050_ADDR << 1) | 1);  // Endereço + read
    
    // Lê 14 bytes consecutivos
    for (uint8_t i = 0; i < 13; i++) {
        raw_data[i] = i2c_read_ack();
    }
    raw_data[13] = i2c_read_nack();      // Último byte com NACK
    i2c_stop();
    
    // Converte os dados para int16_t
    data->accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    data->accel_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    data->accel_z = (int16_t)(raw_data[4] << 8 | raw_data[5]);
    data->temp = (int16_t)(raw_data[6] << 8 | raw_data[7]);
    data->gyro_x = (int16_t)(raw_data[8] << 8 | raw_data[9]);
    data->gyro_y = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    data->gyro_z = (int16_t)(raw_data[12] << 8 | raw_data[13]);
}

// Testa a conexão com o MPU6050
uint8_t mpu6050_test_connection(void) {
    uint8_t who_am_i = mpu6050_read_register(MPU6050_WHO_AM_I);
    return (who_am_i == 0x68);  // Valor esperado do WHO_AM_I
}