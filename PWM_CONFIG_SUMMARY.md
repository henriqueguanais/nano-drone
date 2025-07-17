# Configuração PWM do Nano-Drone - ATmega328P

## Resumo da Configuração Atual

### Hardware PWM - 4 Canais Utilizados
O ATmega328P possui apenas 4 pinos de hardware PWM, todos estão sendo utilizados:

#### Timer1 (16-bit) - 50Hz Exato
- **PB1 (OC1A)** - Motor 1
- **PB2 (OC1B)** - Motor 2
- **Configuração**: Fast PWM mode 14, ICR1=1249, prescaler=256
- **Frequência**: 16MHz ÷ (256 × 1250) = 50Hz
- **Resolução**: 1250 steps (0-1249)
- **Faixa PWM**: 1000-2000μs → OCR = 62-125

#### Timer2 (8-bit) - ~61Hz
- **PB3 (OC2A)** - Motor 3  
- **PD3 (OC2B)** - Motor 4
- **Configuração**: Fast PWM mode, prescaler=1024
- **Frequência**: 16MHz ÷ (1024 × 256) = ~61Hz
- **Resolução**: 256 steps (0-255)
- **Faixa PWM**: 1000-2000μs → OCR = 15-31

### Mapeamento de Valores RC para PWM

#### Timer1 (16-bit):
```c
// 1000-2000μs → 62-125 (1ms-2ms, duty cycle 5%-10%)
uint16_t pwm_value = ((uint32_t)(rc_value - 1000) * 62) / 1000 + 62;
```

#### Timer2 (8-bit):
```c
// 1000-2000μs → 15-31 (1ms-2ms, duty cycle ~6%-12%)
uint8_t pwm_value = ((uint32_t)(rc_value - 1000) * 16) / 1000 + 15;
```

### Controle de Motores
- **Entrada**: RC Canal 3 (throttle) via PPM em PD2
- **Saída**: Todos os 4 motores recebem o mesmo valor de throttle
- **Armamento**: Valores iniciais em 1ms (OCR1A=62, OCR1B=62, OCR2A=15, OCR2B=15)

### Pinos Utilizados
- **PD2**: INT0 - Entrada PPM do receptor RC
- **PB1**: OC1A - Motor 1 (Timer1)
- **PB2**: OC1B - Motor 2 (Timer1)  
- **PB3**: OC2A - Motor 3 (Timer2)
- **PD3**: OC2B - Motor 4 (Timer2)
- **PB5**: LED de status
- **PC4**: SDA (I2C - MPU6050)
- **PC5**: SCL (I2C - MPU6050)

## Próximos Passos

### 1. Teste de PWM
- Verificar sinais PWM no osciloscópio
- Confirmar frequências e duty cycles
- Testar armamento dos ESCs

### 2. Controle Individual dos Motores
- Implementar controle independente para cada motor
- Integrar dados do MPU6050 para estabilização
- Adicionar controle de pitch, roll e yaw

### 3. Calibração
- Ajustar faixas de PWM para ESCs específicos
- Implementar sequence de armamento
- Adicionar failsafe

## Limitações Conhecidas

1. **Frequência do Timer2**: ~61Hz vs 50Hz ideal
   - Limitação do Timer 8-bit com prescalers disponíveis
   - Maioria dos ESCs aceita 50-400Hz

2. **Resolução do Timer2**: 256 steps vs 1250 do Timer1
   - Menor precisão para motores 3 e 4
   - Ainda adequada para controle de drone

3. **Falta de Hardware PWM**: Apenas 4 canais disponíveis
   - Não há mais pinos de hardware PWM no ATmega328P
   - Servos adicionais requerem software PWM
