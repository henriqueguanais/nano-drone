# Nano-Drone Flight Controller

Controlador de voo para nano-drone baseado em ATmega328P com FreeRTOS, integração do sensor IMU MPU6050, receptor RC PPM e controle PWM de 4 motores.

## 🚀 Características

- **Microcontrolador**: ATmega328P @ 16MHz
- **RTOS**: FreeRTOS para multitarefa
- **Sensor IMU**: MPU6050 (acelerômetro + giroscópio)
- **Controle RC**: Receptor PPM (6 canais)
- **Motores**: 4 motores brushless com ESCs
- **PWM**: 4 canais hardware PWM (50Hz e ~61Hz)
- **Comunicação**: USART para debug

## 📋 Especificações Técnicas

### Hardware PWM
- **Timer1 (16-bit)**: 50Hz exato - Motores 1 e 2 (PB1, PB2)
- **Timer2 (8-bit)**: ~61Hz - Motores 3 e 4 (PB3, PD3)
- **Resolução**: 1250 steps (Timer1) / 256 steps (Timer2)
- **Faixa**: 1000-2000μs (1ms-2ms) para compatibilidade ESC

### Mapeamento de Pinos
```
PD2 - INT0      : Entrada PPM do receptor RC
PB1 - OC1A      : Motor 1 (Timer1)
PB2 - OC1B      : Motor 2 (Timer1)
PB3 - OC2A      : Motor 3 (Timer2)
PD3 - OC2B      : Motor 4 (Timer2)
PB5 - LED       : LED de status
PC4 - SDA       : I2C para MPU6050
PC5 - SCL       : I2C para MPU6050
PD0 - RX        : USART para debug
PD1 - TX        : USART para debug
```

### Uso de Memória
- **Flash**: 18.9KB / 32KB (57.8%)
- **RAM**: 462B / 2KB (22.6%)

## 🔧 Compilação

```bash
# Compilar
make

# Limpar e recompilar
make clean && make

# Programar no microcontrolador
make program
```

## 📊 Estrutura do Projeto

```
nano-drone/
├── src/
│   ├── main.c          # Código principal
│   ├── mpu6050.c       # Driver do sensor IMU
│   └── USART.c         # Comunicação serial
├── include/
│   ├── mpu6050.h       # Header do MPU6050
│   └── USART.h         # Header USART
├── lib/freertos/       # Biblioteca FreeRTOS
├── build/              # Arquivos compilados
└── tests/              # Testes unitários
```

## 🎮 Funcionamento

### 1. Inicialização
- Configuração dos timers PWM
- Inicialização do I2C para MPU6050
- Configuração das interrupções PPM
- Criação das tasks FreeRTOS

### 2. Task RC (Receptor)
- Decodifica sinais PPM do receptor
- Processa 6 canais RC
- Aplica throttle (canal 3) a todos os motores
- Envia dados via USART para debug

### 3. Task MPU6050 (Sensor)
- Lê dados do acelerômetro e giroscópio
- Calcula ângulos usando lookup table
- Pisca LED de status
- Prepara dados para controle de estabilização

### 4. Controle PWM
- Mapeamento de 1000-2000μs para valores OCR
- Geração de sinais PWM compatíveis com ESCs
- Controle individual de 4 motores

## 📈 Próximos Desenvolvimentos

### Controle de Estabilização
- [ ] Implementar controle PID para pitch, roll e yaw
- [ ] Integrar dados do MPU6050 com controle de motores
- [ ] Adicionar controle individual de cada motor

### Melhorias de Sistema
- [ ] Implementar sequência de armamento dos ESCs
- [ ] Adicionar sistema de failsafe
- [ ] Calibração automática do MPU6050
- [ ] Suporte a diferentes tipos de ESCs

### Interface e Debug
- [ ] Interface de configuração via USART
- [ ] Logging de telemetria
- [ ] Ferramenta de calibração RC

## 🔬 Testes

### Verificação de PWM
```bash
# Usar osciloscópio para verificar:
# - Frequência: 50Hz (Timer1), ~61Hz (Timer2)
# - Duty cycle: 5%-10% (1ms-2ms)
# - Estabilidade do sinal
```

### Teste de RC
```bash
# Monitorar via USART:
# - Valores dos canais RC (1000-2000μs)
# - Mapeamento para PWM
# - Resposta dos motores
```

## 📝 Configuração ESCs

### Sequência de Armamento
1. Throttle em posição mínima (1000μs)
2. Aguardar beeps de confirmação dos ESCs
3. Aplicar throttle gradualmente

### Calibração
- Verificar faixa de PWM: 1000-2000μs
- Ajustar valores min/max se necessário
- Testar resposta em diferentes throttles

## 🛠️ Dependências

- **avr-gcc**: Compilador AVR
- **avr-libc**: Biblioteca C para AVR
- **FreeRTOS**: Sistema operacional em tempo real
- **Make**: Sistema de build

## 📄 Licença

Este projeto está licenciado sob a [MIT License](LICENSE).

## 🤝 Contribuição

Contribuições são bem-vindas! Por favor, abra issues ou pull requests para melhorias.

## 📚 Referências

- [ATmega328P Datasheet](https://www.microchip.com/wwwproducts/en/ATmega328P)
- [FreeRTOS Documentation](https://www.freertos.org/)
- [MPU6050 Datasheet](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)