# 🚁 Nano-Drone Flight Controller

Controlador de voo completo para nano-drone baseado em ATmega328P com FreeRTOS, sistema PID, filtro complementar, sensor IMU MPU6050, receptor RC PPM e controle PWM de 4 motores.

## ✅ **Status: Sistema Completo e Funcional**

### 🎯 **Funcionalidades Implementadas**
- **Sistema PID Completo** com controladores para pitch, roll e yaw
- **Filtro Complementar** para estimativa precisa de ângulos
- **PWM Hardware** para 4 motores com frequências otimizadas
- **Decodificação RC PPM** com sistema de armamento seguro
- **FreeRTOS** com execução determinística a 50Hz
- **Calibração Automática** do IMU na inicialização

## 🚀 **Características**

- **Microcontrolador**: ATmega328P @ 16MHz
- **RTOS**: FreeRTOS para multitarefa em tempo real
- **Sensor IMU**: MPU6050 com filtro complementar
- **Controle RC**: Receptor PPM (6 canais) com armamento
- **Motores**: 4 motores brushless com ESCs
- **PWM**: 4 canais hardware PWM (50Hz e ~61Hz)
- **Comunicação**: USART para debug e monitoramento
- **Segurança**: Sistema de armamento e limitação de comandos

## 📋 **Especificações Técnicas**

### **Hardware PWM**
- **Timer1 (16-bit)**: 50Hz exato - Motores 1 e 2 (PB1, PB2)
- **Timer2 (8-bit)**: ~61Hz - Motores 3 e 4 (PB3, PD3)
- **Resolução**: 1250 steps (Timer1) / 256 steps (Timer2)
- **Faixa**: 1000-2000μs (1ms-2ms) para compatibilidade ESC

### **Sistema PID**
- **Controladores**: 3 PID independentes (pitch, roll, yaw)
- **Frequência**: 50Hz de execução
- **Anti-windup**: Proteção contra saturação integral
- **Configurável**: Ganhos ajustáveis via arquivo de configuração

### **Filtro Complementar**
- **Fusão de Sensores**: 95% giroscópio + 5% acelerômetro
- **Calibração**: Automática na inicialização
- **Compensação**: Elimina deriva do giroscópio
- **Precisão**: Ângulos estáveis e responsivos

### **Mapeamento de Pinos**
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

### **Uso de Memória**
- **Flash**: 24.512KB / 32KB (74.8%)
- **RAM**: 667B / 2KB (32.6%)
- **Status**: Otimizado e dentro dos limites seguros

## 🛠️ **Compilação e Upload**

### **Scripts de Desenvolvimento**
```bash
# Compilar firmware
./dev.sh build

# Upload para Arduino
./dev.sh upload /dev/ttyUSB0

# Monitorar via serial
./dev.sh monitor /dev/ttyUSB0

# Build + Upload em uma operação
./dev.sh all /dev/ttyUSB0

# Informações do projeto
./dev.sh info
```

### **Compilação Manual**
```bash
# Compilar
make

# Limpar e recompilar
make clean && make

# Programar no microcontrolador (se avrdude configurado)
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

## 🎮 **Funcionamento**

### **1. Inicialização**
- Configuração dos timers PWM para 4 motores
- Inicialização do I2C para MPU6050 com calibração automática
- Configuração das interrupções PPM para RC
- Criação das tasks FreeRTOS com prioridades otimizadas

### **2. Task RC (Receptor)**
- Decodifica sinais PPM do receptor (6 canais)
- Converte comandos RC em setpoints de voo
- Processa armamento/desarmamento seguro
- Envia comandos para controle de voo

### **3. Task MPU6050 (Sensor)**
- Lê dados do acelerômetro e giroscópio
- Aplica filtro complementar para ângulos precisos
- Realiza calibração automática na inicialização
- Envia dados filtrados para controle PID

### **4. Task Flight Control (PID)**
- Executa controladores PID para pitch, roll e yaw
- Calcula correções individuais para cada motor
- Aplica configuração X-quadcopter para controle
- Atualiza sinais PWM dos motores em tempo real

### **5. Controle PWM**
- Mapeamento de 1000-2000μs para valores OCR
- Geração de sinais PWM compatíveis com ESCs
- Controle individual de 4 motores com anti-windup

## 🧪 **Testes e Calibração**

### **Scripts de Teste**
```bash
# Teste do sensor IMU
./calibrate.sh imu

# Teste do receptor RC
./calibrate.sh rc

# Teste dos sinais PWM
./calibrate.sh pwm

# Teste do sistema PID
./calibrate.sh pid

# Configuração rápida
./calibrate.sh config
```

### **Configurações Rápidas**
```bash
# Configuração conservadora (primeiro voo)
./quick_config.sh conservative

# Configuração agressiva (voo esportivo)
./quick_config.sh aggressive

# Configuração suave (voo estável)
./quick_config.sh smooth

# Configuração de debug
./quick_config.sh debug
```

## 📊 **Monitoramento**

### **Saída Serial em Tempo Real**
```
Roll: 2° | Pitch: -1° | YawRate: 5°/s
RC: 1500 1500 1200 1500 2000 1500 | Armed
Flight: P:0.0 R:0.0 Y:0.0 T:200
Motors: 1200 1200 1200 1200
```

### **Indicadores de Status**
- **LED**: Pisca durante operação normal
- **Serial**: Dados em tempo real a 115200 baud
- **Calibração**: Indicador [CAL] durante calibração
- **Armamento**: Status Armed/Disarmed

## 📚 **Documentação**

### **Guias Completos**
- `FINAL_USER_GUIDE.md` - Guia completo de uso
- `PID_SYSTEM_DOCUMENTATION.md` - Sistema PID detalhado
- `COMPLEMENTARY_FILTER_UPGRADE.md` - Filtro complementar
- `TESTING_GUIDE.md` - Procedimentos de teste
- `PWM_CONFIG_SUMMARY.md` - Configuração PWM

### **Documentação Técnica**
- `FREQUENCY_ANALYSIS.md` - Análise de frequências
- `PROJECT_SUMMARY.md` - Resumo técnico do projeto

## ⚙️ **Configuração**

### **Parâmetros Principais**
```c
// Em include/flight_config.h
#define PID_PITCH_KP    1.0     // Ganho proporcional pitch
#define PID_ROLL_KP     1.0     // Ganho proporcional roll
#define PID_YAW_KP      0.5     // Ganho proporcional yaw

#define COMPLEMENTARY_FILTER_ALPHA  0.95  // Peso do filtro
#define CALIBRATION_SAMPLES         100   // Amostras calibração
```

### **Estrutura de Arquivos**
```
nano-drone/
├── src/
│   ├── main.c          # Código principal com PID
│   ├── mpu6050.c       # Driver IMU + filtro complementar
│   └── USART.c         # Comunicação serial
├── include/
│   ├── flight_config.h # Configurações do sistema
│   ├── mpu6050.h       # Header do MPU6050
│   └── USART.h         # Header USART
├── lib/freertos/       # Biblioteca FreeRTOS
├── build/              # Arquivos compilados
├── *.sh                # Scripts de desenvolvimento
└── *.md                # Documentação completa
```

## 🎯 **Próximos Desenvolvimentos**

### **Melhorias Futuras**
- [ ] Filtro de Kalman para fusão de sensores
- [ ] Controle de altitude com barômetro
- [ ] Telemetria wireless
- [ ] Modo GPS hold
- [ ] Gravação de dados de voo

### **Otimizações**
- [ ] Ajuste fino automático de ganhos
- [ ] Calibração em voo
- [ ] Detecção de falhas
- [ ] Failsafe inteligente
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