# 🚁 Guia de Uso Final - Nano-Drone com Filtro Complementar

## 📊 **Status do Sistema**

### **✅ Funcionalidades Completas**
- **Sistema PID** com controladores para pitch, roll e yaw
- **Filtro Complementar** para estimativa precisa de ângulos
- **PWM Hardware** para 4 motores com frequências otimizadas
- **Decodificação RC PPM** com sistema de armamento
- **FreeRTOS** com execução determinística a 50Hz
- **Calibração Automática** do IMU na inicialização

### **💾 Uso de Memória**
- **Flash**: 24.512KB (74.8% do ATmega328P)
- **RAM**: 667B (32.6% do ATmega328P)
- **Status**: Otimizado e dentro dos limites seguros

## 🛠️ **Scripts de Desenvolvimento**

### **Script Principal de Desenvolvimento**
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

### **Scripts de Calibração e Teste**
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
# Configuração conservadora (ganhos baixos)
./quick_config.sh conservative

# Configuração agressiva (ganhos altos)
./quick_config.sh aggressive

# Configuração suave (ganhos médios)
./quick_config.sh smooth

# Configuração de debug
./quick_config.sh debug
```

## 🔧 **Procedimento de Teste Completo**

### **1. Preparação do Hardware**
```bash
# Conectar componentes:
# - MPU6050: SDA=PC4, SCL=PC5, VCC=3.3V, GND=GND
# - RC PPM: Signal=PD2, VCC=5V, GND=GND
# - Motores: PB1, PB2, PB3, PD3
# - Serial: RX=PD0, TX=PD1
```

### **2. Compilação e Upload**
```bash
# Compilar e fazer upload
./dev.sh all /dev/ttyUSB0

# Verificar se upload foi bem-sucedido
# Saída esperada: "Upload concluído com sucesso!"
```

### **3. Teste de Inicialização**
```bash
# Monitorar inicialização
./monitor.sh /dev/ttyUSB0

# Saída esperada:
# IMU inicializado. Calibrando...
# Roll: 0° | Pitch: 0° | YawRate: 0°/s [CAL]
# Calibracao concluida! Pronto para voo.
```

### **4. Teste de Sensores**
```bash
# Teste IMU (drone em superfície plana)
./calibrate.sh imu

# Valores esperados:
# Roll: -5° a +5°
# Pitch: -5° a +5°
# YawRate: próximo de 0°/s quando parado
```

### **5. Teste RC**
```bash
# Teste receptor RC
./calibrate.sh rc

# Mover sticks e verificar:
# CH1-CH4: 1000-2000 µs
# CH5: 1000 (desarm) / 2000 (arm)
```

### **6. Teste de Motores**
```bash
# ⚠️ REMOVER HÉLICES ANTES DO TESTE!
./calibrate.sh pwm

# Verificar com osciloscópio:
# - Frequência: 50Hz (motores 1&2), ~61Hz (motores 3&4)
# - Pulsos: 1000-2000µs
```

## 🎯 **Procedimento de Voo**

### **Pré-Voo**
1. **Calibração**: Aguardar calibração automática
2. **Verificação**: Monitorar ângulos via serial
3. **Teste RC**: Verificar todos os canais
4. **Armamento**: CH5 = 2000, CH3 = baixo

### **Primeiro Voo**
1. **Throttle Gradual**: Aumentar lentamente
2. **Observar Resposta**: Verificar estabilização
3. **Comandos Suaves**: Movimentos pequenos primeiro
4. **Monitoramento**: Observar dados via serial

### **Ajustes de Ganhos**
```bash
# Se drone oscila muito:
./quick_config.sh conservative

# Se resposta muito lenta:
./quick_config.sh aggressive

# Para voo suave:
./quick_config.sh smooth
```

## 📡 **Monitoramento em Tempo Real**

### **Saída Serial Normal**
```
Roll: 2° | Pitch: -1° | YawRate: 5°/s
RC: 1500 1500 1200 1500 2000 1500 | Armed
Flight: P:0.0 R:0.0 Y:0.0 T:200
Motors: 1200 1200 1200 1200
```

### **Interpretação dos Dados**
- **Roll/Pitch**: Ângulos do drone (±30° máximo)
- **YawRate**: Velocidade de rotação (±100°/s máximo)
- **RC**: Valores dos canais (1000-2000µs)
- **Armed**: Status de armamento
- **Flight**: Comandos de voo (P=Pitch, R=Roll, Y=Yaw, T=Throttle)
- **Motors**: Valores PWM dos motores

## 🔧 **Solução de Problemas**

### **IMU não Encontrado**
```
ERRO: MPU6050 nao encontrado!
```
- Verificar conexões I2C (SDA/SCL)
- Verificar alimentação 3.3V
- Verificar endereço I2C (0x68)

### **RC não Responde**
```
RC: 0 0 0 0 0 0
```
- Verificar conexão PPM em PD2
- Verificar transmissor ligado
- Verificar modo PPM no receptor

### **Motores não Giram**
```
Motors: 1000 1000 1000 1000
```
- Verificar armamento (CH5 = 2000)
- Verificar throttle > 1000
- Verificar conexões PWM
- Verificar ESCs calibrados

### **Oscilação Excessiva**
- Diminuir ganhos Kp
- Usar configuração conservative
- Verificar balanceamento das hélices
- Verificar rigidez do frame

## 📋 **Checklist de Voo**

### **Hardware**
- [ ] MPU6050 conectado e funcionando
- [ ] RC PPM respondendo
- [ ] 4 motores conectados
- [ ] Bateria carregada
- [ ] Hélices balanceadas
- [ ] Frame rígido

### **Software**
- [ ] Firmware compilado sem erros
- [ ] Upload bem-sucedido
- [ ] Calibração IMU concluída
- [ ] RC channels respondendo
- [ ] Sistema armando/desarmando
- [ ] Motores girando quando armado

### **Segurança**
- [ ] Área de voo desobstruída
- [ ] Primeiro voo com proteção
- [ ] Transmissor com failsafe
- [ ] Monitoramento ativo
- [ ] Pessoa experiente supervisionando

## 🚀 **Próximos Desenvolvimentos**

### **Possíveis Melhorias**
- Filtro de Kalman para fusão de sensores
- Controle de altitude com barômetro
- Telemetria wireless
- Modo GPS hold
- Gravação de dados de voo

### **Otimizações**
- Ajuste fino dos ganhos PID
- Calibração automática em voo
- Detecção de falhas
- Failsafe inteligente

## 📞 **Suporte**

### **Documentação**
- `PID_SYSTEM_DOCUMENTATION.md` - Sistema PID detalhado
- `COMPLEMENTARY_FILTER_UPGRADE.md` - Filtro complementar
- `TESTING_GUIDE.md` - Guia de testes
- `PWM_CONFIG_SUMMARY.md` - Configuração PWM

### **Configuração**
- `flight_config.h` - Todos os parâmetros
- `quick_config.sh` - Configurações rápidas

---

**🎉 Parabéns! Seu nano-drone está pronto para voo! 🚁**

*Lembre-se: Sempre priorize a segurança e voe com responsabilidade.*
