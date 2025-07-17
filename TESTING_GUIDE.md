# Guia de Teste - Sistema PID Nano-Drone

## 🔧 Preparação para Testes

### Hardware Necessário
- [ ] ATmega328P programado com firmware
- [ ] MPU6050 conectado via I2C (SDA=PC4, SCL=PC5)
- [ ] Receptor RC PPM conectado em PD2
- [ ] 4 ESCs conectados nos pinos PWM (PB1, PB2, PB3, PD3)
- [ ] Cabo USART para monitoramento (PD0=RX, PD1=TX)
- [ ] Osciloscópio (recomendado para verificar PWM)

### Software Necessário
- [ ] Terminal serial (115200 baud)
- [ ] Transmissor RC configurado
- [ ] Firmware gravado no ATmega328P

## 🧪 Sequência de Testes

### Teste 1: Inicialização do Sistema
```bash
# Conectar cabo serial e abrir terminal
# Baudrate: 115200
# Verificar saídas esperadas:
```

**Saídas Esperadas:**
```
# Se MPU6050 OK:
Roll: 0° | Pitch: 0° | YawRate: 0°/s

# Se MPU6050 com erro:
ERRO: MPU6050 nao encontrado!
Verifique as conexoes I2C (SDA/SCL)
LED piscando rapidamente...
```

**Critérios de Aprovação:**
- [ ] MPU6050 detectado e funcionando
- [ ] LED piscando lentamente (0.5s)
- [ ] Ângulos sendo lidos corretamente

### Teste 2: Recepção de Sinais RC
```bash
# Ligar transmissor RC
# Mover sticks e verificar leitura
```

**Saídas Esperadas:**
```
RC: T=1000 P=0 R=0 Y=0 ARM=0
RC: T=1500 P=15 R=-10 Y=5 ARM=0
RC: T=2000 P=0 R=0 Y=0 ARM=1
```

**Critérios de Aprovação:**
- [ ] Todos os 6 canais sendo lidos
- [ ] Valores entre 1000-2000μs
- [ ] Resposta proporcional aos sticks
- [ ] Armamento funcionando (canal 5)

### Teste 3: Geração de PWM
```bash
# Com osciloscópio, verificar pinos PWM
# PB1, PB2, PB3, PD3
```

**Valores Esperados:**
```
Frequência:
- PB1, PB2: 50Hz ± 1Hz
- PB3, PD3: 61Hz ± 2Hz

Duty Cycle (armado):
- Mínimo: ~5% (1ms)
- Máximo: ~10% (2ms)
```

**Critérios de Aprovação:**
- [ ] Frequências corretas
- [ ] Duty cycles na faixa 1-2ms
- [ ] Sinais estáveis sem jitter
- [ ] Resposta ao throttle

### Teste 4: Armamento e Desarmamento
```bash
# Testar sequência de armamento
# Canal 5 > 1500 + Throttle < 1100
```

**Procedimento:**
1. Throttle em mínimo (1000μs)
2. Canal 5 em posição alta (>1500μs)
3. Verificar: `RC: ... ARM=1`
4. Canal 5 em posição baixa (<1500μs)
5. Verificar: `RC: ... ARM=0`

**Critérios de Aprovação:**
- [ ] Armamento só com throttle baixo
- [ ] Desarmamento imediato
- [ ] Motores em 1ms quando desarmado

### Teste 5: Resposta do Sistema PID
```bash
# Com drone armado, inclinar manualmente
# Verificar correções PID
```

**Saídas Esperadas:**
```
PID: P=0 R=0 Y=0 T=1000     # Nível
PID: P=25 R=0 Y=0 T=1200    # Inclinado frente
PID: P=0 R=-15 Y=0 T=1200   # Inclinado direita
```

**Critérios de Aprovação:**
- [ ] Correções proporcionais à inclinação
- [ ] Sinais opostos à inclinação
- [ ] Não há oscilações excessivas
- [ ] Resposta rápida e suave

### Teste 6: Teste de Voo (Bancada)
```bash
# Com drone fixo na bancada
# Testar resposta a comandos
```

**Procedimento:**
1. Armar drone
2. Aumentar throttle gradualmente
3. Testar comandos pitch/roll/yaw
4. Verificar estabilização

**Critérios de Aprovação:**
- [ ] Motores respondem ao throttle
- [ ] Correções automáticas funcionam
- [ ] Comandos RC são processados
- [ ] Sistema estável sem oscilações

## 🔧 Calibração Básica

### Ajuste de Ganhos PID
Se o sistema apresentar problemas:

```c
// Em flight_config.h, ajustar valores:

// Oscilação excessiva:
#define PID_PITCH_KP    0.5    // Reduzir Kp
#define PID_ROLL_KP     0.5

// Resposta lenta:
#define PID_PITCH_KP    1.5    // Aumentar Kp
#define PID_ROLL_KP     1.5

// Overshoot:
#define PID_PITCH_KD    0.2    // Aumentar Kd
#define PID_ROLL_KD     0.2
```

### Calibração do MPU6050
```c
// Para calibrar offset zero:
// 1. Drone em superfície nivelada
// 2. Anotar valores de Roll/Pitch
// 3. Adicionar offset no código
```

## 🚨 Troubleshooting

### Problema: MPU6050 não detectado
**Soluções:**
- Verificar conexões I2C
- Testar com multímetro: 3.3V, SDA, SCL
- Verificar soldas

### Problema: RC não responde
**Soluções:**
- Verificar conexão PPM em PD2
- Confirmar configuração do receptor
- Testar com osciloscópio

### Problema: Motores não giram
**Soluções:**
- Verificar sequência de armamento
- Conferir conexões ESC
- Testar calibração ESC

### Problema: Oscilações no voo
**Soluções:**
- Reduzir ganhos PID
- Verificar balanceamento
- Checar vibração da estrutura

## 📈 Otimizações Futuras

### Prioridade Alta
1. **Filtro Complementar**: Combinar accel + gyro
2. **Calibração Automática**: Offset zero do MPU6050
3. **Failsafe**: Timeout de RC
4. **Modos de Voo**: Stabilize, Acro, etc.

### Prioridade Média
1. **Interface de Configuração**: Ajuste PID via serial
2. **Logging**: Gravação de telemetria
3. **GPS**: Controle de posição
4. **Altitude**: Sensor barométrico

### Prioridade Baixa
1. **Interface Gráfica**: Configuração por PC
2. **Controle Remoto**: App mobile
3. **Autonomia**: Waypoints automáticos
4. **FPV**: Transmissão de vídeo

## 🎯 Métricas de Sucesso

### Teste Básico
- [ ] Sistema inicializa sem erros
- [ ] RC responde corretamente
- [ ] PWM gerado adequadamente
- [ ] Armamento funciona

### Teste Intermediário
- [ ] PID responde a inclinações
- [ ] Correções são aplicadas
- [ ] Sem oscilações excessivas
- [ ] Estabilização básica funciona

### Teste Avançado
- [ ] Voo estável em bancada
- [ ] Resposta suave a comandos
- [ ] Recuperação automática
- [ ] Pronto para voo real

## 📋 Checklist Final

**Antes do Primeiro Voo:**
- [ ] Todos os testes aprovados
- [ ] Ganhos PID ajustados
- [ ] Failsafe configurado
- [ ] Estrutura balanceada
- [ ] Área de teste segura
- [ ] Piloto experiente presente

**Documentação:**
- [ ] Configurações salvas
- [ ] Problemas documentados
- [ ] Melhorias identificadas
- [ ] Backup do firmware

---

**⚠️ IMPORTANTE**: Sempre teste em ambiente seguro com proteções adequadas. O sistema PID é funcional mas requer calibração para cada configuração específica de drone.
