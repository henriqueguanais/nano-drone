# Sistema PID - Controlador de Voo Nano-Drone

## 📋 Resumo da Implementação

O sistema PID (Proporcional-Integral-Derivativo) foi implementado para controle de estabilização do nano-drone, permitindo voo autônomo com correção automática de pitch, roll e yaw.

## 🏗️ Arquitetura do Sistema

### Tasks FreeRTOS
1. **vtask_rc** (Prioridade 1) - Processa sinais RC e converte para comandos de voo
2. **vtask_mpu6050** (Prioridade 2) - Lê dados do sensor IMU e calcula ângulos
3. **vtask_flight_control** (Prioridade 3) - Executa algoritmos PID e controla motores

### Comunicação Entre Tasks
- **xQueueRC**: Dados brutos do receptor RC
- **xQueueIMU**: Dados processados do MPU6050 (ângulos)
- **xQueueFlightCtrl**: Comandos de voo processados

## 🎮 Mapeamento de Canais RC

### Canais de Entrada
```c
Canal 1 (rc_local_values[0]): Roll     -> roll_setpoint (-30° a +30°)
Canal 2 (rc_local_values[1]): Pitch    -> pitch_setpoint (-30° a +30°)
Canal 3 (rc_local_values[2]): Throttle -> throttle (1000-2000μs)
Canal 4 (rc_local_values[3]): Yaw      -> yaw_rate_setpoint (-100°/s a +100°/s)
Canal 5 (rc_local_values[4]): Arm/Disarm -> armed (>1500 = armado)
Canal 6 (rc_local_values[5]): Auxiliar (reservado)
```

### Lógica de Armamento
- **Armar**: Canal 5 > 1500 AND throttle < 1100μs
- **Desarmar**: Canal 5 < 1500 OR throttle em qualquer posição

## 🔧 Configuração dos Controladores PID

### PID Pitch (Eixo Y)
```c
Kp = 1.0    // Ganho proporcional
Ki = 0.0    // Ganho integral (desabilitado inicialmente)
Kd = 0.1    // Ganho derivativo
Limites: ±400 (unidades de correção)
```

### PID Roll (Eixo X)
```c
Kp = 1.0    // Ganho proporcional
Ki = 0.0    // Ganho integral (desabilitado inicialmente)
Kd = 0.1    // Ganho derivativo
Limites: ±400 (unidades de correção)
```

### PID Yaw (Taxa de Rotação Z)
```c
Kp = 0.5    // Ganho proporcional (menor para yaw)
Ki = 0.0    // Ganho integral (desabilitado inicialmente)
Kd = 0.05   // Ganho derivativo (menor para yaw)
Limites: ±200 (unidades de correção)
```

## 🚁 Layout dos Motores

### Configuração em X (Vista Superior)
```
    M1      M2
      \    /
       \  /
        ><
       /  \
      /    \
    M4      M3
```

### Matriz de Controle
```c
Motor 1 = throttle - pitch - roll - yaw
Motor 2 = throttle - pitch + roll + yaw
Motor 3 = throttle + pitch + roll - yaw
Motor 4 = throttle + pitch - roll + yaw
```

## 🔄 Fluxo de Operação

### 1. Inicialização
```c
init_pid_controllers();  // Configura ganhos e limites
xQueueCreate();         // Cria filas de comunicação
xTaskCreate();          // Cria tasks com prioridades
```

### 2. Processamento RC (50Hz)
```c
1. Recebe dados PPM do receptor
2. Converte para comandos de voo
3. Aplica limitação de segurança
4. Envia para xQueueFlightCtrl
```

### 3. Processamento IMU (50Hz)
```c
1. Lê dados brutos do MPU6050
2. Calcula ângulos usando lookup table
3. Converte para float (graus)
4. Envia para xQueueIMU
```

### 4. Controle de Voo (50Hz)
```c
1. Recebe dados IMU e comandos RC
2. Executa algoritmos PID
3. Calcula correções individuais
4. Aplica ao controle de motores
```

## 📊 Algoritmo PID Implementado

### Função Principal
```c
float pid_calculate(pid_controller_t *pid, float setpoint, float measured_value, uint32_t current_time)
```

### Componentes
```c
// Termo Proporcional
P = Kp × erro

// Termo Integral (com anti-windup)
I = Ki × ∫(erro × dt)

// Termo Derivativo
D = Kd × (d_erro / dt)

// Saída Total
output = P + I + D
```

### Proteções Implementadas
- **Anti-windup**: Previne saturação do termo integral
- **Limitação de saída**: Evita comandos excessivos
- **Filtro temporal**: Evita divisão por zero na primeira execução

## 🛡️ Sistemas de Segurança

### Proteção de Armamento
- Só permite armamento com throttle mínimo
- Desarmamento automático em emergência
- Motores em posição mínima quando desarmado

### Limitação de Comandos
- Setpoints limitados: ±30° (pitch/roll), ±100°/s (yaw)
- Correções PID limitadas para evitar oscilações
- Valores PWM sempre dentro da faixa 1000-2000μs

### Failsafe
- Modo desarmado = todos os motores em 1000μs
- Verificação de validade dos dados RC
- Timeout protection (implementação futura)

## 📈 Monitoramento e Debug

### Saída USART - Task RC
```
RC: T=1500 P=0 R=0 Y=0 ARM=1
```

### Saída USART - Task IMU
```
Roll: 2° | Pitch: -1° | YawRate: 5°/s
```

### Saída USART - Task Flight Control
```
PID: P=15 R=-8 Y=2 T=1500
```

## 🔧 Calibração e Tuning

### Processo de Calibração
1. **Teste de Armamento**: Verificar sequência de arm/disarm
2. **Teste de Throttle**: Confirmar resposta linear
3. **Teste de Setpoints**: Validar limites de pitch/roll/yaw
4. **Tuning PID**: Ajustar ganhos conforme resposta

### Valores Sugeridos para Tuning
```c
// Começar conservador
Kp = 0.5, Ki = 0.0, Kd = 0.05

// Aumentar gradualmente
Kp = 1.0, Ki = 0.0, Kd = 0.1

// Adicionar integral se necessário
Kp = 1.0, Ki = 0.01, Kd = 0.1
```

## 💾 Uso de Memória

### Após Implementação PID
- **Flash**: 22.172KB / 32KB (69.3%)
- **RAM**: 573B / 2KB (28.0%)
- **Margem**: Ainda adequada para futuras melhorias

### Comparação
- **Antes**: 18.9KB Flash, 462B RAM
- **Depois**: 22.1KB Flash, 573B RAM
- **Incremento**: +3.2KB Flash, +111B RAM

## 🚀 Próximos Passos

### Melhorias Imediatas
1. Calibração do MPU6050 (offset zero)
2. Filtro complementar para ângulos
3. Implementação de timeout/failsafe
4. Interface de configuração PID

### Funcionalidades Avançadas
1. Modos de voo (Stabilize, Acro, AltHold)
2. Integração com GPS
3. Controle de altitude
4. Sistema de logs

## 🔍 Troubleshooting

### Problemas Comuns
1. **Oscilação**: Reduzir Kp ou aumentar Kd
2. **Resposta lenta**: Aumentar Kp
3. **Overshoot**: Reduzir Kp ou adicionar Kd
4. **Drift**: Adicionar Ki (cuidado com windup)

### Verificações
- Conectividade I2C do MPU6050
- Sinais PWM no osciloscópio
- Valores RC via USART
- Resposta dos ESCs
