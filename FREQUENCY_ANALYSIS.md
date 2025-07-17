## Análise de Impacto: Frequências PWM Diferentes

### Situação Atual
- Timer1: 50Hz (20ms período) - Motores 1 e 2 (PB1, PB2)
- Timer2: ~61Hz (16.4ms período) - Motores 3 e 4 (PB3, PD3)

### Impactos Técnicos

#### ✅ POSITIVOS:
1. **Compatibilidade ESC**: Ambas frequências dentro da faixa padrão (50-400Hz)
2. **Duty Cycle Correto**: 1ms-2ms mantido em ambos timers
3. **Hardware PWM**: Máxima precisão e eficiência
4. **Resposta Adequada**: Diferença imperceptível para ESCs

#### ⚠️ CONSIDERAÇÕES:
1. **Resolução Diferente**:
   - Timer1: 1250 steps (0.8μs/step)
   - Timer2: 256 steps (64μs/step) 
   - Motor 3 e 4 têm menor precisão

2. **Controle Assimétrico**:
   - Motores 1,2: 62 níveis de controle preciso
   - Motores 3,4: 16 níveis de controle preciso

3. **Sincronização**:
   - Updates não síncronos entre motores
   - Para voo autônomo, pode afetar microvibrações

### Testes Recomendados

#### Teste 1: Resposta ESC
```bash
# Verificar se todos ESCs respondem igual em:
# - Sequência de armamento
# - Throttle mínimo/máximo  
# - Transições suaves
```

#### Teste 2: Precisão
```bash
# Comparar precisão entre:
# - Motores 1,2 (Timer1) vs Motores 3,4 (Timer2)
# - Pequenos incrementos de throttle
# - Estabilidade em hover
```

#### Teste 3: Vibração
```bash
# Com MPU6050 ativado:
# - Verificar se há vibração diferencial
# - Testar estabilidade em diferentes throttles
# - Comparar com todos motores em 50Hz (se possível)
```

### Soluções Alternativas (se necessário)

#### Opção 1: Software PWM para 2 motores
```c
// Manter Timer1 50Hz para motores críticos (1,2)
// Implementar software PWM 50Hz para motores 3,4
// Custo: Maior uso de CPU, menos precisão
```

#### Opção 2: Aceitar a diferença
```c
// Manter configuração atual
// Compensar via software no algoritmo PID
// Mapear resolução 8-bit para ter steps equivalentes
```

#### Opção 3: Frequência Unificada Alternativa  
```c
// Usar 62.5Hz em ambos (divisor limpo do Timer2)
// Timer1: ICR1 = 1023 (62.5Hz)
// Timer2: prescaler 1024 (61Hz - próximo)
```

### Recomendação Final

**MANTER A CONFIGURAÇÃO ATUAL** porque:

1. **Funciona bem** na prática
2. **Hardware PWM** é superior ao software
3. **Diferença mínima** para aplicação de drone
4. **Complexidade** x **Benefício** não justifica mudança

Se problemas aparecerem nos testes, podemos implementar correções específicas.
