#!/bin/bash

# Script para calibração e teste do nano-drone
# Uso: ./calibrate.sh [test_type]

TEST_TYPE=${1:-help}

echo "🚁 Nano-Drone Calibration & Test"
echo "================================"

case $TEST_TYPE in
    imu)
        echo "📊 Teste IMU - Posicionamento e Leitura"
        echo "======================================="
        echo ""
        echo "Instruções:"
        echo "1. Posicione o drone em superfície plana"
        echo "2. Aguarde estabilização (2-3 segundos)"
        echo "3. Observe os valores de Roll/Pitch (devem estar próximos de 0°)"
        echo "4. Mova suavemente para testar resposta"
        echo ""
        echo "Valores esperados:"
        echo "  Roll: -5° a +5°"
        echo "  Pitch: -5° a +5°"
        echo "  YawRate: variável conforme movimento"
        echo ""
        echo "Pressione ENTER para iniciar monitoramento..."
        read
        ./monitor.sh
        ;;
    
    rc)
        echo "📡 Teste RC - Receptor PPM"
        echo "=========================="
        echo ""
        echo "Instruções:"
        echo "1. Ligue o transmissor RC"
        echo "2. Verifique se receptor está conectado em PD2"
        echo "3. Mova sticks e observe valores mudarem"
        echo "4. Teste switch de armamento"
        echo ""
        echo "Valores esperados:"
        echo "  CH1 (Roll): 1000-2000 µs"
        echo "  CH2 (Pitch): 1000-2000 µs"
        echo "  CH3 (Throttle): 1000-2000 µs"
        echo "  CH4 (Yaw): 1000-2000 µs"
        echo "  CH5 (Arm): 1000 (desarm) / 2000 (arm)"
        echo ""
        echo "Pressione ENTER para iniciar monitoramento..."
        read
        ./monitor.sh
        ;;
    
    pwm)
        echo "⚡ Teste PWM - Sinais dos Motores"
        echo "================================"
        echo ""
        echo "⚠️  ATENÇÃO: Remova hélices antes do teste!"
        echo ""
        echo "Instruções:"
        echo "1. Conecte osciloscópio nos pinos PWM:"
        echo "   - Motor 1: PB1 (Pin 9)"
        echo "   - Motor 2: PB2 (Pin 10)"
        echo "   - Motor 3: PB3 (Pin 11)"
        echo "   - Motor 4: PD3 (Pin 3)"
        echo "2. Arme o drone (CH5 = 2000, CH3 = baixo)"
        echo "3. Aumente throttle gradualmente"
        echo "4. Observe frequências:"
        echo "   - Motores 1&2: 50Hz"
        echo "   - Motores 3&4: ~61Hz"
        echo ""
        echo "Valores esperados:"
        echo "  Mínimo: 1000µs (1ms)"
        echo "  Máximo: 2000µs (2ms)"
        echo "  Período: 20ms (50Hz) ou 16.4ms (61Hz)"
        echo ""
        echo "Pressione ENTER para continuar..."
        read
        ;;
    
    pid)
        echo "🎯 Teste PID - Sistema de Controle"
        echo "=================================="
        echo ""
        echo "⚠️  ATENÇÃO: Teste com drone fixo em suporte!"
        echo ""
        echo "Instruções:"
        echo "1. Fixe drone em suporte que permita movimento"
        echo "2. Arme o sistema (CH5 = 2000, CH3 = baixo)"
        echo "3. Aumente throttle para ~40%"
        echo "4. Mova drone manualmente e observe correção"
        echo "5. Teste comandos de stick suaves"
        echo ""
        echo "Comportamento esperado:"
        echo "  - Drone tenta manter posição quando perturbado"
        echo "  - Resposta suave aos comandos de stick"
        echo "  - Motores não oscilam excessivamente"
        echo ""
        echo "Ajuste de ganhos (flight_config.h):"
        echo "  - Se oscila: diminua Kp"
        echo "  - Se lento: aumente Kp"
        echo "  - Se deriva: ajuste Ki"
        echo "  - Se instável: ajuste Kd"
        echo ""
        echo "Pressione ENTER para continuar..."
        read
        ;;
    
    config)
        echo "⚙️  Configuração Rápida"
        echo "======================"
        echo ""
        echo "Configurações disponíveis:"
        echo "1. conservative - Ganhos baixos, resposta suave"
        echo "2. aggressive - Ganhos altos, resposta rápida"
        echo "3. smooth - Ganhos médios, voo estável"
        echo "4. debug - Habilitado debug serial"
        echo ""
        echo "Qual configuração usar?"
        read -p "Escolha (1-4): " choice
        
        case $choice in
            1) ./quick_config.sh conservative ;;
            2) ./quick_config.sh aggressive ;;
            3) ./quick_config.sh smooth ;;
            4) ./quick_config.sh debug ;;
            *) echo "Opção inválida!" ;;
        esac
        ;;
    
    help|*)
        echo "Tipos de teste disponíveis:"
        echo ""
        echo "  imu      - Teste do sensor MPU6050"
        echo "  rc       - Teste do receptor RC PPM"
        echo "  pwm      - Teste dos sinais PWM dos motores"
        echo "  pid      - Teste do sistema de controle PID"
        echo "  config   - Configuração rápida de parâmetros"
        echo ""
        echo "Exemplos:"
        echo "  ./calibrate.sh imu"
        echo "  ./calibrate.sh rc"
        echo "  ./calibrate.sh pwm"
        echo ""
        ;;
esac
