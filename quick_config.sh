#!/bin/bash

# Script de Configuração Rápida - Nano-Drone PID
# Uso: ./quick_config.sh [preset]

FLIGHT_CONFIG_FILE="include/flight_config.h"

function show_help() {
    echo "=== Configuração Rápida do Nano-Drone PID ==="
    echo "Uso: $0 [preset]"
    echo ""
    echo "Presets disponíveis:"
    echo "  conservative  - Ganhos baixos, estável (padrão)"
    echo "  aggressive    - Ganhos altos, resposta rápida"
    echo "  smooth        - Ganhos médios, voo suave"
    echo "  debug         - Valores para debugging"
    echo "  restore       - Restaurar configuração original"
    echo ""
    echo "Exemplo: $0 aggressive"
    echo ""
    echo "Após alterar configuração, execute:"
    echo "  make clean && make"
}

function backup_config() {
    if [ ! -f "${FLIGHT_CONFIG_FILE}.backup" ]; then
        cp "$FLIGHT_CONFIG_FILE" "${FLIGHT_CONFIG_FILE}.backup"
        echo "✅ Backup criado: ${FLIGHT_CONFIG_FILE}.backup"
    fi
}

function apply_conservative() {
    echo "🔧 Aplicando configuração CONSERVATIVE (estável)..."
    sed -i 's/#define PID_PITCH_KP.*/#define PID_PITCH_KP    0.8/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_PITCH_KD.*/#define PID_PITCH_KD    0.08/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KP.*/#define PID_ROLL_KP     0.8/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KD.*/#define PID_ROLL_KD     0.08/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KP.*/#define PID_YAW_KP      0.3/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KD.*/#define PID_YAW_KD      0.03/' "$FLIGHT_CONFIG_FILE"
    echo "✅ Configuração CONSERVATIVE aplicada"
}

function apply_aggressive() {
    echo "🔧 Aplicando configuração AGGRESSIVE (resposta rápida)..."
    sed -i 's/#define PID_PITCH_KP.*/#define PID_PITCH_KP    1.5/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_PITCH_KD.*/#define PID_PITCH_KD    0.15/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KP.*/#define PID_ROLL_KP     1.5/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KD.*/#define PID_ROLL_KD     0.15/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KP.*/#define PID_YAW_KP      0.8/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KD.*/#define PID_YAW_KD      0.08/' "$FLIGHT_CONFIG_FILE"
    echo "✅ Configuração AGGRESSIVE aplicada"
}

function apply_smooth() {
    echo "🔧 Aplicando configuração SMOOTH (voo suave)..."
    sed -i 's/#define PID_PITCH_KP.*/#define PID_PITCH_KP    1.0/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_PITCH_KD.*/#define PID_PITCH_KD    0.12/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KP.*/#define PID_ROLL_KP     1.0/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KD.*/#define PID_ROLL_KD     0.12/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KP.*/#define PID_YAW_KP      0.5/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KD.*/#define PID_YAW_KD      0.05/' "$FLIGHT_CONFIG_FILE"
    echo "✅ Configuração SMOOTH aplicada"
}

function apply_debug() {
    echo "🔧 Aplicando configuração DEBUG (valores baixos)..."
    sed -i 's/#define PID_PITCH_KP.*/#define PID_PITCH_KP    0.5/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_PITCH_KD.*/#define PID_PITCH_KD    0.05/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KP.*/#define PID_ROLL_KP     0.5/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_ROLL_KD.*/#define PID_ROLL_KD     0.05/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KP.*/#define PID_YAW_KP      0.2/' "$FLIGHT_CONFIG_FILE"
    sed -i 's/#define PID_YAW_KD.*/#define PID_YAW_KD      0.02/' "$FLIGHT_CONFIG_FILE"
    echo "✅ Configuração DEBUG aplicada"
}

function restore_config() {
    if [ -f "${FLIGHT_CONFIG_FILE}.backup" ]; then
        cp "${FLIGHT_CONFIG_FILE}.backup" "$FLIGHT_CONFIG_FILE"
        echo "✅ Configuração original restaurada"
    else
        echo "❌ Backup não encontrado!"
        exit 1
    fi
}

function show_current_config() {
    echo ""
    echo "📊 Configuração atual:"
    echo "====================="
    grep "PID_PITCH_KP\|PID_PITCH_KD\|PID_ROLL_KP\|PID_ROLL_KD\|PID_YAW_KP\|PID_YAW_KD" "$FLIGHT_CONFIG_FILE"
    echo ""
}

function compile_firmware() {
    echo "🔨 Compilando firmware..."
    make clean > /dev/null 2>&1
    if make; then
        echo "✅ Firmware compilado com sucesso!"
        echo "📦 Tamanho:"
        avr-size build/firmware.elf
    else
        echo "❌ Erro na compilação!"
        exit 1
    fi
}

# Verificar se arquivo existe
if [ ! -f "$FLIGHT_CONFIG_FILE" ]; then
    echo "❌ Arquivo de configuração não encontrado: $FLIGHT_CONFIG_FILE"
    exit 1
fi

# Processar argumentos
case "$1" in
    "conservative")
        backup_config
        apply_conservative
        ;;
    "aggressive")
        backup_config
        apply_aggressive
        ;;
    "smooth")
        backup_config
        apply_smooth
        ;;
    "debug")
        backup_config
        apply_debug
        ;;
    "restore")
        restore_config
        ;;
    "")
        show_help
        exit 0
        ;;
    *)
        echo "❌ Preset inválido: $1"
        show_help
        exit 1
        ;;
esac

# Mostrar configuração atual
show_current_config

# Perguntar se quer compilar
echo "🤔 Deseja compilar o firmware agora? (y/n)"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
    compile_firmware
else
    echo "⚠️  Lembre-se de compilar com: make clean && make"
fi

echo "🎉 Configuração concluída!"
