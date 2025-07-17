#!/bin/bash

# Script para monitoramento serial do nano-drone
# Uso: ./monitor.sh [porta_serial]

PORT=${1:-/dev/ttyUSB0}
BAUD=115200

echo "🚁 Nano-Drone Monitor"
echo "===================="
echo "Porta: $PORT"
echo "Baudrate: $BAUD"
echo ""
echo "Comandos especiais:"
echo "  Ctrl+A -> Sair"
echo "  Ctrl+C -> Interrupt"
echo ""
echo "Conectando..."
echo ""

# Verificar se porta existe
if [ ! -e "$PORT" ]; then
    echo "❌ Porta $PORT não encontrada!"
    echo "   Verifique a conexão ou especifique a porta:"
    echo "   ./monitor.sh /dev/ttyUSB0"
    exit 1
fi

# Usar screen para monitoramento
screen $PORT $BAUD
