#!/bin/bash

# Script para upload automático do firmware
# Uso: ./upload.sh [porta_serial]

# Configurações padrão
PORT=${1:-/dev/ttyUSB0}
BAUD=57600

echo "🚁 Nano-Drone Upload Script"
echo "=========================="

# Verificar se arquivo existe
if [ ! -f "build/firmware.hex" ]; then
    echo "❌ Arquivo build/firmware.hex não encontrado!"
    echo "   Execute 'make' primeiro."
    exit 1
fi

# Verificar se porta existe
if [ ! -e "$PORT" ]; then
    echo "❌ Porta $PORT não encontrada!"
    echo "   Verifique a conexão ou especifique a porta:"
    echo "   ./upload.sh /dev/ttyUSB0"
    exit 1
fi

echo "📋 Configurações:"
echo "   Porta: $PORT"
echo "   Baudrate: $BAUD"
echo "   Arquivo: build/firmware.hex"

# Mostrar informações do firmware
echo ""
echo "💾 Informações do firmware:"
avr-size --format=avr --mcu=atmega328p build/firmware.elf

echo ""
echo "🔄 Gravando firmware..."

# Upload usando avrdude
avrdude -p atmega328p -c arduino -P $PORT -b $BAUD -U flash:w:build/firmware.hex:i

if [ $? -eq 0 ]; then
    echo "✅ Upload concluído com sucesso!"
    echo ""
    echo "📡 Para monitorar via serial:"
    echo "   screen $PORT 115200"
    echo "   ou"
    echo "   ./monitor.sh"
else
    echo "❌ Erro durante o upload!"
    exit 1
fi
