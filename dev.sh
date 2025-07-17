#!/bin/bash

# Script completo para desenvolvimento do nano-drone
# Uso: ./dev.sh [build|upload|monitor|clean|info]

ACTION=${1:-help}

case $ACTION in
    build)
        echo "🔨 Compilando firmware..."
        make clean && make
        if [ $? -eq 0 ]; then
            echo "✅ Compilação concluída!"
            avr-size --format=avr --mcu=atmega328p build/firmware.elf
        else
            echo "❌ Erro na compilação!"
            exit 1
        fi
        ;;
    
    upload)
        echo "📡 Upload do firmware..."
        ./upload.sh $2
        ;;
    
    monitor)
        echo "📊 Monitoramento serial..."
        ./monitor.sh $2
        ;;
    
    clean)
        echo "🧹 Limpando arquivos..."
        make clean
        echo "✅ Limpeza concluída!"
        ;;
    
    info)
        echo "📋 Informações do projeto:"
        echo "=========================="
        echo "Arquivo principal: src/main.c"
        echo "Configuração: include/flight_config.h"
        echo "Documentação: *.md"
        echo ""
        if [ -f "build/firmware.elf" ]; then
            echo "💾 Uso de memória:"
            avr-size --format=avr --mcu=atmega328p build/firmware.elf
        else
            echo "⚠️  Firmware não compilado (execute ./dev.sh build)"
        fi
        ;;
    
    all)
        echo "🚀 Build completo + Upload..."
        make clean && make
        if [ $? -eq 0 ]; then
            ./upload.sh $2
        else
            echo "❌ Erro na compilação!"
            exit 1
        fi
        ;;
    
    help|*)
        echo "🚁 Nano-Drone Development Script"
        echo "================================"
        echo ""
        echo "Uso: ./dev.sh [comando] [opções]"
        echo ""
        echo "Comandos:"
        echo "  build     - Compilar firmware"
        echo "  upload    - Upload para Arduino (./dev.sh upload /dev/ttyUSB0)"
        echo "  monitor   - Monitor serial (./dev.sh monitor /dev/ttyUSB0)"
        echo "  clean     - Limpar arquivos de build"
        echo "  info      - Mostrar informações do projeto"
        echo "  all       - Build + Upload em uma operação"
        echo "  help      - Mostrar esta ajuda"
        echo ""
        echo "Exemplos:"
        echo "  ./dev.sh build"
        echo "  ./dev.sh upload /dev/ttyUSB0"
        echo "  ./dev.sh all /dev/ttyUSB0"
        echo ""
        ;;
esac
