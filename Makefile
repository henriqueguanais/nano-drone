# Makefile para projeto AVR ATmega328P

# Configurações
MCU     = atmega328p
F_CPU   = 16000000UL
PORT    = /dev/ttyUSB0
BAUD    = 57600
PROGRAMMER = arduino

# Diretórios
BUILD_DIR = build

# se nao for usar o tests, remova tests do SRC_DIRS
SRC_DIRS = src tests

# Arquivos fonte de múltiplos diretórios
SRC = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.c))
OBJ = $(patsubst %.c, $(BUILD_DIR)/%.o, $(SRC))

# Compilador e flags
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -Wall -Os -DF_CPU=$(F_CPU) -Iinclude
LDFLAGS = -mmcu=$(MCU)

.PHONY: all flash clean

all: $(BUILD_DIR)/firmware.hex

# Compila cada .c em .o no diretório build/
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

# Linka todos os objetos em firmware.elf
$(BUILD_DIR)/firmware.elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^

# Gera o .hex a partir do .elf
$(BUILD_DIR)/firmware.hex: $(BUILD_DIR)/firmware.elf
	avr-objcopy -O ihex -R .eeprom $< $@

# Grava no Arduino
flash: $(BUILD_DIR)/firmware.hex
	avrdude -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$<:i

clean:
	rm -rf $(BUILD_DIR)
