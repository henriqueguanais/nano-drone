# Makefile para projeto AVR ATmega2560

# Configurações
MCU     = atmega2560
F_CPU   = 16000000UL
PORT    = /dev/ttyACM0
BAUD    = 115200
PROGRAMMER = wiring

# Diretórios
BUILD_DIR = build
SRC_DIRS = src lib/freertos/src

# Arquivos fonte de múltiplos diretórios
SRC = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.c))
OBJ = $(patsubst %.c, $(BUILD_DIR)/%.o, $(SRC))

# Compilador e flags
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -Wall -Os -DF_CPU=$(F_CPU) -Iinclude -Ilib/freertos/include
LDFLAGS = -mmcu=$(MCU)

.PHONY: all flash clean

all: $(BUILD_DIR)/firmware.hex

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/firmware.elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^

$(BUILD_DIR)/firmware.hex: $(BUILD_DIR)/firmware.elf
	avr-objcopy -O ihex -R .eeprom $< $@

flash: $(BUILD_DIR)/firmware.hex
	avrdude -p m2560 -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -D -U flash:w:$<:i

clean:
	rm -rf $(BUILD_DIR)
