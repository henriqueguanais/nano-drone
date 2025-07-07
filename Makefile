# Makefile para projeto AVR ATmega328P

# Configurações
MCU     = atmega328p
F_CPU   = 16000000UL
PORT    = COM6   # Altere para a porta correta do seu sistema
BAUD    = 115200
PROGRAMMER = arduino

# Diretórios
BUILD_DIR = build
SRC_DIR = src
INC_DIR = include

# Arquivos fonte
SRC = $(wildcard $(SRC_DIR)/*.c)
OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRC))

# Compilador e flags
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -Wall -Os -DF_CPU=$(F_CPU) -I$(INC_DIR)
LDFLAGS = -mmcu=$(MCU)

.PHONY: all flash clean

all: $(BUILD_DIR)/firmware.hex

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@if not exist $(@D) mkdir $(@D) 2>nul || true  # Compatível com Windows e Unix
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/firmware.elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^

$(BUILD_DIR)/firmware.hex: $(BUILD_DIR)/firmware.elf
	avr-objcopy -O ihex -R .eeprom $< $@

flash: $(BUILD_DIR)/firmware.hex
	avrdude -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$<:i

clean:
	rm -rf $(BUILD_DIR)