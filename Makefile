# Makefile para projeto AVR ATmega328P

# Configurações
MCU     = atmega2560
F_CPU   = 16000000UL
PORT    = COM7  # Altere para a porta correta do seu sistema
BAUD    = 115200
PROGRAMMER = arduino

# Diretórios
BUILD_DIR = build
SRC_DIR = src
INC_DIR = include
LIB_DIR = lib

# Arquivos fonte
SRC = $(wildcard $(SRC_DIR)/*.c)
# Arquivos específicos do FreeRTOS necessários para FreeRTOS.h e task.h
LIB_SRC = $(LIB_DIR)/freertos/src/tasks.c \
		  $(LIB_DIR)/freertos/src/list.c \
		  $(LIB_DIR)/freertos/src/port.c \
		  $(LIB_DIR)/freertos/src/heap_3.c \
		  $(LIB_DIR)/freertos/src/timers.c \
		  $(LIB_DIR)/freertos/src/queue.c
ALL_SRC = $(SRC) $(LIB_SRC)
OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRC)) \
	  $(patsubst $(LIB_DIR)/freertos/src/%.c,$(BUILD_DIR)/%.o,$(LIB_SRC))

# Compilador e flags
CC = avr-gcc
CFLAGS = -mmcu=$(MCU) -Wall -Os -DF_CPU=$(F_CPU) -I$(INC_DIR) -I$(LIB_DIR)/freertos/include
LDFLAGS = -mmcu=$(MCU)

.PHONY: all flash clean

all: $(BUILD_DIR)/firmware.hex

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@if not exist $(@D) mkdir $(@D) 2>nul || true  # Compatível com Windows e Unix
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(LIB_DIR)/freertos/src/%.c
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