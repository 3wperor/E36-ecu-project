# Trigger simulator for Arduino Uno/Nano (ATmega328P).
#
#   make                       build build/stim.hex
#   make flash PORT=/dev/ttyUSB1          (Uno / new-bootloader Nano: 115200)
#   make flash PORT=/dev/ttyUSB1 BAUD=57600   (old-bootloader Nano)
#   make test                  host test of the pattern + simavr timing test

MCU   ?= atmega328p
F_CPU ?= 16000000UL
PORT  ?= /dev/ttyUSB1
BAUD  ?= 115200

BUILD := build
CFLAGS := -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu11 -Wall -Wextra -Werror

.PHONY: all flash test clean

all: $(BUILD)/stim.hex
	avr-size $(BUILD)/stim.elf

$(BUILD)/stim.elf: stim.c pattern.h
	@mkdir -p $(BUILD)
	avr-gcc $(CFLAGS) stim.c -o $@

$(BUILD)/stim.hex: $(BUILD)/stim.elf
	avr-objcopy -O ihex -R .eeprom $< $@

flash: $(BUILD)/stim.hex
	avrdude -p m328p -c arduino -P $(PORT) -b $(BAUD) -D -U flash:w:$<:i

$(BUILD)/test_pattern: test_pattern.c pattern.h
	@mkdir -p $(BUILD)
	gcc -std=gnu11 -Wall -Wextra -Werror test_pattern.c -o $@

$(BUILD)/sim_stim: sim_stim.c
	@mkdir -p $(BUILD)
	gcc -std=gnu11 -Wall -Wextra -I/usr/include/simavr $< -lsimavr -o $@

test: $(BUILD)/test_pattern $(BUILD)/sim_stim $(BUILD)/stim.elf
	./$(BUILD)/test_pattern
	./$(BUILD)/sim_stim $(BUILD)/stim.elf

clean:
	rm -rf $(BUILD)
