# E36 ECU firmware — bare-metal avr-gcc build (no Arduino core).
#
#   make                 build build/ecu.hex for the ATmega1280
#   make MCU=atmega2560  build for a Mega 2560 dev board
#   make flash PORT=/dev/ttyUSB0
#   make test            host-side unit tests (native gcc)
#   make sim-test        boot the firmware in simavr and talk to it over UART

MCU    ?= atmega1280
F_CPU  ?= 16000000UL
PORT   ?= /dev/ttyUSB0

ifeq ($(MCU),atmega2560)
  AVRDUDE_PART := m2560
  AVRDUDE_PROG := wiring
  AVRDUDE_BAUD := 115200
else
  AVRDUDE_PART := m1280
  AVRDUDE_PROG := arduino
  AVRDUDE_BAUD := 57600
endif

CC      := avr-gcc
OBJCOPY := avr-objcopy
SIZE    := avr-size

BUILD   := build
SRC     := src/main.c $(wildcard src/hal/*.c) $(wildcard src/comms/*.c)
OBJ     := $(SRC:%.c=$(BUILD)/%.o)

CFLAGS  := -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu11 -g \
           -Wall -Wextra -Werror -Wshadow -Wstrict-prototypes \
           -ffunction-sections -fdata-sections -Isrc
LDFLAGS := -mmcu=$(MCU) -Wl,--gc-sections

.PHONY: all flash test sim-test clean size

all: $(BUILD)/ecu.hex size

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -MMD -c $< -o $@

$(BUILD)/ecu.elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

$(BUILD)/ecu.hex: $(BUILD)/ecu.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

size: $(BUILD)/ecu.elf
	$(SIZE) --format=avr --mcu=$(MCU) $< 2>/dev/null || $(SIZE) $<

flash: $(BUILD)/ecu.hex
	avrdude -p $(AVRDUDE_PART) -c $(AVRDUDE_PROG) -P $(PORT) -b $(AVRDUDE_BAUD) -D -U flash:w:$<:i

# ---- host unit tests -------------------------------------------------------
HOST_SRC := test/test_main.c src/comms/crc32.c src/comms/frame.c src/comms/commands.c

$(BUILD)/host_tests: $(HOST_SRC) $(wildcard src/*.h src/comms/*.h test/*.h)
	@mkdir -p $(BUILD)
	gcc -std=gnu11 -Wall -Wextra -Werror -Isrc -Itest $(HOST_SRC) -o $@

test: $(BUILD)/host_tests
	./$(BUILD)/host_tests

# ---- simavr end-to-end test ------------------------------------------------
$(BUILD)/sim_test: test/sim_test.c src/comms/crc32.c
	@mkdir -p $(BUILD)
	gcc -std=gnu11 -Wall -Wextra -Isrc -I/usr/include/simavr $^ -lsimavr -o $@

sim-test: $(BUILD)/ecu.elf $(BUILD)/sim_test
	./$(BUILD)/sim_test $(BUILD)/ecu.elf $(MCU)

clean:
	rm -rf $(BUILD)

-include $(OBJ:.o=.d)
