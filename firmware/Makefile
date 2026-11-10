# E36 ECU firmware — bare-metal avr-gcc build (no Arduino core).
#
#   make                 build build/ecu.hex for the ATmega1280
#   make MCU=atmega2560  build for a Mega 2560 dev board
#   make flash PORT=/dev/ttyUSB0
#   make test            host-side unit tests (native gcc)
#   make sim-test        boot the firmware in simavr and talk to it over UART
#   make tuner-test      the tuner GUI against the firmware in simavr

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
SRC     := src/main.c $(wildcard src/hal/*.c) $(wildcard src/comms/*.c) $(wildcard src/decoder/*.c) \
           $(wildcard src/sched/*.c) $(wildcard src/tune/*.c) $(wildcard src/engine/*.c) \
           src/generated/tune_defaults.c
OBJ     := $(SRC:%.c=$(BUILD)/%.o)

CFLAGS  := -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu11 -g \
           -Wall -Wextra -Werror -Wshadow -Wstrict-prototypes \
           -ffunction-sections -fdata-sections -Isrc
LDFLAGS := -mmcu=$(MCU) -Wl,--gc-sections

.PHONY: all flash test sim-test sim-pty tuner-test clean size

all: $(BUILD)/ecu.hex size

# Generated sources (committed) are rebuilt when the definition changes.
GENERATED := src/generated/tune_def.h src/generated/tune_defaults.c src/generated/realtime.h \
             ../tuner/e36ecu.ini ../tuner/ecu_def.json
$(GENERATED) &: ../definitions/ecu.toml ../tools/codegen/gen.py
	python3 ../tools/codegen/gen.py
$(OBJ): | $(GENERATED)

# The decoder and scheduler ISRs are on the critical path: optimise them
# for speed, not size.
$(BUILD)/src/decoder/decoder.o: CFLAGS += -O2
$(BUILD)/src/sched/sched.o: CFLAGS += -O2

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
HOST_CFLAGS := -std=gnu11 -O2 -Wall -Wextra -Werror -Isrc -Itest -I../tools/stim

$(BUILD)/host_tests: $(HOST_SRC) $(wildcard src/*.h src/comms/*.h src/decoder/*.h src/sched/*.h test/*.h)
	@mkdir -p $(BUILD)
	gcc $(HOST_CFLAGS) $(HOST_SRC) -o $@

$(BUILD)/test_decoder: test/test_decoder.c $(wildcard src/decoder/*.h) ../tools/stim/pattern.h
	@mkdir -p $(BUILD)
	gcc $(HOST_CFLAGS) test/test_decoder.c -lm -o $@

$(BUILD)/test_sched: test/test_sched.c $(wildcard src/decoder/*.h src/sched/*.h) ../tools/stim/pattern.h
	@mkdir -p $(BUILD)
	gcc $(HOST_CFLAGS) test/test_sched.c -lm -o $@

$(BUILD)/test_storage: test/test_storage.c src/comms/crc32.c src/generated/tune_defaults.c $(wildcard src/tune/*.h src/generated/*.h)
	@mkdir -p $(BUILD)
	gcc $(HOST_CFLAGS) test/test_storage.c src/comms/crc32.c src/generated/tune_defaults.c -o $@

$(BUILD)/test_engine: test/test_engine.c src/generated/tune_defaults.c $(wildcard src/engine/*.h src/generated/*.h)
	@mkdir -p $(BUILD)
	gcc $(HOST_CFLAGS) test/test_engine.c src/generated/tune_defaults.c -lm -o $@

test: $(BUILD)/host_tests $(BUILD)/test_decoder $(BUILD)/test_sched $(BUILD)/test_storage $(BUILD)/test_engine
	python3 ../tools/codegen/gen.py --check
	./$(BUILD)/host_tests
	./$(BUILD)/test_decoder
	./$(BUILD)/test_sched
	./$(BUILD)/test_storage
	./$(BUILD)/test_engine
	cd .. && python3 -m unittest -q tools/codegen/test_gen.py tools/test_ecu_cli.py
	cd ../tuner && QT_QPA_PLATFORM=offscreen python3 -m unittest -q tests.test_core tests.test_gui

# The tuner (GUI included) against the real firmware in simavr, over a pseudo-terminal.
tuner-test: $(BUILD)/ecu.elf $(BUILD)/sim_pty
	cd ../tuner && QT_QPA_PLATFORM=offscreen python3 -m unittest -q tests.test_firmware

# ---- simavr end-to-end test ------------------------------------------------
$(BUILD)/sim_test: test/sim_test.c src/comms/crc32.c ../tools/stim/pattern.h
	@mkdir -p $(BUILD)
	gcc -std=gnu11 -O2 -Wall -Wextra -Isrc -I../tools/stim -I/usr/include/simavr test/sim_test.c src/comms/crc32.c -lsimavr -lm -o $@

# The crank ISR (TIMER5_CAPT = __vector_46) address and size are passed in so
# the test can time it.
sim-test: $(BUILD)/ecu.elf $(BUILD)/ecu.sym $(BUILD)/sim_test
	./$(BUILD)/sim_test $(BUILD)/ecu.elf $(MCU) $(BUILD)/ecu.sym

$(BUILD)/ecu.sym: $(BUILD)/ecu.elf
	avr-nm -S $< > $@

clean:
	rm -rf $(BUILD)

-include $(OBJ:.o=.d)

# ---- simulated ECU on a pseudo-terminal, for testing PC tools ---------------
RPM     ?= 3000
SECONDS ?= 20

$(BUILD)/sim_pty: test/sim_pty.c ../tools/stim/pattern.h
	@mkdir -p $(BUILD)
	gcc -std=gnu11 -O2 -Wall -Wextra -I../tools/stim -I/usr/include/simavr test/sim_pty.c -lsimavrparts -lsimavr -lutil -lpthread -o $@

sim-pty: $(BUILD)/ecu.elf $(BUILD)/sim_pty
	./$(BUILD)/sim_pty $(BUILD)/ecu.elf $(RPM) $(SECONDS)
