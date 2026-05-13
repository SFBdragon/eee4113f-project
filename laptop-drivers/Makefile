# Simple Makefile for the LoRa driver test program.
# Cross-platform (Mac + Linux) POSIX C99.
#
# Targets:
#   make            - builds test_lora
#   make lib        - builds liblora.a (for Shaun's Rust integration)
#   make clean      - removes build artifacts

CC      ?= cc
CFLAGS  := -Wall -Wextra -Wno-unused-parameter -std=c99 -O2 -I include
LDFLAGS :=

SRC_LIB  := src/serial.c src/at_protocol.c src/lora.c
OBJ_LIB  := $(SRC_LIB:.c=.o)

all: test_lora

test_lora: $(OBJ_LIB) src/test_lora.o
	$(CC) $(LDFLAGS) -o $@ $^

lib: liblora.a

liblora.a: $(OBJ_LIB)
	ar rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o test_lora liblora.a

.PHONY: all lib clean
