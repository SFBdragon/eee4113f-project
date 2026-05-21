CC      ?= cc
CFLAGS  := -Wall -Wextra -Wno-unused-parameter -std=c99 -O2 -I include
LDFLAGS := -pthread

SRC_LORA := src/serial.c src/at_protocol.c src/lora.c
SRC_WIFI := src/serial.c src/wifi.c

OBJ_LORA := $(SRC_LORA:.c=.o)
OBJ_WIFI := $(SRC_WIFI:.c=.o)

all: test_lora test_wifi

test_lora: $(OBJ_LORA) src/test_lora.o
	$(CC) $(LDFLAGS) -o $@ $^

test_wifi: $(OBJ_WIFI) src/test_wifi.o
	$(CC) $(LDFLAGS) -o $@ $^

lib: libdrivers.a

libdrivers.a: $(OBJ_LORA) $(OBJ_WIFI)
	ar rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f src/*.o test_lora test_wifi libdrivers.a liblora.a

.PHONY: all lib clean
