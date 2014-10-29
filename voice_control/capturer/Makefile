#CP=g++
CP=clang++
CFLAGS=-Wall -Wextra -std=c99 -pedantic -g
CPFLAGS=-Wall -Wextra -std=c++11 -g #-pedantic
LIBS=-I/usr/local/include/libfreenect -I/usr/local/include/libusb-1.0  -L/usr/local/lib -lfreenect
PROGNAME=capturer

all: $(PROGNAME)

.PHONY: clean

capturer: src/capturer.cpp
	mkdir -p bin
	$(CP) $(CPFLAGS) $(LIBS) $< -o bin/$@

clean:
	-rm -rf bin

play:
	-play -e signed-integer -r 16k -b 16 tmp.raw

play32:
	-play -e signed-integer -r 16k -b 32 tmp.raw
