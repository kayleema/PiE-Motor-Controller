# Compilation Settings
DEVICE     = atmega328
CLOCK      = 16000000
OBJECTS    = blink.o

# Code Upload Settings
SERIAL_PORT = /dev/tty.usbserial-A900cehS
SERIAL_BAUD = 9600
PROGRAMMER_FLAGS = -c avrisp -p m328p -P $(SERIAL_PORT) -F -b $(SERIAL_BAUD)
FUSES      = -U lfuse:w:0xFF:m

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER_FLAGS)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	main.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

