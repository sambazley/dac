OBJS=boot.o vec.o main.o dac.o io.o spdif.o uart.o usb.o
DEPS=$(OBJS:.o=.d)

CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
AS = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy

CFLAGS += -std=c99 -mcpu=cortex-m0 -Wall -Wextra -Wpedantic -mthumb
LDFLAGS = -Tstm32.ld -mcpu=cortex-m0 -lc -lnosys -mthumb -Wl,--gc-sections

CFLAGS += -DSTM32F042x6

CFLAGS += -I/opt/STM32/STM32F0
CFLAGS += -I/usr/arm-none-eabi/include/

CFLAGS += -Ilibstm32usb
#LDFLAGS += -Llibstm32usb -lusb

ifeq ($(DEBUG),1)
CFLAGS += -Og -ggdb -DDEBUG
else
CFLAGS += -O2
endif

all: $(DEPS)

-include $(DEPS)

all: stm32.bin

stm32.bin: stm32
	$(OBJCOPY) $^ $@ -O binary

stm32: $(OBJS) libstm32usb/libusb.a
	$(CC) $^ -o $@ $(LDFLAGS)

libstm32usb/libusb.a:
	make -C libstm32usb TARGET=STM32F042x6

flash: stm32.bin
ifeq ($(DEBUG),1)
	openocd -f flash.cfg
else
	st-flash --reset write $^ 0x08000000
endif

%.d: %.c
	$(CC) $(CFLAGS) $< -MM -MT $(@:.d=.o) > $@

%.d: %.s
	$(CC) $(CFLAGS) $< -MM -MT $(@:.d=.o) > $@

clean:
	rm -f $(OBJS) $(DEPS) stm32 stm32.bin
	make -C libstm32usb clean

.PHONY: all flash clean
