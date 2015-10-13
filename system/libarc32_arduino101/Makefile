ASM_SRC+=$(wildcard $(PWD)/bootcode/*.S)
ASM_SRC+=$(wildcard $(PWD)/drivers/*.S)
ASM_SRC+=$(wildcard $(PWD)/common/*.S)
C_SRC+=$(wildcard $(PWD)/bootcode/*.c)
C_SRC+=$(wildcard $(PWD)/drivers/*.c)
C_SRC+=$(wildcard $(PWD)/common/*.c)
C_SRC+=$(wildcard $(PWD)/framework/src/*.c)
C_SRC+=$(wildcard $(PWD)/framework/src/services/*.c)
C_SRC+=$(wildcard $(PWD)/framework/src/services/ble/*.c)
C_SRC+=$(wildcard $(PWD)/framework/src/cfw/*.c)
C_SRC+=$(wildcard $(PWD)/framework/src/infra/*.c)
C_SRC+=$(wildcard $(PWD)/framework/src/util/*.c)
C_SRC+=$(wildcard $(PWD)/framework/src/os/*.c)

ARCH=ARC
CC=arc-elf32-gcc
AS=arc-elf32-as
RM=rm -f
INSTALL=install

TARGET=arc32drv_arduino101
TARGET_LIB=lib$(TARGET).a

HWFLAGS=-mARCv2EM -mav2em -mlittle-endian

CFGFLAGS=-DCONFIG_SOC_GPIO_32 -DCONFIG_SOC_GPIO_AON -DINFRA_MULTI_CPU_SUPPORT -DCFW_MULTI_CPU_SUPPORT -DCONFIG_HAS_SHARED_MEM -DCONFIG_INFRA_IS_MASTER
OPTFLAGS=-g -Os -Wall -Werror
CFGFLAGS+=-DCONFIG_IPC_UART_BAUDRATE=1000000
INCLUDES=-I. -Icommon -Idrivers -Ibootcode -Iframework/include -Iframework/src/services/ble
EXTRA_CFLAGS=-D__CPU_ARC__ -DCLOCK_SPEED=32 -std=c99 -fno-reorder-functions -fno-asynchronous-unwind-tables -fno-omit-frame-pointer -fno-defer-pop -Wno-unused-but-set-variable -Wno-main -ffreestanding -fno-stack-protector -mno-sdata -ffunction-sections -fdata-sections
CFLAGS=$(HWFLAGS) $(OPTFLAGS) $(EXTRA_CFLAGS) $(CFGFLAGS) $(INCLUDES)

C_OBJ=$(C_SRC:.c=.o)
ASM_OBJ=$(ASM_SRC:.S=.o)

all: $(C_SRC) $(ASM_SRC) $(TARGET_LIB)

lib: $(TARGET_LIB)

$(TARGET_LIB): $(C_OBJ) $(ASM_OBJ)
	@echo "Link $@"
	$(AR) rcs $@ $^

%.o: %.S
	@echo "Assembling $<"
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.c
	@echo "Compiling $<"
	$(CC) -c $(CFLAGS) $< -o $@

lib_install: lib
	@if test "$(LIB_INSTALL_PATH)" = "" ; then \
		echo "LIB_INSTALL_PATH not set"; \
		exit 1; \
	else \
		$(INSTALL) $(TARGET_LIB) $(LIB_INSTALL_PATH); \
	fi \

clean:
	-$(RM) $(C_OBJ) $(ASM_OBJ) $(TARGET_LIB)
