ASM_SRC+=$(wildcard $(PWD)/bootcode/*.S)
ASM_SRC+=$(wildcard $(PWD)/drivers/*.S)
ASM_SRC+=$(wildcard $(PWD)/common/*.S)
C_SRC+=$(wildcard $(PWD)/bootcode/*.c)
C_SRC+=$(wildcard $(PWD)/drivers/*.c)
C_SRC+=$(wildcard $(PWD)/common/*.c)

ARCH=ARC
CC=arc-elf32-gcc
AS=arc-elf32-as
LD=$(CC)
OC=arc-elf32-objcopy
RM=rm -f
INSTALL=install
BINVER=$(PWD)/build/add_binary_version_header.py

BINVER_MAJOR=0
BINVER_MINOR=0
BINVER_PATCH=0
BINVER_STRING=ATP1ARC000-1516C5527

TARGET=arc32drv_edu
TARGET_LIB=lib$(TARGET).a
TARGET_ELF=$(TARGET).elf
TARGET_BIN=$(TARGET).bin

HWFLAGS=-mARCv2EM -mav2em -mlittle-endian
CFGFLAGS=-DCONFIG_SOC_GPIO_32
OPTFLAGS=-g -Os -Wall
INCLUDES=-Icommon -Ibootcode
EXTRA_CFLAGS=-D__CPU_ARC__ -DCLOCK_SPEED=32 -fno-reorder-functions -fno-asynchronous-unwind-tables -fno-omit-frame-pointer -fno-defer-pop -Wno-unused-but-set-variable -Wno-main -ffreestanding -fno-stack-protector -mno-sdata -ffunction-sections -fdata-sections 
CFLAGS=$(HWFLAGS) $(OPTFLAGS) $(EXTRA_CFLAGS) $(CFGFLAGS) $(INCLUDES)

LDFLAGS=-nostartfiles -nodefaultlibs -nostdlib -static \
-Wl,-X -Wl,-N -Wl,-mARCv2EM -Wl,-marcelf -Wl,--gc-sections \
-Wl,-Map,$(TARGET_ELF).map \
-Wl,--whole-archive $(TARGET_LIB) -Wl,--no-whole-archive \
-Wl,--start-group -lc -lm -lgcc

OCFLAGS=-S -O binary -R .note -R .comment -R COMMON -R .eh_frame

LINKER_CMD_FILE=$(PWD)/build/linker.cmd

C_OBJ=$(C_SRC:.c=.o)
ASM_OBJ=$(ASM_SRC:.S=.o)

all: $(C_SRC) $(ASM_SRC) main.c $(TARGET_BIN)

app: $(TARGET_BIN)

lib: $(TARGET_LIB)

$(TARGET_LIB): $(C_OBJ) $(ASM_OBJ)
	@echo "Link $@"
	$(AR) rcs $@ $^

$(TARGET_ELF): $(TARGET_LIB) main.o
	@echo "Link $@"
	$(LD) -T $(LINKER_CMD_FILE) $(LDFLAGS) -o $@ $^

$(TARGET_BIN): $(TARGET_ELF)
	$(OC) $(OCFLAGS) $^ $@
	$(BINVER) --major $(BINVER_MAJOR) --minor $(BINVER_MINOR) --patch $(BINVER_PATCH) --version_string $(BINVER_STRING) $@

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
	-$(RM) $(C_OBJ) $(ASM_OBJ) $(TARGET_LIB) $(TARGET_ELF) $(TARGET_BIN) $(TARGET_ELF).map
