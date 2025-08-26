# Target name
TARGET = main

# Toolchain programs
CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
LD      = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size
SYMBOLS = arm-none-eabi-nm
READEFL = arm-none-eabi-readelf
FLASH   = st-flash

# MCU and CPU settings
MCU     = cortex-m4
FPU     = fpv4-sp-d16
FLOAT_ABI = hard
INSTRUCTIONS = -mthumb

# Optimization flag
OPT = -O0

# Directories
SRC_DIR = .
BUILD_DIR = build

# Source files
SRCS = \
    $(SRC_DIR)/main.c \
    $(SRC_DIR)/startup.c

# Linker script
LDSCRIPT = $(SRC_DIR)/link.ld

# Compiler flags
CFLAGS  = -mcpu=$(MCU) $(INSTRUCTIONS) -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI)
CFLAGS += -Wall -Wextra -Werror $(OPT) -g3
# Ensure no unwanted libc startup code
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -I$(SRC_DIR) -std=c11
# Debugging helpers for .data/.bss
CFLAGS += -fno-common -fno-builtin -fverbose-asm
#  -fno-common: force globals into .bss or .data, not common section
#  -fno-builtin: avoid compiler replacing code with builtins
#  -fverbose-asm: richer disassembly

# Linker flags
LDFLAGS  = -T$(LDSCRIPT)
# Remove unused sections, keeping binary size small
LDFLAGS += -nostdlib -Wl,--gc-sections

# Build object list
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)

# Default build target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Link ELF file
$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)
	$(SIZE) $@

# Create BIN file from ELF
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Disassemble for inspection
disasm: $(BUILD_DIR)/$(TARGET).elf
	$(OBJDUMP) -D $< > $(BUILD_DIR)/$(TARGET).lst

# List symbols (functions, vars, sizes) sorted by size
symbols: $(BUILD_DIR)/$(TARGET).elf
	$(SYMBOLS) --size-sort -r $< > $(BUILD_DIR)/$(TARGET).sym

# List sections with addresses and sizes
sections: $(BUILD_DIR)/$(TARGET).elf
	$(READEFL) -S $< > $(BUILD_DIR)/$(TARGET).sec

# Flash to NUCLEO board using st-flash
flash: $(BUILD_DIR)/$(TARGET).bin
	$(FLASH) --reset write $< 0x08000000

# Clean build directory
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean flash disasm symbols sections
