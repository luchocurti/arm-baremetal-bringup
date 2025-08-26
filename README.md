# ARM Bare-Metal Bring Up (STM32L476RG + GCC)

This project is a **bare-metal bring up** for ARM Cortex-M development
using the STM32L476RG microcontroller (NUCLEO-L476RG board).\
It is designed as a minimal starting point for working with
**ARM bare-metal programming** using `gcc` and without relying on vendor
HAL libraries or CMSIS startup code.

------------------------------------------------------------------------

## Features

-   Written in **C11** with a custom `startup.c` (vector table, reset
    handler).
-   **Blink + USART echo example** in `main.c`:
    -   Toggles the onboard LED (GPIO).
    -   Non-blocking USART2 echo at 115200 baud.
    -   Uses `wfi` (wait for interrupt) for efficient low-power looping.
-   Minimal **linker script** (`link.ld`) tailored for STM32L476RG
    flash/SRAM layout.
-   **Makefile** with convenient targets:
    -   `all`      → Build ELF and BIN.
    -   `flash`    → Program via `st-flash`.
    -   `disasm`   → Generate annotated disassembly (`.lst`).
    -   `symbols`  → Dump symbols sorted by size (`.sym`).
    -   `sections` → Read section headers (`.sec`).
    -   `clean`    → Remove build artifacts.
-   No libc / no startup files → pure **freestanding** environment.
-   Explicit control of `.data` and `.bss` initialization in
    `Reset_Handler`.

------------------------------------------------------------------------

## Repository Structure

    .
    ├── link.ld         # Linker script
    ├── main.c          # Main program: LED blink + USART echo
    ├── Makefile        # Build, flash, disasm, symbols, sections, clean
    ├── README.md       # This file
    ├── startup.c       # Vector table and Reset_Handler
    └── stm32l476xx.h   # Minimal register definitions

------------------------------------------------------------------------

## Toolchain Requirements

A Unix-like shell with `make` available, the **ARM GNU toolchain**, and the **ST-Link utilities** are required:

``` bash
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch
sudo apt install stlink-tools
sudo apt install picocom minicom
```

------------------------------------------------------------------------

## Building

Compile everything into an ELF + BIN:

``` bash
make [all]
```

Artifacts are placed in the `build/` directory:
- `main.bin` → Raw binary for flashing
- `main.elf` → Executable ELF
- `main.lst` → Disassembly (via `make disasm`)
- `main.sec` → Section headers (via `make sections`)
- `main.sym` → Symbol table (via `make symbols`)

------------------------------------------------------------------------

## Flashing

Flash the program to the STM32L476RG board at address `0x08000000`:

``` bash
make flash
```

This uses **st-flash** from the `stlink` package.

------------------------------------------------------------------------

## Testing

For testing the project open a *second* terminal to view the device console (UART) and interact with the running firmware. Use the correct device node and baud rate your firmware uses (common example: `115200`, `8N1`, no flow control).

**picocom** (simple, minimal):
```bash
picocom -b 115200 /dev/ttyACM0
# or (to avoid resetting the port and avoid lockfiles)
picocom -b 115200 -r -l /dev/ttyACM0
```
To exit: press `Ctrl-A` then `Ctrl-X`

**minicom** (configurable, menu-driven):
```bash
# quick run:
minicom -D /dev/ttyACM0 -b 115200
```
To exit: press `Ctrl-A` then `Z` (help) and finally `X` (Exit)

------------------------------------------------------------------------

## Step-by-Step: From Source to Flash

This section explains how the code goes from **C source** to a **running
program** on the MCU:

1.  **Compile C to object files (`.o`)**

    ``` bash
    arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -c main.c -o build/main.o
    ```

    -   Each `.c` file becomes a `.o` file containing machine code +
        relocation info.

2.  **Link object files into an ELF (`.elf`)**

    ``` bash
    arm-none-eabi-gcc build/main.o build/startup.o -o build/main.elf -T link.ld -nostdlib
    ```

    -   The linker script (`link.ld`) places `.text`, `.data`, `.bss`,
        stack, etc.
    -   Produces `main.elf` with full symbol information.

3.  **Check binary size**

    ``` bash
    arm-none-eabi-size build/main.elf
    ```

    -   Prints RAM/flash usage summary.

4.  **Disassemble (optional)**

    ``` bash
    arm-none-eabi-objdump -D build/main.elf > build/main.lst
    ```

    -   Allows inspection of generated assembly.

5.  **Extract raw binary**

    ``` bash
    arm-none-eabi-objcopy -O binary build/main.elf build/main.bin
    ```

    -   Strips ELF headers, leaving raw flash image.

6.  **Flash the binary**

    ``` bash
    st-flash --reset write build/main.bin 0x08000000
    ```

    -   Writes program into flash at address `0x08000000` (STM32
        internal flash base).
    -   Resets MCU → The code runs.

------------------------------------------------------------------------

## Other optional make targets

- `make clean`
  - Remove the build directory and all generated files.

- `make disasm`
  - Generate a disassembly listing from the ELF file and write it to `main.lst`.
  - Useful for inspection and debugging.

- `make symbols`
  - Generate a list symbols (functions, vars, sizes) sorted by size and write it to `main.sym`.
  - Useful for inspection and debugging.

- `make sections`
  - List section headers with addresses and sizes, and write it to `main.sec`.
  - Useful for inspection and debugging.

------------------------------------------------------------------------

## Using This Project as a Template

1.  Clone the repo:

    ``` bash
    git clone https://github.com/<user>/arm-baremetal-bringup.git
    cd arm-baremetal-bringup
    ```

2.  Modify `main.c` to implement your own application logic.

3.  Add new source files in the `Makefile` under `SRCS`.

4.  Extend `stm32l476xx.h` with the registers needed.

5.  Adapt `link.ld` if using a different STM32 or Cortex-M MCU.

------------------------------------------------------------------------

## Notes and Tips

- Change `MCU`, `FPU`, `FLOAT_ABI` or `INSTRUCTIONS` in the Makefile to target a different Cortex-M core or floating point configuration.
- If new source files are added, update the `SRCS` Makefile variable with their paths.
- Adjust `OPT` variable in the Makefile to obtain the desired optimization level:
  - `-O0`    No optimization (slow, large, best for debugging)
  - `-O1`    Most common forms of optimization that requires no size versus speed decisions
  - `-O2`    Additional optimizations, such as instruction scheduling
  - `-O3`    Speed-focused optimizations (faster, often larger)
  - `-Ofast` Optimize for speed disregarding exact standards compliance
  - `-Og`    Optimize for debugging experience rather than speed or size
  - `-Os`    Size optimization (smaller code, sometimes slightly slower)
  - `-Oz`    Optimize for space aggressively rather than speed
- If flashing fails, verify the board connection, that `st-flash` supports your board, and that the write address (`0x08000000`) is correct for your target.

------------------------------------------------------------------------

## Linker Memory Map & Sections

### Summary (Addresses & Sizes)

- **FLASH**
  - Origin: `0x08000000`
  - End:    `0x08100000`
  - Size:   `0x100000` (**1 MB**)

- **RAM**
  - Origin: `0x20000000`
  - End:    `0x20018000`
  - Size:   `0x18000` (**96 KB**)

### High-level layout (visual)

```
FLASH (0x08000000 — 0x08100000)
+-----------------------------------------------------------------------+
| 0x08000000                                                            |
|  .isr_vector (KEEP)                                                   |
|  .text (code, functions)                                              |
|  .rodata (read-only data: const, literal strings)                     |
|                                                                       |
|  [end of .text]  == _etext  (address inside FLASH)                    |
|         v                                                             |
|  .data (LMA in FLASH)  @ AT(_etext)  --> initializers stored in FLASH |
|  (these bytes are the source copied at startup into RAM .data (VMA))  |
|                                                                       |
|  (possibly other load-time data...)                                   |
| 0x08100000  (FLASH end)                                               |
+-----------------------------------------------------------------------+

RAM (0x20000000 — 0x20018000)
+-----------------------------------------------------------------------+
| 0x20000000  (ORIGIN(RAM))                                             |
|  _sdata = start of .data (VMA)  --> .data runtime lives here          |
|  .data  (initialized variables)  [size: SIZE(.data) = link-time]      |
|                                                                       |
|  _edata  = end of .data  ( = _sdata + SIZE(.data) )                   |
|                                                                       |
|  _sbss = start of .bss  (immediately after .data)                     |
|  .bss  (uninitialized variables)  [size: SIZE(.bss) = link-time]      |
|                                                                       |
|  _ebss = end of .bss                                                  |
|                                                                       |
|  ._user_heap_stack  (align 8; PROVIDE(end = .) )                      |
|  - symbol `end` provided here: marks end of statically-used RAM       |
|  - typical usage: heap starts at `end` and grows up                   |
|                                                                       |
|  (free RAM between `end` and _estack available for heap & stack)      |
|                                                                       |
|  _estack = 0x20018000  (top of RAM; initial stack pointer)            |
| 0x20018000  (RAM end)                                                 |
+-----------------------------------------------------------------------+
```

### Sections table

| Section / Region | Location      | Attributes         | Start address | Size                | Purpose |
|------------------|---------------|--------------------|---------------|---------------------|---------|
| `.isr_vector`    | FLASH (start) | read-only, execute | `0x08000000`  | `N_vectors * 4`     | MCU vector table: reset + IRQ vectors. Kept by `KEEP()` so linker won't drop it. |
| `.text`          | FLASH         | r/o, execute       | `0x08000000` | link-time           | Program code (functions). Executed in-place from FLASH. |
| `.rodata`        | FLASH         | read-only          | link-time     | link-time           | Read-only constants, literal strings. |
| `.data`          | FLASH (LMA) / RAM (VMA) | read-write | `0x20000000` (VMA) | link-time | Initialized globals & statics. Initial values stored in FLASH at the LMA and copied to RAM at startup. |
| `.bss`           | RAM           | read-write         | link-time     | link-time           | Zero-initialized globals & statics. Not stored in FLASH; cleared at startup. |
| `COMMON`         | RAM           | read-write         | link-time     | merged into `.bss`  | Tentative definitions (uninitialized) collected into `.bss`. |
| `heap`           | RAM           | read-write         | `end` (symbol) | runtime             | Dynamic allocation region (malloc/new). Grows upward. |
| `stack`          | RAM           | read-write         | `0x20018000`  | depends on RAM left | Stack grows downward. Initial SP = `_estack`. |

------------------------------------------------------------------------

## Startup actions at Reset

1. The CPU loads the initial stack pointer from the first word of the vector table (commonly `_estack`).
2. Execution jumps to the reset handler (address at the second word of the vector table).
3. Startup code performs low-level initialisation, typically:
   - Optionally disable interrupts while relocating/clearing memory.
   - Copy `.data` from its LMA in FLASH (commonly starting at `_etext`) to its VMA in RAM (`_sdata`..`_edata`).
   - Zero-fill `.bss` (`_sbss`..`_ebss`).
   - Call constructors (for C++), initialize libc if used.
   - Jump to `main()`.

------------------------------------------------------------------------

## Documentation

### NUCLEO-L476RG board

- DB2196 - STM Nucleo-64 boards
- MB1136C - Schematic Layout
- UM1724 - STM32 Nucleo-64 boards (MB1136)
- UM1727 - Getting started with STM32 Nucleo board software development tools

### STM32L476RGT6U processor

- Arm Cortex-M4 Processor - Technical Reference Manual
- Armv7-M Architecture Reference Manual
- PM0214 - Programming manual
- RM0351 - Reference manual
- STM32L476xx Ultra-low-power Arm Cortex-M4 32-bit MCU+FPU - Datasheet

------------------------------------------------------------------------

## License

MIT License.\
Feel free to use, modify, and distribute for your own projects.
