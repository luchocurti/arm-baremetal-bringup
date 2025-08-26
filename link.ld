/* link.ld */
/* Memory map + Section placement */
/* Linker script for STM32L476RG (1MB Flash, 96KB RAM) */

ENTRY(Reset_Handler)

/* Memory Regions */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

/* Symbols for startup.c */
_estack = ORIGIN(RAM) + LENGTH(RAM);
_sidata = LOADADDR(.data);

SECTIONS
{
    /* Vector table and code */
    .text :
    {
        KEEP(*(.isr_vector))       /* Vector table */
        *(.text*)                  /* Code */
        *(.rodata*)                /* Read-only data */
        _etext = .;                /* End of text, used as LMA for .data */
    } > FLASH

    /* Initialized data section in RAM, load from FLASH */
    .data : AT(_etext)
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM

    /* Uninitialized data section (BSS) */
    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    /* Stack pointer */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        /* Provide a symbol called `end` (useful for heap start) */
        PROVIDE(end = .);
    } > RAM
}
