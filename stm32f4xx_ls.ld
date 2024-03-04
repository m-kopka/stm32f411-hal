ENTRY(Reset_Handler)

MEMORY 
{
    FLASH(rx):ORIGIN =0x08000000,LENGTH =512K
    SRAM(rxx):ORIGIN =0x20000000,LENGTH =128K
}

SECTIONS
{
    .text :
    {
        KEEP(*(.vector_table))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);

        _la_data = .;    /* load address of .data in FLASH */
    }> FLASH

    .data :
    {
        . = ALIGN(4);
        _sdata = .;     /* start address of .data in SRAM */
        *(.data*)
        . = ALIGN(4);
        _edata = .;     /* end address of .data in SRAM */

    }> SRAM AT> FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;      /* start address of .bss in SRAM */
        *(.bss*)
        . = ALIGN(4);
        _ebss = .;      /* end address of .bss in SRAM */

    }> SRAM
}