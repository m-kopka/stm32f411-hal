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
        *(.vector_table)
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    }> FLASH

    _la_data = LOADADDR(.data);

    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
        . = ALIGN(4);
    }> SRAM AT> FLASH

    .bss :
    {
        _sbss = .;
        *(.bss*)
        _ebss = .;
        . = ALIGN(4);
    }> SRAM
}