MEMORY
{
  ram (rwx)        : ORIGIN = 0x20000000, LENGTH = 8K
  stack(rwx)       : ORIGIN = 0x20001FF8, LENGTH = 0K

  rom (rx)         : ORIGIN = 0x00000000, LENGTH = 31K
  fixedrom (rx)    : ORIGIN = 0x00007C00, LENGTH = 1K  /* Includes flasher program etc., cannot be rewritten in-application (STM32 bootloader must be used)*/
}

SECTIONS
{
    .nvic_vector : 
    {
        *(vectors)    /* Vector table */
    } >rom

    . = ALIGN(8);

    .text : 
    {
        *(.text)      /* Program code */
        *(.text.*)
        *(.rodata)    /* Read only data */
        *(.rodata.*)        
    } >rom

    . = ALIGN(8);

    _DATAI_BEGIN = LOADADDR(.data);

    .data : 
    {
        _DATA_BEGIN = .;
        *(.data)      /* Data memory */
        *(.data.*)        
        _DATA_END = .;
    } >ram AT>rom

    . = ALIGN(8);

    .bss :
    {
        _BSS_BEGIN = .;
        *(.bss)
        *(COMMON)        
        _BSS_END = .;
    } >ram 

    . = ALIGN(8);

    .heap :
    {
        _HEAP = .;
    } >ram

    .stack :
    {
        _STACKTOP = .;
    } >stack

    . = ALIGN(8);

    .flasher : 
    {
        *(.flasher)
        *(.flasher.*)
    } >fixedrom

    . = ALIGN(8);

}  
