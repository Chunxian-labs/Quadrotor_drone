ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 20K
}

_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Keep executable Flash content and writable RAM content in separate
   ELF load segments.  This prevents the linker from emitting an RWX
   LOAD segment for the initialized-data image. */
PHDRS
{
  text PT_LOAD FLAGS(5); /* Read + execute */
  data PT_LOAD FLAGS(6); /* Read + write */
}

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)); . = ALIGN(4); } > FLASH :text
  .text :
  {
    . = ALIGN(4); *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .;
  } > FLASH :text
  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } > FLASH :text
  .ARM.extab : { . = ALIGN(4); *(.ARM.extab* .gnu.linkonce.armextab.*) . = ALIGN(4); } > FLASH :text
  .ARM : { . = ALIGN(4); __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; . = ALIGN(4); } > FLASH :text
  .preinit_array : { . = ALIGN(4); PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH :text
  .init_array : { . = ALIGN(4); PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH :text
  .fini_array : { . = ALIGN(4); PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH :text
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data) *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH :data
  /* These sections exist only in RAM and must not join the ELF data LOAD
     segment.  Assigning them to :data makes ld align their unused LMA. */
  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; __bss_start__ = _sbss; *(.bss) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; __bss_end__ = _ebss; } > RAM :NONE
  ._user_heap_stack (NOLOAD) : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . = . + _Min_Heap_Size; . = . + _Min_Stack_Size; . = ALIGN(8); } > RAM :NONE
}
