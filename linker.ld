/*
 * linker.ld - memory layout for the ARM MPS2-AN385 (Cortex-M3) under QEMU
 *
 * Tells the linker WHERE each piece of the program lives in memory.
 * Addresses come from AN385 Table 3-1 (Detailed System Memory Map).
 */

/* First instruction of the program. Used by debuggers/QEMU; the CPU itself
 * finds Reset_Handler through the vector table, not through this line. */
ENTRY(Reset_Handler)

/* ---------------------------------------------------------------------------
 * MEMORY: the physical regions that exist on the chip.
 *   r = readable, w = writable, x = executable
 * ------------------------------------------------------------------------- */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 4M   /* ZBT SRAM1, used as code memory */
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 4M   /* ZBT SRAM2&3, 4MB fitted */
}

/* Main stack size. The stack starts at the top of RAM and grows downward. */
_stack_size = 8K;
_estack     = ORIGIN(RAM) + LENGTH(RAM);           /* 0x20400000 */

/* ---------------------------------------------------------------------------
 * SECTIONS: which pieces of the compiled code go into which region, in order.
 * "> FLASH" means "place this in FLASH".
 * ------------------------------------------------------------------------- */
SECTIONS
{
  /* Vector table: MUST be first, at address 0x00000000. On reset the CPU
   * reads word 0 as the initial stack pointer and word 1 as the address of
   * Reset_Handler. KEEP stops the linker from deleting it as "unused",
   * since no code ever calls it directly. */
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  /* Code (.text) and constants (.rodata). These never change, so they stay
   * in FLASH and the CPU runs them from there. */
  .text :
  {
    . = ALIGN(4);
    *(.text .text.*)
    *(.rodata .rodata.*)
    . = ALIGN(4);
  } > FLASH

  /* Stack-unwinding tables. Small, and can be emitted by libgcc even with
   * -fno-exceptions, so give them a home rather than let them land anywhere. */
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH

  /* Constructors for C++ global objects. The compiler stores a pointer to
   * each one here; Reset_Handler must call them all before main(). */
  .init_array :
  {
    . = ALIGN(4);
    __init_array_start = .;
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    __init_array_end = .;
  } > FLASH

  /* Initialized globals (e.g. `int x = 5;`). They must live in RAM because
   * the program can change them, but RAM is empty at power-up, so their
   * starting values are STORED in FLASH ("AT > FLASH") and Reset_Handler
   * copies them into RAM.
   *   _sidata = where the initial values sit in FLASH (load address)
   *   _sdata/_edata = where they must end up in RAM (run address) */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  _sidata = LOADADDR(.data);

  /* Zero-initialized globals (e.g. `int count;`). Nothing is stored in FLASH,
   * since the values are all zero; Reset_Handler just zeroes this range. */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Not real data: reserves _stack_size bytes after .bss so the link FAILS
   * if globals + stack would not fit in RAM, instead of silently overlapping
   * at runtime. */
  ._stack_check (NOLOAD) :
  {
    . = ALIGN(8);
    . = . + _stack_size;
  } > RAM
}
