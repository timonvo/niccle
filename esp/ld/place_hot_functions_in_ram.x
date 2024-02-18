/* Ensures that key functions are placed in RAM. This ensures that we don't get stuck waiting on the
CPU to load instructions from flash into RAM at critical times, e.g. while processing an incoming
packet. We must do this via a linker script, because we cannot use the `#[ram]` annotation on code
for crates we depend on and don't own ourselves. */

SECTIONS {
  .rwtext.thingbuf : ALIGN(4)
  {
    . = ALIGN (4);
    /* Functions that might be called within a critical section block, or within an interrupt.
    Note that if the CPU is blocked on loading code from flash, even if that code was not
    running in an interrupt, then any incoming interrupts are blocked behind that as well.
    Hence we must cast fairly wide net here and include most of the code in our binary in order
    to achieve maximum throughput.
     */
    *(.text.*bbqueue*)
    *(.text.*bitvec*)
    *(.text.*crc32fast*)
    *(.text.*critical_section*)
    *(.text.*memcpy*)
    *(.text.*niccle*)
    *(.text.*smoltcp*)
    . = ALIGN(4);
  } > RWTEXT
}