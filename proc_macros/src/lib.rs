use proc_macro::TokenStream;
use quote::quote;

/// An alternative to the [core::arch::asm!] macro, which in addition to emitting the provided
/// assembly code also generates instructions that reads the CPU's performance counter value
/// before/after the assembly code is executed, and returns the delta. Can be used to count CPU
/// cycles, executed instructions, etc.
///
/// This macro guarantees that the first provided instruction will be placed at a 4-byte aligned
/// address.
///
/// Performance counting must already have been enabled elsewhere. This macro only manages the
/// performance counter count register.
/// 
/// To use this macro you must also declare `#![feature(asm_const)]` at the top of your source file.
///
/// Note: this macro does not allow a trailing comma after the last argument. This is different from
/// the [core::arch::asm!] macro, where a trailing comma is allowed.
///
/// This has only been tested on ESP32-C3 and ESP32-C6 chips.
#[proc_macro]
pub fn asm_with_perf_counter(input: TokenStream) -> TokenStream {

    // Note that `parsed` will generally contain both string literal and register definition
    // entries. This means that we can prepend instructions to it, and append register definitions
    // to it, and pass the combination of those to `asm!`, but not the other way around (e.g. we
    // cannot append instructions to it).
    let parsed: proc_macro2::TokenStream = syn::parse(input).unwrap();
    TokenStream::from(quote! {
      unsafe {
        /// The address of the Machine Performance Counter Event Register.
        const MPCER: usize = 0x7E0;
        /// The address of the Machine Performance Counter Count Register.
        const MPCCR: usize = 0x7E2;
        let cycles_start: u32;
        let cycles_end: u32;
        // Start by injecting instructions to reset and then read the performance counter.
        //
        // We must have the csr instruction below be part of the same asm! block as the provided
        // assembly instructions to measure. If we instead used a separate asm! block for the csr
        // instructions and one for the instructions to measure, then we'd accidentally measure the
        // instructions that are injected by the Rust compiler to initialize register values etc.
        asm!(
            // These nops are here to ensure further separation of any previous instructions from
            // this instruction, to remove any potential hazards that subsequent code may introduce,
            // and therefore try and increase repeatability of measurements.
            "nop","nop","nop","nop",
            ".align 4",
            "csrw {mpccr}, 0", // Resets the counter, 32-bit instruction.
            "csrr {cycles_start}, {mpccr}", // Read the initial counter value, 32-bit instruction.
            // The first instruction in #parsed is guaranteed to be 4-byte aligned, since the
            // previous instruction is 4-byte aligned and is 32 bits long.
            #parsed,
            cycles_start = out(reg) cycles_start,
            mpccr = const(MPCCR),
        );
        // Read the performance counter value.
        asm!(
            "csrr {cycles_end}, {mpccr}",
            // These nops are here to ensure further separation of any subsequent instructions from
            // this instruction, to remove any potential hazards that subsequent code may introduce,
            // and therefore try and increase repeatability of measurements.
            "nop","nop","nop","nop",
            cycles_end = out(reg) cycles_end,
            mpccr = const(MPCCR),
          );
        let count_type: u32;
        // Calculate the perf counter value. If the performance counter was counting CPU cycles or
        // instructions then we must subtract one from the count, to ensure we don't count the
        // initial `cssr` instruction that reads the start counter value at the start of the block.
        asm!("csrr {count_type}, {mpcer}", count_type = out(reg) count_type,
            mpcer = const(MPCER),
        );
        let offset = if count_type == 1<<0 || count_type == 1<<1 { 1 } else { 0 };
        cycles_end - cycles_start - offset
    }
    })
}
