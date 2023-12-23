//! Provides low-level functions that implement transmission functionality in assembly code, using
//! the dedicated IO feature of the ESP32-C6 chip.

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use core::arch::asm;
use hal::prelude::*;
use log::warn;
use niccle_proc_macros::asm_with_perf_counter;

/// For now the implementation is hardcoded to always use CPU output signal 0 when using dedicated
/// IO on the TX pin. In the future we could try to make this more flexible, e.g. using an argument,
/// or perhaps even a const generic parameter with a const expression bound.
#[cfg(feature = "esp32c3")]
pub const TX_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal = hal::gpio::OutputSignal::CPU_GPIO_0;
#[cfg(feature = "esp32c6")]
pub const TX_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal = hal::gpio::OutputSignal::CPU_GPIO_OUT0;
/// The dedicated IO output signal index to use (from 0 to 7, where 0 corresponds to CPU_GPIO_OUT0).
/// Based on whatever [TX_CPU_OUTPUT_SIGNAL] is set to.
#[cfg(feature = "esp32c3")]
const TX_CPU_OUTPUT_SIGNAL_CSR_IDX: isize =
    (TX_CPU_OUTPUT_SIGNAL as isize) - hal::gpio::OutputSignal::CPU_GPIO_0 as isize;
#[cfg(feature = "esp32c6")]
const TX_CPU_OUTPUT_SIGNAL_CSR_IDX: isize =
    (TX_CPU_OUTPUT_SIGNAL as isize) - hal::gpio::OutputSignal::CPU_GPIO_OUT0 as isize;

/// This CSR isn't defined by the esp-rs/esp-hal framework yet (nor by the espressif/svd files), so
/// we hardcode their addresses. See the "1.14 Dedicated IO" chapter of the technical reference
/// manual.
const CSR_CPU_GPIO_OUT: u32 = 0x805;

/// Transmits a single link test pulse.
///
/// Note that `_tx_periph` is technically unused, but by requiring a reference to it we ensure
/// mutually exclusive use of the pin's corresponding `TX_CPU_OUTPUT_SIGNAL`.
///
// Note: the use of #[ram] is important, as it it ensures that there won't be any instruction
// execution delays, and each instruction will take 6.25ns. See `A note on writing precisely timed
// assembly code` on [transmit_packet] below.
#[ram]
pub fn transmit_ltp(_tx_periph: &mut impl hal::gpio::OutputPin) {
    unsafe {
        asm!(
            // Set the output high, and keep it high for 62.5ns (10 CPU cycles).
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // 1 cycle
            // 9 cycles
            "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
            // Set the output low.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}",
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
        );
    }
}

/// The address of the Machine Performance Counter Event Register.
const MPCER: usize = 0x7E0;
/// The address of the Machine Performance Counter Mode Register.
const MPCMR: usize = 0x7E1;

/// Transmits the given Ethernet packet over the transmission line.
///
/// See [Phy::transmit_packet] for more info.
///
/// # A note on writing precisely timed assembly code
///
/// When transmitting data, each half of a Manchester-encoded bit symbol should take 50ns, for a
/// total of 100ns per bit symbol. Therefore, given a clock speed of 160MHz (enforced above), we
/// must use exactly eight CPU cycles between each signal transition. We can't really rely on the
/// compiler to generate code for us that adheres to this requirement, so we must write the whole
/// transmission control loop in assembly code instead, using the dedicated IO feature to emit
/// signals using just a single CPU cycle.
///
/// Unsurprisingly that has its own perils. At 160MHz, each clock cycle takes 6.25ns. Some
/// instructions take just a single cycle to execute, but some can take more, and the behavior is
/// chip dependent and situation dependent. The `cycle_counting` example binary contains a lot more
/// useful information on this topic, as does the discussion at
/// https://ctrlsrc.io/posts/2023/counting-cpu-cycles-on-esp32c3-esp32c6/.
///
/// For this particular function, the following insights gained from the `cycle_counting` example
/// binary are most relevant (focusing on the behavior on ESP32-C6 for now):
///
/// * `csrr`, `andi`, xori`, and other simple instructions not otherwise called out below take 1 CPU
///   cycle.
/// * `lbu` instructions take 2 CPU cycles, as long as there is a data dependency immediately after
///   them (i.e. an instruction that uses the result of the load instruction).
/// * `bne` instructions in this type of loop takes 3 CPU cycles when not taken (at the end of the
///   iteration) if the instruction falls on a 4 byte-aligned address, and 4 CPU cycles when not
///   taken and unaligned. They take 2 CPU cycles when taken for the first time (in the first
///   iteration), and 1 CPU cycle when taken after having been taken before already (in the second
///   to second-to-last iterations).
///
/// See the [cycle_counting::branch_back_usually_taken] and
/// [cycle_counter::_branch_back_usually_taken_w_lbu] for the benchmarks showing this behavior for
/// the ESP32-C6 CPU.
///
/// Note: the use of #[ram] is also important, as it it ensures that there won't be any instruction
/// execution delays, and each instruction will take 6.25ns.
#[ram]
pub fn transmit_packet(data: &[u8], _tx_periph: &mut impl hal::gpio::OutputPin) {
    let data_ptr_range = data.as_ptr_range();

    // Enable performance counting, and set it up to count CPU cycles.
    unsafe {
        asm!(
            "csrw {mpcer}, (1<<0)", // Enable counting of CPU cycles.
            "csrw {mpcmr}, 0", // Disable the counter.
            "csrw {mpcmr}, 1", // Enable the counter.
            mpcer = const(MPCER),
            mpcmr = const(MPCMR),
        );
    }

    // Given the concerns listed above, the easiest approach to writing a transmission control loop
    // with deterministic timing is to write an unrolled loop that transmits an octet (8 bits) of
    // data at a time, with only a single jump at the end. That way we can most easily ensure that
    // the loop takes exactly the right amount of CPU cycles, by ensuring that the single branch
    // instruction is placed at a 4 byte-aligned address.

    // Note: the following loop performs Manchester encoding as part of the iteration, by taking
    // each bit to transmit and then XOR'ing it with 1, emitting that value as the first bit-half
    // and then emitting the original bit value as the second bit-half. This means we don't have to
    // encode the input data first, before then transmitting the encoded data, saving on CPU time
    // and avoiding the need for a 2nd buffer to hold the encoded data.
    let cycles_transmitting = unsafe {
        asm_with_perf_counter!(
            // Read the current CSR value.
            "csrr {csr_base_value}, {csr_cpu_gpio_out}",
            // Clear the bit corresponding to the CPU output signal we will use.
            "andi {csr_base_value}, {csr_base_value},~(1 << {cpu_signal_idx})",
            // Writing {csr_base_value} to the CSR will now preserve the state of all the other CPU
            // output signals that we aren't using in this function (just in case some other piece
            // of code is using the dedicated IO feature as well..)

            // Start by emitting only zeroes in the first iteration. We do this because the `bne`
            // instruction at the end of the first iteration will take an extra CPU cycles compared
            // to all subsequent iterations, and we want to ensure that the timing for each
            // transmitted data byte is perfect. By emitting zeroes in the first iteration we
            // effectively make it a no-op, meaning that the extra CPU cycle for the backward jump
            // doesn't matter. timing is perfect.
            "li {data_byte}, 0",
            "li {data_bit}, 0",
            // We make the first iteration emit zeroes by effectively disabling the XOR operation
            // that is otherwise used to perform the Manchester-encoding of the data.
            "li {xor_value}, 0",
            // Subtract the data pointer by one, since we'll run through one iteration without
            // actually transmitting any data, increment the data pointer, and only the start
            // transmitting data from the 2nd iteration onwards.
            "addi {data_ptr}, {data_ptr}, -1",
            // Jump into the first loop iteration, but skipping the first `lbu` instruction. That
            // way we a) won't read from a potentially invalid address (due to {data_ptr} having
            // been decremented just now), and b) we'll simply act as if the very first data byte is
            // zero.
            "j 2f",

            // Start of next byte processing logic. There should be exactly 7 CPU cycles up to and
            // including the `csrw` instruction, so that there's 8 cycles in total when accounting
            // for the `bne` instruction at the end of the previous iteration.
            //
            // Note that we know that this label's address will be 4 byte-aligned, since we know
            // that the start of this block is 4 byte-aligned (`asm_with_perf_counter` guarantees
            // that), and that the 6 `andi`/`li`/`addi`/`j`` instructions before this label are 16
            // bit instructions, and hence this is still a 4 byte-aligned address.
            "1:",
            // Load the next byte into the {data_byte} register.
            "lbu {data_byte}, 0({data_ptr})", // 1st and 2nd cycle
            "2:",
            // Process bit 1, starting by extracting the data bit from the LSB.
            //
            // Note that the `andi` instruction *must* follow the `lbu` instruction to ensure that
            // the `lbu` instruction consistently takes 2 cycles each time it is executed. See the
            // comment in [cycle_counting::_branch_back_usually_taken_w_lbu] for more info on why
            // this is important.
            "andi {data_bit}, {data_byte}, 1", // 3rd cycle
            // XOR with 1 to generate the first half of the Manchester-encoded bit symbol.
            "xor {tmp}, {data_bit}, {xor_value}", // 4th cycle
            // Shift the XOR'd data bit to the right CPU output signal index.
            "slli {tmp}, {tmp}, {cpu_signal_idx}", // 5th cycle
            // Apply it to the base CSR value to produce the target CSR value (writing to our target
            // bit but leaving the other bits unchanged from the base CSR value).
            "or {tmp}, {tmp}, {csr_base_value}", // 6th cycle
            // Emit the first half of the bit symbol. Note that when we execute this instruction the
            // very first time, the timing doesn't really matter because the first iteration of the
            // loop will emit only zeroes. After that, the timing of the 2nd iteration also doesn't
            // matter, since the actual data transmission still won't have started yet. After that,
            // this instruction needs to be executed on the 8th cycle since the last `csrw` at the
            // end of the previous iteration. It's 7 CPU cycles away from the "1:" label, which
            // means it's exactly 8 cycles away from the previous iteration's `csrw`, when we take
            // the `bne` instruction into account, once we're in the 3rd iteration of the loop
            // onwards.
            //
            // Once we hit this instruction for the first real data bit, we must spend exactly
            // `data.len() * 8 * 16 + 40` CPU cycles between this instruction and the second the
            // last instruction of this block (the last `nop` holding the TP_IDL signal high).
            "csrw {csr_cpu_gpio_out}, {tmp}", // 7th cycle
            // Now proceed to the second half of the Manchester-encoded bit symbol, by again
            // shifting the (non-XOR'd) data bit to the right CPU output signal index.
            "slli {tmp}, {data_bit}, {cpu_signal_idx}", // 1st cycle
            // Apply it to the CSR value to produce the target CSR value.
            "or {tmp}, {tmp}, {csr_base_value}", // 2nd cycle
            "nop", // 3rd cycle
            "nop", // 4th cycle
            "nop", // 5th cycle
            "nop", // 6th cycle
            "nop", // 7th cycle
            // Emit the second half of the bit symbol. This must happen in the 8th cycle since the
            // last `csrw`.
            "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle

            // A macro that shifts the data one to the right and writes the two bit halves
            // corresponding the LSB to the CPU output signal, taking exactly 8 CPU cycles for each
            // bit half.
            ".macro write_single_bit bit_idx",
            // Shift the data byte by one and extract the new data bit from the LSB.
            "srl {data_byte}, {data_byte}, 1", // 1st cycle
            "andi {data_bit}, {data_byte}, 1", // 2nd cycle
            // XOR with 1 to generate the first half of the bit symbol.
            "xor {tmp}, {data_bit}, {xor_value}", // 3rd cycle
            // Create the new CSR value.
            "slli {tmp}, {tmp}, {cpu_signal_idx}", // 4th cycle
            "or {tmp}, {tmp}, {csr_base_value}", // 5th cycle
            "nop", // 6th cycle
            "nop", // 7th cycle
            // Output the first half of the bit symbol.
            "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle
            // Now proceed to the second half of the bit symbol, by again shifting the (non-XOR'd)
            // data bit to the right CPU output signal index.
            "slli {tmp}, {data_bit}, {cpu_signal_idx}", // 1st cycle
            // Create the new CSR value.
            "or {tmp}, {tmp}, {csr_base_value}", // 2nd cycle
            "nop", // 3rd cycle
            "nop", // 4th cycle
            "nop", // 5th cycle
            "nop", // 6th cycle
            "nop", // 7th cycle
            // Output the second half of the bit symbol.
            "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle
            ".endm",

            // Process bits 2 through 7.
            "write_single_bit",
            "write_single_bit",
            "write_single_bit",
            "write_single_bit",
            "write_single_bit",
            "write_single_bit",

            // Process bit 8. Shift the data byte by one and extract the new data bit from the LSB.
            "srl {data_byte}, {data_byte}, 1", // 1st cycle
            "andi {data_bit}, {data_byte}, 1", // 2nd cycle
            // XOR with 1 to generate the first half of the bit symbol.
            "xor {tmp}, {data_bit}, {xor_value}", // 3rd cycle
            // Create the new CSR value.
            "slli {tmp}, {tmp}, {cpu_signal_idx}", // 4th cycle
            "or {tmp}, {tmp}, {csr_base_value}", // 5th cycle
            "nop", // 6th cycle
            "nop", // 7th cycle
            // Emit the first half of the bit symbol
            "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle
            // Now proceed to the second half of the bit symbol, by again shifting the (non-XOR'd)
            // data bit to the right CPU output signal index.
            "slli {tmp}, {data_bit}, {cpu_signal_idx}", // 1st cycle
            // Create the new CSR value.
            "or {tmp}, {tmp}, {csr_base_value}", // 2nd cycle
            // We've now set everything up to emit the last half of the last bit symbol, but we
            // still have 5 extra CPU cycles before we should emit it. We'll use those remaining CPU
            // cycles to prep for the next iteration of the loop.
            "nop", // 3rd cycle
            "nop", // 4th cycle
            // 5th cycle, this is equivalent to `nop`, but is encoded as a 32-bit instruction. We
            // use it instead of `nop` to ensure that the `bne` instruction below is at a 4-byte
            // aligned address.
            "andi zero, zero, 0",
            // Ensure that all subsequent iterations actually perform the necessary XOR operation to
            // Manchester-encode the data. The very first iteration disables the XOR operation (by
            // XORing with zero), since the first iteration is intended to emit a steady stream of
            // zeros.
            "addi {xor_value}, zero, 1", // 6th cycle
            // Advance the data pointer by one in anticipation of the branch check below.
            "addi {data_ptr}, {data_ptr}, 1", // 7th cycle
            // Output the bit via the CSR.
            "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle

            // Loop back to process the next data byte, if there's any more. The first time we hit
            // this instruction it'll take 2 cycles, then it'll take 1 cycle for taken branches in
            // the future, and 3 cycles for the final non-taken branch. Note that the non-taken
            // branch takes 3 cycles because we've ensured, above, that this instruction falls on a
            // 4 byte-aligned address. Also note that the very first iteration of this loop only
            // emits zeroes, and not any real data, hence we don't mind that the first branch takes
            // an extra CPU cycle.
            "bne {data_ptr}, {data_end_ptr}, 1b",
            // If we reach here then we've reached the end of the data transmission and we need to
            // proceed to the start-of-idle signal. These `nop`` instructions ensure we hold the
            // line high or low for long a total of at least 8 cycles on behalf of the last bit sent
            // before we do so.
            "nop", // 4th cycle
            "nop", // 5th cycle
            "nop", // 6th cycle
            "nop", // 7th cycle

            // Emit a TP_IDL signal by setting the pin high and keeping it high 250ns (40 cycles),
            // then setting it low.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // 8th cycle
            // 39 cycles
            "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
            "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
            "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
            "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
            // Set the output low.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // 40th cycle
            // The specifier of the CSR we have to write to to perform direct IO.
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_signal_idx = const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
            // The address of the byte array we're processing. Must be `inout(...)` since we modify
            // the register during execution.
            data_ptr = inout(reg) data_ptr_range.start => _,
            // The end address (exclusive, i.e. the address one past the last valid address).
            data_end_ptr = in(reg) data_ptr_range.end,
            // Will hold the byte we're processing.
            data_byte = out(reg) _,
            // Will hold the data bit being processed.
            data_bit = out(reg) _,
            // Will hold the complement of the data bit being processed.
            xor_value = out(reg) _,
            // Will hold the original CSR value, to which we'll apply the specific GPIO's bit value.
            csr_base_value = out(reg) _,
            // A general scratch register.
            tmp = out(reg) _
        )
    };
    // Cycles spent at the start for `csrr`, `andi`, `li`, and `addi` instructions.
    let mut cycles_expected = 6;
    // Cycles spent in the very first iteration from the `j` instruction and up to but not including
    // the first `csrw` (the `j` instruction seems to take 2 CPU cycles here).
    cycles_expected += 6;
    // Exactly 8 * 16 cycles, plus one extra, spent emitting zeroes.
    cycles_expected += 8 * 16 + 1;
    // Exactly 8 * 16 CPU cycles spent emitting the real data.
    cycles_expected += data.len() as u32 * 8 * 16;
    // Exactly 40 cycles spent emitting the TP_IDL signal.
    cycles_expected += 40;
    // Ehe final cycle spent resetting the signal level back to 0 after TP_IDL.
    cycles_expected += 1;
    if cycles_transmitting != cycles_expected {
        warn!(
            "transmit_packet loop took {cycles_transmitting} cycles instead of the \
                {cycles_expected} cycles we expected."
        );
    }
}
