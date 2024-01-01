//! Provides low-level functions that implement transmission functionality in assembly code, using
//! the dedicated IO feature of the ESP32-C6 chip.

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use core::arch::asm;
use hal::prelude::*;
use log::{trace, warn};
use niccle::eth_mac;
use niccle_proc_macros::asm_with_perf_counter;

/// For now the implementation is hardcoded to always use CPU output signal 0 when using dedicated
/// IO on the TX pin. In the future we could try to make this more flexible, e.g. using an argument,
/// or perhaps even a const generic parameter with a const expression bound.
pub const TX_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal = {
    #[cfg(feature = "esp32c3")]
    let signal = hal::gpio::OutputSignal::CPU_GPIO_0;
    #[cfg(feature = "esp32c6")]
    let signal = hal::gpio::OutputSignal::CPU_GPIO_OUT0;
    signal
};
/// The dedicated IO output signal index to use (from 0 to 7, where 0 corresponds to CPU_GPIO_OUT0).
/// Based on whatever [TX_CPU_OUTPUT_SIGNAL] is set to.
const TX_CPU_OUTPUT_SIGNAL_CSR_IDX: isize = {
    #[cfg(feature = "esp32c3")]
    let signal0 = hal::gpio::OutputSignal::CPU_GPIO_0;
    #[cfg(feature = "esp32c6")]
    let signal0 = hal::gpio::OutputSignal::CPU_GPIO_OUT0;
    TX_CPU_OUTPUT_SIGNAL as isize - signal0 as isize
};

/// For now the implementation is hardcoded to always use CPU input signal 7 when using dedicated IO
/// on the RX pin. In the future we could try to make this more flexible, e.g. using an argument, or
/// perhaps even a const generic parameter with a const expression bound.
///
/// WARNING: The current implementation requires that no other dedicated IO input signals are used
/// while this library is active (it relies on the fact that, whenever we read the dedicated IO
/// input value, all bits except for the 8th bit are zero).
pub const RX_CPU_INPUT_SIGNAL: hal::gpio::InputSignal = {
    #[cfg(feature = "esp32c3")]
    let signal = hal::gpio::InputSignal::CPU_GPIO_7;
    #[cfg(feature = "esp32c6")]
    let signal = hal::gpio::InputSignal::CPU_GPIO_IN7;
    signal
};

/// Users of the [crate::eth_phy] library must configure a GPIO to be connected to this dedicated IO
/// CPU output signal. The RX receive loop will emit debug signals to this CPU output signals.
///
/// For now the implementation is hardcoded to always use CPU output signal 1 for the debug signal
/// we emit in the RX loop. In the future we could try to make this more flexible, e.g. using an
/// argument, or perhaps even a const generic parameter with a const expression bound.
pub const RX_DEBUG_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal = {
    #[cfg(feature = "esp32c3")]
    let signal = hal::gpio::OutputSignal::CPU_GPIO_1;
    #[cfg(feature = "esp32c6")]
    let signal = hal::gpio::OutputSignal::CPU_GPIO_OUT1;
    signal
};
/// The dedicated IO output signal index to use for the debug signal we emit in the RX loop(from 0
/// to 7, where 0 corresponds to CPU_GPIO_OUT0). Based on whatever [RX_DEBUG_CPU_OUTPUT_SIGNAL] is
/// set to.
const RX_DEBUG_CPU_OUTPUT_SIGNAL_CSR_IDX: isize = {
    #[cfg(feature = "esp32c3")]
    let signal0 = hal::gpio::OutputSignal::CPU_GPIO_0;
    #[cfg(feature = "esp32c6")]
    let signal0 = hal::gpio::OutputSignal::CPU_GPIO_OUT0;
    RX_DEBUG_CPU_OUTPUT_SIGNAL as isize - signal0 as isize
};

/// The address of the Machine Performance Counter Event Register.
const MPCER: usize = 0x7E0;
/// The address of the Machine Performance Counter Mode Register.
const MPCMR: usize = 0x7E1;

/// This CSR isn't defined by the esp-rs/esp-hal framework yet (nor by the espressif/svd files), so
/// we hardcode their addresses. See the "1.14 Dedicated IO" chapter of the technical reference
/// manual.
const CSR_CPU_GPIO_OUT: u32 = 0x805;
/// This CSR isn't defined by the esp-rs/esp-hal framework yet (nor by the espressif/svd files), so
/// we hardcode their addresses. See the "1.14 Dedicated IO" chapter of the technical reference
/// manual.
const CSR_CPU_GPIO_IN: u32 = 0x804;

/// The minimum amount of data we expect to have written to our receive buffer during an incoming
/// transmission of an actual data packet. This is the minimum frame size plus one byte for the SFD,
/// since we must have observed the SFD in order to be to align the frame data correctly.
const RX_MIN_DATA_SIZE_BYTES: usize = eth_mac::MIN_FRAME_SIZE + 1;
/// The maximum amount of data we expect to write to our receive buffer during an incoming
/// transmission. This is the max packet size plus one byte, to account for parts of the TP_IDL that
/// may get written to the buffer before we end the receive loop.
pub const RX_BUFFER_SIZE_BYTES: usize = eth_mac::MAX_PACKET_SIZE + 1;

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

/// Transmits the given Ethernet packet over the transmission line.
///
/// See [super::eth_phy::PhyTx::transmit_packet](super::eth_phy::Phy) for more info.
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
/// <https://ctrlsrc.io/posts/2023/counting-cpu-cycles-on-esp32c3-esp32c6/>.
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
/// See `cycle_counting::branch_back_usually_taken_aligned` and
/// `cycle_counting::_branch_back_usually_taken_w_lbu` for the benchmarks showing this behavior for
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
            // The specifier of the CSR we have to write to to perform dedicated IO-based output.
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

/// Emits a signal to dedicated IO CPU signal [RX_DEBUG_CPU_OUTPUT_SIGNAL_CSR_IDX], to indicate that
/// an RX interrupt has occurred.
///
/// This function purposely can be called at any time, without any synchronization or access to the
/// RX debug pin, since we want its invocation to be as fast/cheap as possible in order to
/// accurately reflect the timing of interrupt invocation, relative to the timing of the incoming
/// packet. E.g. both the incoming signal and the debug signal can be inspected using an
/// oscilloscope to figure out the interrupt routine latency.
#[ram]
pub fn emit_receive_interrupt_debug_signal() {
    unsafe {
        asm!(
            // Clear the line, in case it was left high for some reason.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            // Next, pulse the line twice, leaving it low at the end.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            debug_cpu_signal_idx = const(RX_DEBUG_CPU_OUTPUT_SIGNAL_CSR_IDX)
        );
    }
}

/// Represents all of the types of transmissions [receive_packet] may encounter.
#[derive(Debug)]
pub enum ReceivedTransmission<'a> {
    /// The transmission was most likely a link test pulse, and no data was received.
    LinkTestPulse,
    /// The transmission was probably not a link test pulse, but also resulted in too little data
    /// being received to make up a full Ethernet packet.
    TruncatedTransmission,
    //// The transmission was most likely a data packet (but it has not been validated yet).
    Packet(&'a mut [u8]),
}

/// Attempts to receive and Manchester-decode an Ethernet packet from the transmission line. This
/// function should be invoked when an incoming transmission is suspected to be ongoing (e.g. after
/// an interrupt was triggered due to a detected signal edge on the transmission line).
///
/// If a possible packet was received, then the return value will be [ReceivedTransmission::Packet],
/// and the returned slice will in that case contain the packet data, incl. part of the preamble,
/// the SFD, the frame data, and up to two bits of TP_IDL. Each byte in the slice is expected to
/// contain one or more bits of packet data. The MAC layer is responsible for processing the packet
/// by finding the SFD, stripping the preamble, and validating the FCS.
///
/// # A note on writing precisely timed assembly code
///
/// When receiving data, each half of a Manchester-encoded bit symbol is transmitted for 50ns, for a
/// total of 100ns per bit symbol. Therefore, given a clock speed of 160MHz, we have exactly eight
/// CPU cycles during which we can observe each half bit symbol. See the docs for [transmit_packet]
/// for more info on why we must write the receive loop in assembly code.
///
/// For this particular function, the following insights gained from the `cycle_counting` example
/// binary are most relevant (focusing on the behavior on ESP32-C6 for now):
///
/// * `csrr`, `andi`, `or`, `srli` and other simple instructions not otherwise called out below take
///   1 CPU cycle.
/// * `sb` instructions take 1 CPU cycle as long as they fall on 4 byte-aligned addresses.
/// * Backward-pointing `bne` instructions such as the very last one in this loop take 3 CPU cycles
///   when not taken (at the end of the iteration) if the instruction falls on a 4 byte-aligned
///   address, and 4 CPU cycles when not taken and unaligned. They take 2 CPU cycles when taken for
///   the first time (in the first iteration), and 1 CPU cycle when taken after having been taken
///   before already (in the second to second-to-last iterations), regardless of alignment. See
///   `cycle_counting::branch_back_usually_taken_aligned` and
///   `cycle_counting::_branch_back_usually_taken_w_sb` for the benchmarks showing this behavior for
///   the ESP32-C6 CPU.
/// * Forward-pointing `beq`, `bne`, `bge` and `ble` instructions in this loop should take 2 CPU
///   cycles when not taken and when encountered for the first time, and 1 CPU cycle when not taken
///   and encountered subsequent times after that. **However**, contrary to the what the
///   `cycle_counting` example binary shows, I've found that in this particular case these
///   instructions seem to take only 1 CPU when not taken, regardless of whether it's the first time
///   the instruction is encountered or not.
///
/// Note: the use of #[ram] is also important, as it it ensures that there won't be any instruction
/// execution delays, and each instruction will take 6.25ns.
///
/// # Implementation details
///
/// This function works by:
///
///   1. Sampling the line until a non-zero signal is detected. If a zero signal is continuously
///      detected for >=100ns, then the function assumes that the interrupt may have been triggered
///      by an incoming link test pulse which immediately left the line idle, and in this case
///      [ReceivedTransmission::LinkTestPulse] is returned.
///
///   2. If a non-zero signal is detected, then we subsequently wait in a tight loop until the
///      signal returns to zero again. This point in time is assumed to correspond to a falling edge
///      in the preamble signal, and the code will subsequently start sampling the signal every
///      50ns, starting approximately 75ns after the falling edge was detected. Since the falling
///      edge represents the start of the second half of a bit symbol in the preamble, sampling 75ns
///      from that point means we'll start sampling in the middle of the first half of the next bit
///      symbol.
///
///   3. From that point on we sample one bit of data every 50ns, thereby sampling twice per bit
///      symbol, once per bit symbol half. As long as the sender doesn't introduce too much jitter
///      in the data signal, the sampling points should always fall more or less in the middle of
///      each bit symbol half. Hence, as long as this function is invoked before the preamble signal
///      has completely arrived at our receiver, this function should be able to successfully align
///      the sampling frequency and read the incoming transmission.
///
///      This approach is somewhat similar to what a phase-locked loop (PLL) does, except it's
///      implemented in software: it tries to find the clock edge in the preamble signal, and then
///      tries to align its sampling frequency as close as possible to that clock edge, after which
///      point the sampling occurs at a consistent frequency.
///
///   4. We keep sampling data until we notice three or more consecutive 0 or 1 samples. Since the
///      Manchester encoding ensures a signal transition at most every 100ns, we should never sample
///      the same value for more 150ns in a row. Hence, if we do see such a sequence it indicates
///      that the TP_IDL signal was encountered (signalling the end of the packet) or that the
///      transmission was interrupted.
///
///   5. Once the receive loop finishes, we check how much data was written to the buffer. If enough
///      data was written to include both the SFD octet and the data frame itself, then we consider
///      it likely that the buffer now contains an Ethernet packet somewhere within it, and we
///      return [ReceivedTransmission::Packet]. A MAC layer is subsequently expected to process the
///      buffer.
///
///   6. If not enough data was received we consider the transmission to have been truncated, and
///      return [ReceivedTransmission::TruncatedTransmission].
///
/// The function also performs Manchester-decoding on the fly, thereby avoiding the need for an
/// auxiliary buffer and an additional decoding step. It does this by considering each sampled
/// second bit half signal to be the decoded data bit. Because the signal alignment logic above
/// ensures that we always start sampling in the middle of the **first** half of some bit symbol, we
/// know that every second sample will correspond to the second bit half, and hence the data.
///
/// ## Illustration of the clock alignment mechanism.
///
/// The schematic below illustrates the clock alignment mechanism described above. A falling edge is
/// detected in the preamble signal, which then allows us to align our sampling period to the data
/// signal.
///
/// ```text
///              .                 .                 .                 .                 .
///              .< --- 100ns --- >.<---- 100ns ---->.<---- 100ns ---->.<---- 100ns ---->.
///              .< 50ns > < 50ns >.< 50ns > < 50ns >.< 50ns > < 50ns >.< 50ns > < 50ns >.
///              .                 .                 .                 .                 .
///              .  BIT SYMBOL #1  .  BIT SYMBOL #2  .  BIT SYMBOL #3  .  BIT SYMBOL #4  .
///              .                 .                 .                 .                 .
///              <--------|        .        |-----------------|        |--------|        |------->
///     SIGNAL   .        |        .        |        .        |        |        |        |
///              .        |-----------------|        .        |--------|        |--------|
///              .                 .                 .                 .                 .
///     SAMPLES  .        ^        .   ^        ^    .   ^        ^    .   ^        ^    .    ^
///                       |            |        |
///                       |            |        |
///                       |            |        |
///     Detect falling edge.           |        Sample every 50ns after that, always
///                                    |        in the middle of a bit symbol half.
///                                    |
///                                    Start sampling 75ns after edge, in the
///                                    middle of the next bit symbol's first half.
/// ```
///
/// Note that `_rx_periph` and `_rx_debug_periph` are technically unused, but by requiring a
/// reference to them we ensure mutually exclusive use of the pin's corresponding
/// `RX_CPU_INPUT_SIGNAL` and `RX_DEBUG_CPU_OUTPUT_SIGNAL` signals.
#[ram]
pub fn receive_packet<'a>(
    buffer: &'a mut [u8; RX_BUFFER_SIZE_BYTES],
    _rx_periph: &mut impl hal::gpio::InputPin,
    _rx_debug_periph: &mut impl hal::gpio::OutputPin,
) -> ReceivedTransmission<'a> {
    const RESULT_CODE_END_OF_BUFFER: u32 = 1;
    const RESULT_CODE_IDLE_LINE: u32 = 2;
    const RESULT_CODE_TP_IDL: u32 = 3;
    const RESULT_CODE_TRUNCATED: u32 = 4;

    let buffer_ptr_range = buffer.as_ptr_range();
    // Will hold the end address of the data that was received by the receive loop (it will point 1
    // byte past the address of the byte we last wrote to).
    let buffer_ptr: *const u8;
    // Will hold a code that indicates the reason why the loop finished. This can be useful for
    // debugging its behavior.
    let mut result_code: u32;
    // Will hold the final decoded byte (which may only be partially populated, if the transmission
    // was not perfectly aligned on byte boundaries).
    let mut decoded_byte: u32;

    // Perform the receive loop.
    //
    // Note that in addition to using the dedicated IO feature to sample the incoming data signal,
    // the loop will also use dedicated IO to output a debug signal on an auxiliary GPIO CPU signal.
    // This debug signal can be used to validate the timing of the receive loop.
    //
    // However, note that when using dedicated IO to sample input signals there is a 2 CPU cycle
    // latency for sensing input signals. Hence, the receive loop (and the debug signals it emits)
    // will not align exactly with the input signal, when both the input signal and the debug signal
    // are observed with oscilloscope at the same time. You must imagine that the debug signal is
    // shifted to the left by at least 3 CPU cycles (2 to account for the sensing latency, and 1
    // additional one to account for the fact we output the debug signal *after* we've sampled the
    // input signal).
    let cycles_receiving = unsafe {
        asm_with_perf_counter!(
            // A nop-equivalent instruction that is guaranteed to be encoded as a 4 byte long
            // instruction (vs. the 2 byte `nop` instruction).
            ".macro four_byte_nop",
            "andi x0, x0, 0",
            ".endm",

            // Emit a debug signal on the RX debug GPIO, to indicate we're about to start the
            // receive loop. First, clear the line in case it was held high for some reason.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            // Then pulse it high-low briefly.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            // Lastly, leave the line HIGH. We'll pull it low again once we've detected the falling
            // clock edge further below.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",

            // Sample the line up to 9 times, to see if we can detect a high value. If we cannot,
            // then the line has been low for at least >=9*2 CPU cycles, which is >100ns, which
            // means that we're not in the middle of a preamble (which can have at most 100ns of the
            // line being low), and hence this is probably not a valid transmission (and instead we
            // may have just been interrupted by an LTP and then started reading the idle, low
            // line).
            //
            // If we didn't perform this check, then in the case of LTP-caused interrupt the code at
            // label 0 would loop until the *next* LTP is received, blocking the ISR for ~16ms. We'd
            // effectively spend 50% of CPU time blocked here, which would cause all kinds of havoc.
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "bne {sampled_csr_value}, zero, 10f",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            "beq {sampled_csr_value}, zero, 50f",

            // We've seen some non-zero data on the line. Now try to align our sampling frequency as
            // clearly as we can to the incoming clock signal, by reading bits until we see a zero
            // bit again.
            //
            // Note: The approach we take here means that we could loop forever if the host on the
            // other side of the line pulls the line high and never brings it low again. We could
            // introduce a counter/deadline and break out of the loop, but that'd mean more cycles
            // spent tracking the counter, which we then cannot use to align our timing to the
            // preamble signal. So instead we assume that the other host's implementation follows
            // the spec, and never holds the line high for long periods. This is likely worth re
            ".align 4",
            "10:",
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0",
            // Wait until the read bit is zero. This will take 3 cycles for the non-taken branch
            // (because we've ensured the instruction falls on a 4 byte-aligned address), and 1
            // cycle for the taken branch (2 cycles the first time it's taken).
            //
            // This is a 2 byte instruction, so it will leave the next instruction not 4
            // byte-aligned.
            "bne {sampled_csr_value}, zero, 10b",

            // We've now detected a transition from one to zero, with the read that detected it
            // happening 4 cycles ago. This means that the actual transition may have happened
            // anywhere from 4-6 cycles ago (since we don't know what the state of the GPIO was in
            // the 1 or 2 cycles spent executing the `bne ..., 20b` instruction before our last GPIO
            // read instruction).
            //
            // The ideal time to start sampling at is hence 6 or 7 cycles from now, which would mean
            // we sample between 10 to 13 cycles or between 62.5ns to 81.25ns from the estimated
            // transition time (depending on when the bit actually flipped vs. when we detected it).
            //
            // However, we choose to sample only 5 cycles from now, because after the first
            // iteration of our receive loop the final `bne` instruction will take 2 CPU cycles
            // rather than 1 CPU cycle for subsequent iterations, and hence our sampling period will
            // shift forward by 1 CPU cycle after the first iteration. This choice seems to work
            // well, and when inspecting the debug signal with an oscilloscope and accounting for
            // the 3 CPU cycle sampling delay, we see sampling points falling neatly in the middle
            // of each bit symbol half.
            //
            // In practice, waiting anywhere between 4 and 8 cycles seems to work equally well, at
            // least for the somewhat short test packets I tested with.
            //
            // We use one of the 5 idle cycles to pull the debug line low, to signal that we've
            // detected a clock edge. From this point on, we'll pull the line high whenever we've
            // just sampled the first half of a bit symbol, and pull the line low whenever we've
            // just sampled the second half of a bit symbol.
            //
            // This is a 4 byte instruction.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            "four_byte_nop",
            // We must ensure the remaining nop instructions end a 4 byte-aligned address. Since the
            // `bne` instruction above placed us on a non 4 byte-aligned address, the uneven number
            // of 2 byte `nop` instructions below will place the next code block at a 4 byte-aligned
            // address.
            "nop",
            "nop",
            "nop",

            // The line is now expected to be low, representing the 0 in the next 0-to-1 manchester
            // symbol in the preamble (we've already passed the first 0-to-1 and the 2nd 1-to-0
            // symbols). Note that either way, our first sample point will always fall in the middle
            // of the first half of the next bit symbol.

            // A macro that reads a single data bit. It performs two samples, one for the first half
            // of the bit symbol, and one for the second half of the bit symbol. It uses the value
            // of the second half as the Manchester-decoded data bit.
            //
            // The macro also inspects the sequence of samples to detect the TP_IDL signal, as well
            // as the case where the line become idle/zero unexpectedly.
            //
            // NOTE: This code assumes that the GPIO we're reading from is configured at dedicated
            // IO CPU signal index 7, and that no other dedicated IO CPU signals are configured as
            // inputs. This removes the need for extra CPU instructions, which is critical to fit
            // everything within the 16 CPU cycles we have per bit symbol.
            ".macro read_single_bit prev_decoded_byte_reg, shift_amt, aligned_nop",
            // ----- Sample the first bit half.
            //
            // Sample the signal (we assume the high bit, bit 7, holds the sampled bit).
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0", // 1st cycle
            // Pull the debug line high to indicate we've just sampled the first half of a bit
            // symbol. Under normal conditions this rising edge should always be 8 CPU cycles (50ns)
            // away from the previous falling edge, indicating sampling is happening at a consistent
            // 50ns frequency.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}", // 2nd cycle
            // Prepend the bit to our register holding the stream of the 8 previously sampled bits.
            "or {sampled_bits}, {sampled_bits}, {sampled_csr_value}", // 3rd cycle
            // Shift the stream of sampled bits one to the right to make room for the next sample.
            "srli {sampled_bits}, {sampled_bits}, 1", // 4th cycle.
            // Idle until it's time for the next sample.
            "nop", // 5th cycle
            "nop", // 6th cycle
            "nop", // 7th cycle
            // 8th cycle. This nop needs to a standard two-byte-long `nop` instruction for bits 6
            // through 1, but has to be a four-byte-long nop-equivalent instruction for bit 7, since
            // one of the `or` instructions below takes up 2 bytes for bits 6 through 1 but 4 bytes
            // for bit 7 (see below). This way we ensure that each invocation of the
            // `read_single_bit` macro produces 4-byte-aligned code.
            "\\aligned_nop",

            // ----- Sample the second bit half. Its value is the decoded output bit.
            //
            // Sample the signal.
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0", // 9th cycle
            // Pull the debug line low to indicate we've just sampled the second half of a bit
            // symbol. Under normal conditions this falling edge should always be 8 CPU cycles
            // (50ns) away from the previous rising edge, indicating sampling is happening at a
            // consistent 50ns frequency.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}", // 10th cycle
            // Prepend the bit to our register holding the stream of the 8 previously sampled bits.
            "or {sampled_bits}, {sampled_bits}, {sampled_csr_value}", // 11th cycle
            // Shift the sampled bit (in bit index 7) to the right by the appropriate amount, and
            // add to the result byte. E.g. the first bit in an octet is shifted 7 to the right into
            // bit index 0, since we want to return data in LSB-received-first order.
            "srli {sampled_csr_value}, {sampled_csr_value}, \\shift_amt", // 12th cycle
            // Place the sampled bit into the decoded byte (at the appropriate bit index). When
            // reading the first bit, this also resets all other {decoded_byte} bits to 0.
            //
            // NOTE: This instruction takes up 2 bytes when \\prev_decoded_byte is set to
            // {decoded_byte}, and 4 bytes if not. Hence, the first invocation of this macro for the
            // 7th bit will require different alignment/padding than the other invocations. Hence
            // the use of the `aligned_nop` macro argument.
            "or {decoded_byte}, \\prev_decoded_byte_reg, {sampled_csr_value}", // 13th cycle
            // Now check for the TP_IDL signal, or for a consistent idle/zero line.
            //
            // If the {sampled_bits} value is greater than or equal to the TP_IDL threshold, then
            // we've read 3 ones in a row. This is either an invalid sequence, or the end of the
            // frame (the TP_IDL signal). In either case we should bail.
            "bge {sampled_bits}, {tp_idl_threshold}, 60f", // 14th cycle
            // If {sampled_bits} is ever less than the value in the specific zero_threshold_reg,
            // then we've read N zeros in a row (N should be >= 3). This is an invalid sequence, and
            // probably means we got interrupted during the receipt, or we hit some kind of
            // truncated transmission. Either way we should bail.
            "ble {sampled_bits}, {zero_data_threshold}, 70f", // 15th cycle
            // If we didn't detect either a TP_IDL or all-zero signal, then shift the stream of
            // sampled bits one to the right to make room for the next sample.
            "srli {sampled_bits}, {sampled_bits}, 1", // 16th cycle.
            ".endm",

            // Sample bits 1 through 7, placing the first sampled bit in the LSB of {decoded_byte},
            // and the 7th sampled bit in the 7th bit of {decoded_byte}.
            //
            // Note that this label is guaranteed to be a 4 byte-aligned address because the
            // preceding instructions ensure this, and note that each read_single_bit macro
            // invocation will maintain this 4 byte-alignment.
            "20:",
            // When we read the first bit we must store the sampled bit in a a fresh,
            // zero-initialized register. Also note that the first invocation of read_single_bit
            // needs to use a different 4 byte long nop instruction to ensure it maintains the 4
            // byte instruction alignment.
            "read_single_bit x0, 7, four_byte_nop",
            // Subsequent sampled bits should be added to the {decoded_byte} register, and should
            // use standard 2 byte nop long instructions to maintain 4 byte instruction alignment.
            "read_single_bit {decoded_byte}, 6, nop",
            "read_single_bit {decoded_byte}, 5, nop",
            "read_single_bit {decoded_byte}, 4, nop",
            "read_single_bit {decoded_byte}, 3, nop",
            "read_single_bit {decoded_byte}, 2, nop",
            "read_single_bit {decoded_byte}, 1, nop",

            // Read the 8th bit and store the fully-read byte in the memory array.
            //
            //----- Sample the first bit half.
            //
            // Sample the signal. Note that this instruction will be 4 byte-aligned.
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0", // 1st cycle
            // Pull the debug line low to indicate we've just sampled the first half of a bit
            // symbol.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}", // 2nd cycle
            // Prepend the bit to our register holding the stream of the 8 previously sampled bits.
            "or {sampled_bits}, {sampled_bits}, {sampled_csr_value}", // 3rd cycle
            // Shift the stream of sampled bits one to the right to make room for the next sample.
            "srli {sampled_bits}, {sampled_bits}, 1", // 4th cycle
            // Idle until it's time for the next sample.
            "nop", // 5th cycle
            "nop", // 6th cycle
            "nop", // 7th cycle
            "four_byte_nop", // 8th cycle
            // ----- Sample the second bit half, store the decoded byte, and loop back.
            //
            // Sample the signal (we assume the high bit, bit 7, holds the sampled bit).
            "csrrsi {sampled_csr_value}, {csr_cpu_gpio_in}, 0", // 9th cycle
            // Pull the debug line low to indicate we've just sampled the second half of a bit
            // symbol.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}", // 10th cycle
            // Place the sampled bit into the decoded byte at bit 7 (i.e. the MSB).
            "or {decoded_byte}, {decoded_byte}, {sampled_csr_value}", // 11th cycle
            // Store the decoded byte into the output array.
            //
            // NOTE: this instruction MUST fall on a 4 byte-aligned address to guarantee that it
            // takes 1 CPU cycle.
            "sb {decoded_byte}, 0({buffer_ptr})", // 12th cycle
            // Increment the output address.
            "addi {buffer_ptr}, {buffer_ptr}, 1", // 13th cycle
            // Now also prepend the bit to our register holding the stream of the 8 previously
            // sampled bits, so that in the next iteration the stream can be checked for a TP_IDL or
            // all-zeroes signal. We don't perform that check here like we do for bits 1 through 7,
            // since we don't enough spare CPU cycles to do that, and since we can afford skipping
            // the check for up to two sampled bits in the sequence anyway.
            "or {sampled_bits}, {sampled_bits}, {sampled_csr_value}", // 14th cycle
            // Shift the stream of sampled bits one to the right to make room for the next sample.
            "srli {sampled_bits}, {sampled_bits}, 1", // 15th cycle
            // Loop back to receive the next data byte, if there's more room for it in the buffer.
            // The first time we take this branch it'll take 2 cycles, then it'll take 1 cycle in
            // the future.
            //
            // Because the first branch takes 2 CPU cycles it means that after the first iteration
            // the sampling period will shift by one CPU cycle. Luckily this is fine, and we'll
            // still end up sampling approximately in the middle of each bit symbol half despite
            // this (see note above).
            "bne {buffer_ptr}, {buffer_end_ptr}, 20b", // 16th cycle

            // If we make it here we must have run out of buffer space.
            "li {result_code}, {result_code_end_of_buffer}",
            // We jump to the very end of the loop, since if we reached here we will just have
            // written a byte to the output buffer and increment the pointer, and we shouldn't do
            // that again.
            "j 101f",

            // This label corresponds to having detected a consistently zero signal at the start of
            // the loop, most likely an LTP signal.
            "50: li {result_code}, {result_code_idle_line}",
            // Note that we skip straight to the end, without writing to the output buffer, since no
            // actual data will have been received.
            "j 101f",

            // This label corresponds to the TP_IDL signal being detected.
            "60: li {result_code}, {result_code_tp_idl}",
            "j 100f",

            // This label corresponds to an all-zero signal being detected (a truncated signal?).
            "70: li {result_code}, {result_code_truncated}",
            "j 100f",

            // Common bail-early code path, where the last decoded byte (which will only be
            // partially populated), should still be written to the output buffer before returning.
            "100:",
            // Store the last byte we read.
            "sb {decoded_byte}, 0({buffer_ptr})",
            // Increment the output data pointer one last time.
            "addi {buffer_ptr}, {buffer_ptr}, 1",

            // Common end state.
            "101:",
            // Pull the debug line low, then pulse it briefly to indicate the end of the receive
            // loop has been reached.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{debug_cpu_signal_idx}",
            // The specifier of the CSR we have to read from to perform dedicated IO-based input
            // sensing.
            //
            // Note that the code assumes/hardcodes that the input signal is configured to be read
            // into CPU input signal index 7.
            csr_cpu_gpio_in = const(CSR_CPU_GPIO_IN),
            // The specifier of the CSR we have to write to to perform dedicated IO-based output.
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            // The CPU output signal index of the debug line, used to emit debug signals.
            debug_cpu_signal_idx = const(RX_DEBUG_CPU_OUTPUT_SIGNAL_CSR_IDX),
            // Various result codes indicating the reason for terminating the loop.
            result_code_end_of_buffer = const(RESULT_CODE_END_OF_BUFFER),
            result_code_idle_line = const(RESULT_CODE_IDLE_LINE),
            result_code_truncated = const(RESULT_CODE_TRUNCATED),
            result_code_tp_idl = const(RESULT_CODE_TP_IDL),
            // A register that we'll use to read the input signal into.
            sampled_csr_value = out(reg) _,
            // A register that we'll use to place decoded bits into.
            decoded_byte = inout(reg) 0b0 => decoded_byte,
            // This register will hold the pointer to write the next byte to, and at the end of the
            // loop will hold the end address of the data range that was written out.
            buffer_ptr = inout(reg) buffer_ptr_range.start => buffer_ptr,
            // This register holds the pointer indicating the end of the buffer space, past which we
            // cannot write.
            buffer_end_ptr = in(reg) buffer_ptr_range.end,
            // We initialize the {sampled_bits} register with a pattern of alternating ones and
            // zeroes, to ensure that when we check the TP_IDL threshold and the zero data threshold
            // when reading bits further below, that neither of those thresholds will trigger during
            // the very first iteration.
            sampled_bits = inout(reg) 0b10101010 => _,
            // If at any point we've read 3 ones in a row, then either we've hit the TP_IDL (end of
            // frame) sequence, or we've read some invalid data.
            tp_idl_threshold = in(reg) 0b1110_0000,
            // If at any point we've read 3 zeros in a row (in which case bits 6 through 8 would be
            // zero, and hence the value would be less than or equal to the value below), then we
            // must've hit invalid data.
            zero_data_threshold = in(reg) 0b00011111,
            // This register is used to describe the reason why the loop finished.
            result_code = inout(reg) 0 => result_code
        )
    };

    // Calculate the amount of data we read by doing pointer math.
    let result_size = unsafe { buffer_ptr.offset_from(buffer_ptr_range.start) } as usize;

    // Check that we didn't write beyond our buffer. This should never happen, but let's be
    // conservative and try to detect bugs early if they ever do happen.
    assert!(
        result_size <= buffer.len(),
        "Receive loop wrote past the end of buffer: \
            result_code: {result_code}, result_size: {result_size} vs. buffer size: {} ",
        buffer.len()
    );

    // A valid Ethernet frame has a minimum length of 64 bytes. Check that the result is at least
    // large enough to contain that amount of data plus an SFD byte. If we received anything less
    // then that we probably didn't receive a valid frame frame.
    if result_size < RX_MIN_DATA_SIZE_BYTES {
        // If we didn't write any data to the buffer, then that means that we only saw zeros on the
        // line. That likely means that the interrupt for which we invoked this function was caused
        // by an LTP, and that by the time we actually got to executing the receive loop, the line
        // was already idle again (low value).
        if result_size == 0 {
            // We expect LTPs to be detected at the very start of the loop. Log a warning if that
            // wasn't the case.
            if result_code != RESULT_CODE_IDLE_LINE {
                warn!("Detected LTP but result_code {result_code} != {RESULT_CODE_IDLE_LINE}!");
            }
            trace!("LTP: code: {result_code}, CPU cycles spent: {cycles_receiving}");
            return ReceivedTransmission::LinkTestPulse;
        }
        warn!(
            "Truncated transmission (code: {result_code}, size: {result_size}, CPU cycles spent: \
            {cycles_receiving}): {:08b} {:08b} {:08b}",
            buffer[0], buffer[1], buffer[2],
        );
        // If we received more than one byte of data but not enough to make up a whole frame, then
        // this was likely not a real frame, or a truncated transmission.
        return ReceivedTransmission::TruncatedTransmission;
    }

    // We've distinguished LTPs and truncated transmissions from likely packets at this point. This
    // is as much as we can do at the PHY layer. We'll return the data to the MAC layer and let it
    // handle detecting the SFD, aligning the frame data, and validating the CRC.

    // Before returning the data we should, however, strip off the last byte if it contains only
    // TP_IDL data. The receive loop above will write up to two sequential 1 bits that do not
    // correspond to real data to the final byte in the buffer before it detects the TP_IDL signal.
    // Therefore, if the final buffer byte is <=0b11, we know that that only contains TP_IDL
    // non-data, and can be skipped.
    let result_size = if decoded_byte <= 0b11 {
        result_size - 1
    } else {
        result_size
    };
    trace!(
        "PHY Packet received: code {result_code}, size {result_size}, decoded_byte \
        {decoded_byte:08b}, CPU cycles spent: {cycles_receiving}"
    );
    return ReceivedTransmission::Packet(&mut buffer[0..result_size]);
}
