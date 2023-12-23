//! This binary runs various microbenchmarks, measuring the CPU cycles spent executing them.
//!
//! It measures the number of CPU cycles spent executing a given number of iterations of a various
//! looping blocks of assembly code, and tries to define a set of rules that describe/predict how
//! many CPU cycles are spent executing each benchmark,
//!
//! WARNING: The observed results, and the rules defined in this example, are inherently fragile and
//! chip-specific. They have currently only been run on an ESP32-C3 and an ESP32-C6 chip. I can
//! re-run the benchmark and consistently get the same results when using a given toolchain version,
//! but I wouldn't be surprised if some of the results change when run by others or when run with
//! newer/older versions of the toolchain. The benchmarks are most useful when they reflect the
//! particular situation you're interested in as closely as possible, and it's easy to add new
//! benchmarks for additional cases.
//!
//! Some of the benchmarks seem particularly fragile. For example, the memory load/store benchmarks
//! seems to be dependent on the location and/or alignment of the memory buffers. In the past I've
//! noticed that a benchmark result would change as a result of adding another unrelated benchmark
//! to the set, with the only impact on the load/store benchmark assembly code being that the data
//! buffer's address changed as a result. I've tried to make the code more robust to this by
//! introducing variations of some benchmarks with buffers with different levels of guaranteed
//! alignment, but I wouldn't be surprised if some amount of non-determinism is still hiding
//! somewhere.
//!
//! The set of 'rules' I've come up with also aren't fully consistent. E.g.
//! [cycles::BRANCH_FWD_TAKEN_EXTRA_SLOW] represents a situation in which a taken conditional
//! forward branch takes an extra CPU cycle compared to all other cases of taken conditional forward
//! branches, which I can't fully explain.
//!
//! However, the upside is that all or most of the exceptions to the rules seem to apply only to the
//! first or second iterations, but not the subsequent repeated iterations after that. This means
//! that even with some exceptions to the rule, we can probably reason about the behavior of a loop
//! as long as there's enough repetitions of it.j

#![no_std]
#![no_main]
#![feature(asm_const)]

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use core::arch::asm;
use esp_backtrace as _;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, IO};
use log::{info, log};
use niccle_proc_macros::asm_with_perf_counter;

/// The address of the Machine Performance Counter Event Register.
const MPCER: usize = 0x7E0;
/// The address of the Machine Performance Counter Mode Register.
const MPCMR: usize = 0x7E1;

/// This CSR isn't defined by the esp-rs/esp-hal framework yet (nor by the espressif/svd files),
/// so we hardcode their addresses. See the "1.14 Dedicated IO" chapter of the technical
/// reference manual.
const CSR_CPU_GPIO_OUT: u32 = 0x805;
/// For now the implementation is hardcoded to always use CPU output signal 0 when using
/// dedicated IO on the TX pin. In the future we could try to make this more flexible, e.g.
/// using an argument, or perhaps even a const generic parameter with a const expression bound.
#[cfg(feature = "esp32c3")]
pub const TX_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal = hal::gpio::OutputSignal::CPU_GPIO_0;
#[cfg(feature = "esp32c6")]
pub const TX_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal = hal::gpio::OutputSignal::CPU_GPIO_OUT0;
/// The dedicated IO output signal index to use (from 0 to 7, where 0 corresponds to
/// CPU_GPIO_OUT0). Based on whatever [TX_CPU_OUTPUT_SIGNAL] is set to.
#[cfg(feature = "esp32c3")]
const TX_CPU_OUTPUT_SIGNAL_CSR_IDX: isize =
    (TX_CPU_OUTPUT_SIGNAL as isize) - hal::gpio::OutputSignal::CPU_GPIO_0 as isize;
#[cfg(feature = "esp32c6")]
const TX_CPU_OUTPUT_SIGNAL_CSR_IDX: isize =
    (TX_CPU_OUTPUT_SIGNAL as isize) - hal::gpio::OutputSignal::CPU_GPIO_OUT0 as isize;

/// Runs the set of benchmarks in a loop, printing results to the serial line.
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    #[cfg(feature = "esp32c3")]
    let system = peripherals.SYSTEM.split();
    #[cfg(feature = "esp32c6")]
    let system = peripherals.PCR.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    esp_println::logger::init_logger_from_env();
    info!("Booted up!");

    // Set up the dedicated GPIO feature on pin GPIO5, which some benchmarks use to allow us to
    // observe the CPU timing with an oscilloscope.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut tx_pin = io.pins.gpio7;
    tx_pin.set_to_push_pull_output();
    tx_pin.connect_peripheral_to_output_with_options(
        TX_CPU_OUTPUT_SIGNAL,
        false,
        false,
        true,
        false,
    );

    let mut delay = Delay::new(&clocks);
    loop {
        do_bench(branch_fwd_usually_taken, &mut delay);
        do_bench(branch_fwd_usually_taken_w_extra_instr, &mut delay);
        do_bench(branch_fwd_rarely_taken, &mut delay);
        do_bench(branch_fwd_rarely_taken_w_unaligned_jmp_back, &mut delay);
        do_bench(branch_back_rarely_taken, &mut delay);
        do_bench(branch_back_rarely_taken_w_gpio, &mut delay);
        do_bench(branch_back_usually_taken_aligned, &mut delay);
        do_bench(branch_back_usually_taken_unaligned, &mut delay);
        do_bench(branch_back_usually_taken_with_gpio_aligned, &mut delay);
        do_bench(branch_back_usually_taken_with_gpio_unaligned, &mut delay);
        do_bench(branch_back_usually_taken_with_final_nop, &mut delay);
        do_bench(branch_back_usually_taken_w_dead_branch_fwd, &mut delay);
        do_bench(branch_back_usually_taken_w_aligned_jmp_fwd, &mut delay);
        do_bench(branch_back_usually_taken_w_unaligned_jmp_fwd, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align1, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align2, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align4, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align8, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align256, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align1_w_gpio, &mut delay);
        do_bench(branch_back_usually_taken_w_lw_align1, &mut delay);
        do_bench(branch_back_usually_taken_w_lw_align2, &mut delay);
        do_bench(branch_back_usually_taken_w_lw_align4, &mut delay);
        do_bench(branch_back_usually_taken_w_lw_align8, &mut delay);
        do_bench(branch_back_usually_taken_w_lw_align256, &mut delay);
        do_bench(branch_back_usually_taken_w_sb_align1, &mut delay);
        do_bench(branch_back_usually_taken_w_sb_align2, &mut delay);
        do_bench(branch_back_usually_taken_w_sb_align4, &mut delay);
        do_bench(branch_back_usually_taken_w_sb_align8, &mut delay);
        do_bench(branch_back_usually_taken_w_sb_align256, &mut delay);
        do_bench(branch_back_usually_taken_w_sb_align1_w_gpio, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align1, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align2, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align4, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align8, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align256, &mut delay);
        info!("--------------------------------------------------");
    }
}

/// Runs a given benchmark for 1, 2, 3, 4, and 1000 iterations, to validate that our predictions are
/// valid for any number of iterations. I so far haven't seen any CPU behavior that differs between
/// the 3rd and subsequent iterations.
fn do_bench(bench: fn(u32) -> (&'static str, u32, u32), delay: &mut Delay) {
    do_bench_single(bench, delay, 1);
    do_bench_single(bench, delay, 2);
    do_bench_single(bench, delay, 3);
    do_bench_single(bench, delay, 4);
    do_bench_single(bench, delay, 1000);
}

/// Runs a given number of iterations for a single benchmark, and prints results.
fn do_bench_single(bench: fn(u32) -> (&'static str, u32, u32), delay: &mut Delay, iters: u32) {
    // Enable performance counting, starting with CPU cycles.
    unsafe {
        asm!(
            // What to count:
            // - 1<<0: CPU cycles
            // - 1<<1: instructions
            // - 1<<2: load hazards
            // - 1<<3: jump hazards
            // - 1<<4: idle cycles
            "csrw {mpcer}, (1<<0)",
            "csrw {mpcmr}, 0", // Disable the counter
            "csrw {mpcmr}, 1", // Enable the counter.
            mpcer = const(MPCER),
            mpcmr = const(MPCMR),
        );
    }
    let (name, cpu_predicted, cpu_actual) = bench(iters);
    // Count instructions (we ignore the "predicted" return value).
    unsafe { asm!("csrwi {mpcer}, (1<<1)", mpcer=const(MPCER)) };
    let (_, _, instrs_actual) = bench(iters);
    // Count load hazards.
    unsafe { asm!("csrwi {mpcer}, (1<<2)", mpcer=const(MPCER)) };
    let (_, _, load_hazards_actual) = bench(iters);
    // Count jump hazards.
    unsafe { asm!("csrwi {mpcer}, (1<<3)", mpcer=const(MPCER)) };
    let (_, _, jump_hazards_actual) = bench(iters);
    // Count idle cycles.
    unsafe { asm!("csrwi {mpcer}, (1<<4)", mpcer=const(MPCER)) };
    let (_, _, idle_actual) = bench(iters);

    // Print at error level if the prediction is wrong.
    let log_level = if cpu_predicted == cpu_actual {
        log::Level::Info
    } else {
        log::Level::Error
    };
    // Print the results of the benchmark.
    log!(
        log_level,
        "{name:45} {iters:4}x | \
        {cpu_predicted:4} vs {cpu_actual:4} actual \
        ({instrs_actual:4} INS, {idle_actual:4} IDL, {load_hazards_actual:4} LH, \
        {jump_hazards_actual} JH)"
    );

    // Throttle the benchmarks a bit so that we don't flood the serial display.
    delay.delay_ms(25u32);
}

/// A set of constants describing the number of CPU cycles various instructions take, in various
/// situations, and on the ESP32-C6 chip I used.
///
/// I arrived at these by trying to work backwards from the measured results I observed in the
/// benchmarks below.
///
/// The RISC-V instruction manual has the following to say about conditional branch instructions.
///
///   Software should be optimized such that the sequential code path is the most common path, with
///   less-frequently taken code paths placed out of line.
///
///   Software should also assume that backward branches will be predicted taken and forward
///   branches as not taken, at least the first time they are encountered.
///
///   Dynamic predictors should quickly learn any predictable branch behavior.
///
/// We'll reference this part of the spec in some of the branch instruction comments below.
pub mod cycles {
    /// A macro that selects one the provided values, based on the chip currently being targeted.
    macro_rules! chip_dependent {
        (esp32c3=$esp32c3:expr, esp32c6=$esp32c6:expr) => {
            if cfg!(feature = "esp32c3") {
                $esp32c3
            } else if cfg!(feature = "esp32c6") {
                $esp32c6
            } else {
                panic!("pick a feature, either esp32c3 or esp32c6")
            }
        };
    }

    /// The `nop` instruction.
    pub const NOP: u32 = 1;
    /// The `csrr*` instructions.
    pub const CSRR: u32 = 1;
    /// The `add` and `addi` instructions.
    pub const ADD: u32 = 1;
    /// The `xor` instructions.
    pub const XOR: u32 = 1;

    /// The `j` instruction. On ESP32-C6 some jumps take only one cycle. On ESP32-C3 they all take
    /// two cycles.
    pub const JUMP: u32 = chip_dependent!(esp32c3 = JUMP_EXTRA_SLOW, esp32c6 = 1);
    /// The `j` instruction. On ESP32-C6 sometimes jumps take two cycles instead of one. At first I
    /// thought it happened with all unaligned jumps, but that doesn't seem be the case
    /// consistently, at least not consistently.
    pub const JUMP_EXTRA_SLOW: u32 = 2;

    /// The `lb`, `lbu`, `lw` instructions.
    pub const LOAD: u32 = 2;
    /// The `sb` and `sw` instructions.
    pub const STORE: u32 = 1;

    // --- INITIAL ENCOUNTERS OF CONDITIONAL BRANCHES WITH EXPECTED OUTCOMES.
    //
    /// The `beq`, `bne`, and similar instructions with a forward target label. When a forward
    /// branch is encountered for the first time and is not taken (in line with the RISC-V manual
    /// spec), it takes two cycles on ESP32-C6, but only one cycle on ESP32-C3. In both cases this
    /// is faster than a *taken* forward branch ([BRANCH_FWD_TAKEN]), which would be unexpected as
    /// per the spec.
    pub const BRANCH_FWD_NOT_TAKEN_INITIAL: u32 =
        chip_dependent!(esp32c3 = BRANCH_FWD_NOT_TAKEN_SUBSEQUENT, esp32c6 = 2);

    /// The `beq`, `bne`, and similar instructions with a backward target label. When a backward
    /// branch is encountered for the first time and is taken (in line with the RISC-V manual spec),
    /// it takes two cycles on ESP32-C6 and three cycles on ESP32-C3. On ESP32-C6 this is faster
    /// than a *non-taken* backward branch ([BRANCH_BACK_NOT_TAKEN]), which would be unexpected as
    /// per the spec. One ESP32-C3 this is actually slower than [BRANCH_BACK_NOT_TAKEN].
    pub const BRANCH_BACK_TAKEN_INITIAL: u32 = chip_dependent!(esp32c3 = 3, esp32c6 = 2);

    // --- REPEATED ENCOUNTERS OF CONDITIONAL BRANCHES WITH EXPECTED OUTCOMES.
    //
    /// The `beq`, `bne`, and similar instructions with a forward target label. When a forward
    /// non-taken branch is encountered more than once, it only takes one cycle. Presumably because
    /// some further branch prediction mechanism or pipelining kicks in. This is the same as for
    /// repeated backward taken branches.
    pub const BRANCH_FWD_NOT_TAKEN_SUBSEQUENT: u32 = 1;
    /// The `beq`, `bne`, and similar instructions with a backward target label. When a backward
    /// taken branch is encountered more than once, it only takes one cycle on ESP32-C6. Presumably
    /// because some further branch prediction mechanism or pipelining kicks in. This is the same as
    /// for repeated forward non-taken branches. On ESP32-C3 it actually takes three cycles, the
    /// same as when the instruction is initially encountered, as if no branch prediction exists for
    /// such branches.
    pub const BRANCH_BACK_TAKEN_SUBSEQUENT: u32 =
        chip_dependent!(esp32c3 = BRANCH_BACK_TAKEN_INITIAL, esp32c6 = 1);

    // --- ENCOUNTERS OF CONDITIONAL BRANCHES WITH UNEXPECTED OUTCOMES.
    //
    /// The `beq`, `bne`, and similar instructions with a forward target label. When a forward taken
    /// branch is encountered it takes three (C6) / four (C3) cycles. This is the same as a
    /// non-taken backward branch. It is also longer than a non-taken branch, and probably because
    /// they're not expected to be taken in the usual case, as per the RISC-V manual.
    pub const BRANCH_FWD_TAKEN: u32 = 3;
    /// The `beq`, `bne`, and similar instructions with a forward target label. Sometimes a taken
    /// forward branch an extra cycle compared to the usual [BRANCH_FWD_TAKEN]. In our benchmark
    /// this is the case when there's a non-jump instruction behind the conditional branch
    /// instruction.
    pub const BRANCH_FWD_TAKEN_EXTRA_SLOW: u32 = chip_dependent!(esp32c3 = 3, esp32c6 = 4);

    /// The `beq`, `bne`, and similar instructions with a backward target label. On ESP32-C6 when a
    /// backward non-taken branch is encountered it takes three cycles in some cases, or four cycles
    /// in other cases ([BRANCH_BACK_NOT_TAKEN_INSTR_UNALIGNED]). This is longer than a non-taken branch,
    /// probably because they are expected to be taken in the usual case, as per the RISC-V spec.
    /// One ESP32-C3 backward non-taken branches take a single cycle, just like non-taken forward
    /// branches.
    pub const BRANCH_BACK_NOT_TAKEN: u32 = chip_dependent!(esp32c3 = 1, esp32c6 = 3);

    /// The `beq`, `bne`, and similar instructions with a backward target label sometimes take an
    /// extra cycle on ESP32-C6, when the instruction falls on a non-4 byte-aligned address (ending
    /// in 0x2, 0x6, ...).
    pub const BRANCH_BACK_NOT_TAKEN_INSTR_UNALIGNED: u32 =
        chip_dependent!(esp32c3 = BRANCH_BACK_NOT_TAKEN, esp32c6 = 4);
}

/// Measures the case of a loop with a forward branch instruction that is taken for all but the last
/// iteration. This goes against the RISC-V manual guidance that forward branch instructions will be
/// predicted not-taken initially.
#[ram]
fn branch_fwd_usually_taken(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "addi {i}, {i}, 1",
            "bne {i}, {iters}, 2f",
            "j 3f",
            ".align 2",
            "2: j 1b",
            ".align 2",
            "3:",
            iters = in(reg) iters,
            i = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::JUMP;
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_TAKEN + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2.. => predicted_iter_last + (predicted_iter_rest) * (iters - 1),
    };

    ("BNE FWD usually taken", predicted, cycles)
}

/// Like [branch_fwd_usually_taken] but with an extra `nop` instruction after the `bne` instruction,
/// which seems to slow the 2nd iteration down a bit (see [cycles::BRANCH_FWD_TAKEN_EXTRA_SLOW]). I
/// can't really explain why.
#[ram]
fn branch_fwd_usually_taken_w_extra_instr(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "addi {i}, {i}, 1",
            "bne {i}, {iters}, 2f",
            "nop",
            "j 3f",
            ".align 2",
            "2: j 1b",
            ".align 2",
            "3:",
            iters = in(reg) iters,
            i = inout(reg) 0 => _
        )
    };
    let predicted_iter_last =
        cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::NOP + cycles::JUMP;
    let predicted_iter_first = cycles::ADD + cycles::BRANCH_FWD_TAKEN_EXTRA_SLOW + cycles::JUMP;
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_TAKEN + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };

    (
        "BNE FWD usually taken w/ extra instruction",
        predicted,
        cycles,
    )
}

/// Measures the case of a loop with a forward branch instruction that is taken only in the last
/// iteration of the loop. This is in line with the RISC-V manual guidance that forward branch
/// instructions will be predicted not-taken initially, and is a type of loop you might actually
/// encounter in real code (e.g. a bail-out break condition).
///
/// See [branch_fwd_rarely_taken_w_unaligned_jmp_back] for a similar benchmark but with an unaligned
/// jump target for the unconditional jump instruction.
#[ram]
fn branch_fwd_rarely_taken(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "addi {y}, {y}, 1",
            "beq {y}, {iters}, 2f",
            "j 1b",
            ".align 2",
            "2:",
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::ADD + cycles::BRANCH_FWD_TAKEN;
    let predicted_iter_first = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::JUMP;
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    ("BEQ FWD rarely taken", predicted, cycles)
}

/// Like [branch_fwd_rarely_taken] but with an unaligned jump target for the unconditional jump
/// instruction.
///
/// The behavior ends up being exactly the same (even though I've seen unaligned jumps take two
/// cycles in other situations...).
#[ram]
fn branch_fwd_rarely_taken_w_unaligned_jmp_back(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "nop",
            "1:", // Not 4 byte-aligned, due to the 16-bit instruction right before this one.
            "addi {y}, {y}, 1",
            "beq {y}, {iters}, 2f",
            "j 1b",
            ".align 2",
            "2:",
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::NOP + cycles::ADD + cycles::BRANCH_FWD_TAKEN;
    let predicted_iter_first = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::JUMP;
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (
        "BEQ FWD rarely taken w/ unaligned J BACK",
        predicted,
        cycles,
    )
}

/// Measures the case of a loop with a backward branch instruction that is only taken in the last
/// iteration of the loop. This goes against the RISC-V manual guidance that backward branch
/// instructions will be predicted taken initially, and is not something you'd commonly write.
#[ram]
fn branch_back_rarely_taken(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "j 2f", // 16-bit instruction.
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned. (NEVER EXECUTED)
            "1:", // Hence, 4 byte-aligned.
            "j 3f", // 16-bit instruction.
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned. (NEVER EXECUTED)
            "2:",
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned.
            "addi {y}, {y}, 1", // 16-bit instruction.
            "beq {y}, {iters}, 1b", // Hence, 4 byte-aligned.
            "j 2b",  // Also 4 byte-aligned.
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned. (NEVER EXECUTED)
            "3:", // Hence, 4 byte-aligned.
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::JUMP_EXTRA_SLOW
        + cycles::NOP
        + cycles::ADD
        + cycles::BRANCH_BACK_TAKEN_INITIAL
        + cycles::JUMP_EXTRA_SLOW;
    let predicted_iter_rest =
        cycles::NOP + cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2.. => predicted_iter_last + (predicted_iter_rest) * (iters - 1),
    };
    ("BEQ BACK rarely taken", predicted, cycles)
}

/// Like [branch_back_rarely_taken], but with dedicated GPIO instructions inserted between some of
/// the each instructions in the loop. This allows us to inspect the per-iteration timing with an
/// oscilloscope. When we do so, we can confirm that it is the jump instructions that take one extra
/// cycle each, in the last iteration (rather than, say, the taken branch instruction taken longer).
#[ram]
fn branch_back_rarely_taken_w_gpio(iters: u32) -> (&'static str, u32, u32) {
    // Make sure the pin is set low at the start of the benchmark, without including this
    // instruction in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    let cycles = unsafe {
        asm_with_perf_counter!(
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            "j 4f", // 16-bit instruction.
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned. (NEVER EXECUTED)
            "1:", // Hence, 4 byte-aligned.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction
            "j 3f", // 16-bit instruction.
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned. (NEVER EXECUTED)
            "4:",
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction
            "2:",
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned.
            "addi {y}, {y}, 1", // 16-bit instruction.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            "beq {y}, {iters}, 1b", // Hence, 4 byte-aligned.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction
            "j 2b",  // Also 4 byte-aligned.
            "nop", // 16-bit instruction to ensure the next one is 4 byte-aligned. (NEVER EXECUTED)
            "3:", // Hence, 4 byte-aligned.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    // Make sure the pin is set low at the end of the benchmark, without including this instruction
    // in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };

    let predicted_iter_last = cycles::CSRR
        + cycles::JUMP_EXTRA_SLOW
        + cycles::CSRR * 2
        + cycles::NOP
        + cycles::ADD
        + cycles::CSRR * 2
        + cycles::BRANCH_BACK_TAKEN_INITIAL
        + cycles::CSRR
        + cycles::JUMP_EXTRA_SLOW
        + cycles::CSRR;
    let predicted_iter_rest = cycles::CSRR
        + cycles::NOP
        + cycles::ADD
        + cycles::CSRR * 2
        + cycles::BRANCH_BACK_NOT_TAKEN
        + cycles::CSRR
        + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2.. => predicted_iter_last + (predicted_iter_rest) * (iters - 1),
    };
    ("BEQ BACK rarely taken w/ GPIO", predicted, cycles)
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. This is in line with the RISC-V manual guidance that backward branch
/// instructions will be predicted taken initially, and is a type of loop you might actually
/// encounter in real code (e.g. a simple for loop with a counter as the exit condition).
#[ram]
fn branch_back_usually_taken_aligned(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:", // This label and the first instruction are two-byte aligned.
            "addi {y}, {y}, 1", // This is a 16-bit instruction.
            "nop", // This is a 16-bit instruction, to ensure the next one is 4 byte-aligned.
            "bne {y}, {iters}, 1b", // Hence, this is a 4 byte-aligned instruction.
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first = cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken aligned", predicted, cycles)
}

/// Like [branch_back_usually_taken_aligned] but with an unaligned jump target. This seems to have the same
/// behavior, but it's good to verify since I previously saw unaligned unconditional jumps take
/// longer than aligned unconditional jumps.
#[ram]
fn branch_back_usually_taken_unaligned(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:", // This label and the first instruction are two-byte aligned.
            "addi {y}, {y}, 1", // This is a 16-bit instruction
            "bne {y}, {iters}, 1b", // Hence this instruction's address is *not* 4-byte-aligned.
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    // The branch is extra slow b/c the instruction is not 4 byte-aligned.
    let predicted_iter_last = cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN_INSTR_UNALIGNED;
    let predicted_iter_first = cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken unaligned", predicted, cycles)
}

/// Like [branch_back_usually_taken_aligned], but with dedicated GPIO instructions inserted between
/// each instruction in the loop. This allows us to inspect the per-iteration timing with an
/// oscilloscope. When we do so, we can confirm that in all cases the final iteration takes the same
/// number of CPU cycles (i.e. the untaken backward branch always take the same number of cycles,
/// regardless of whether one, two or more iterations are run).
#[ram]
fn branch_back_usually_taken_with_gpio_aligned(iters: u32) -> (&'static str, u32, u32) {
    // Make sure the pin is set low at the start of the benchmark, without including this
    // instruction in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    // Run the benchmark, setting and clearing the pin between each instruction.
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:", // 4 byte-aligned.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            "addi {y}, {y}, 1", // 16-bit instruction
            "nop", // 16-bit instruction, to ensure next instruction is 4 byte-aligned.
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction.
            "bne {y}, {iters}, 1b", // Hence, 4 byte-aligned.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    // Make sure the pin is set low at the end of the benchmark, without including this instruction
    // in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    let predicted_iter_last = cycles::CSRR
        + cycles::ADD
        + cycles::NOP
        + cycles::CSRR
        + cycles::BRANCH_BACK_NOT_TAKEN
        + cycles::CSRR;
    let predicted_iter_first =
        cycles::CSRR + cycles::ADD + cycles::NOP + cycles::CSRR + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::CSRR
        + cycles::ADD
        + cycles::NOP
        + cycles::CSRR
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken w/ GPIO aligned", predicted, cycles)
}

/// Like [branch_back_usually_taken_unaligned], but with dedicated GPIO instructions inserted
/// between each instruction in the loop. This allows us to inspect the per-iteration timing with an
/// oscilloscope. When we do so, we can confirm that in all cases the final iteration takes the same
/// number of CPU cycles (i.e. the untaken backward branch always take the same number of cycles,
/// regardless of whether one, two or more iterations are run).
#[ram]
fn branch_back_usually_taken_with_gpio_unaligned(iters: u32) -> (&'static str, u32, u32) {
    // Make sure the pin is set low at the start of the benchmark, without including this
    // instruction in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    // Run the benchmark, setting and clearing the pin between each instruction.
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            "addi {y}, {y}, 1", // 16-bit instruction
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction
            "bne {y}, {iters}, 1b", // Hence, not 4 byte-aligned. !!!
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    // Make sure the pin is set low at the end of the benchmark, without including this instruction
    // in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    let predicted_iter_last = cycles::CSRR
        + cycles::ADD
        + cycles::CSRR
        // The branch is extra slow b/c the instruction is not 4 byte-aligned.
        + cycles::BRANCH_BACK_NOT_TAKEN_INSTR_UNALIGNED
        + cycles::CSRR;
    let predicted_iter_first =
        cycles::CSRR + cycles::ADD + cycles::CSRR + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::CSRR + cycles::ADD + cycles::CSRR + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (
        "BNE BACK usually taken w/ GPIO unaligned",
        predicted,
        cycles,
    )
}

/// Like [branch_back_usually_taken_aligned] but with a final nop at the end. In this case the branch
/// instruction one less CPU cycle, but the nop takes one additional CPU cycle, resulting in the
/// same total number of CPU cycles.
#[ram]
fn branch_back_usually_taken_with_final_nop(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "addi {y}, {y}, 1",
            "bne {y}, {iters}, 1b",
            "nop", // This nop causes the non-taken branch instruction to take one fewer CPU cycle.
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN + cycles::NOP;
    let predicted_iter_first = cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken w/ final NOP", predicted, cycles)
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a forward branch instruction that is
/// never taken. This represents a common scenario of a conditional loop with a rarely-taken
/// bail-out break condition.
#[ram]
fn branch_back_usually_taken_w_dead_branch_fwd(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "addi {y}, {y}, 1", // 16 bit instruction
            "beq zero, {tmp}, 2f", // never taken, i.e. dead branch, 32bit instruction
            "bne {y}, {iters}, 1b", // 32 bit instruction
            "2:", // Guaranteed to be aligned
            iters = in(reg) iters,
            y = inout(reg) 0 => _,
            tmp = inout(reg) 1 => _
        )
    };
    let predicted_iter_last =
        cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first =
        cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::ADD
        + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken w/ dead FWD BEQ", predicted, cycles)
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a forward jump instruction to an
/// aligned target.
///
/// This is meant to gauge the cost of the unconditional jump instructions.
#[ram]
fn branch_back_usually_taken_w_aligned_jmp_fwd(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "j 2f",
            ".align 4",
            "2:",
            "addi {y}, {y}, 1", // 16-bit instruction
            "nop", // 16-bit instruction
            "bne {y}, {iters}, 1b", // Hence, 4 byte-aligned.
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    let predicted_iter_last =
        cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first =
        cycles::JUMP + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken w/ aligned J FWD", predicted, cycles)
}

/// Like [branch_back_usually_taken_w_aligned_jmp_fwd] but with unconditional jump target that is
/// *not* 4 byte-aligned. This is meant to check whether the jump target alignment matters (it
/// doesn't seem to).
#[ram]
fn branch_back_usually_taken_w_unaligned_jmp_fwd(iters: u32) -> (&'static str, u32, u32) {
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "j 2f",
            ".align 4",
            "nop", // 16-bit instruction
            "2:", // Guaranteed to be unaligned.
            "addi {y}, {y}, 1", // 16-bit-instruction
            "bne {y}, {iters}, 1b", // Hence, 4 byte-aligned.
            iters = in(reg) iters,
            y = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first = cycles::JUMP + cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (
        "BNE BACK usually taken w/ unaligned J FWD",
        predicted,
        cycles,
    )
}

/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 1-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align1(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken w/ LBU align(1)", unsafe {
        &mut DATA_U8_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken w/ LBU align(2)", unsafe {
        &mut DATA_U8_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 4 byte-aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken w/ LBU align(4)", unsafe {
        &mut DATA_U8_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken w/ LBU align(8)", unsafe {
        &mut DATA_U8_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken w/ LBU align(256) ", unsafe {
        &mut DATA_U8_ALIGN256.data
    })
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a load byte (`lbu`) instruction,
/// which takes a varying number of CPU cycles.
#[ram]
fn _branch_back_usually_taken_w_lbu(
    iters: u32,
    name: &'static str,
    data: &mut [u8],
) -> (&'static str, u32, u32) {
    // Ensure that the data is filled with ones, since we rely on that to end the loop iteration.
    data.fill(1u8);
    let data_ptr_range = data[0..iters as usize].as_ptr_range();
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            // LBU takes 2 cycles here. However, we must ensure there's actually a data dependency
            // on the result of the load instruction, otherwise the CPU can just pipeline it, and
            // then it will often take just a single cycle (or more precisely, it seems to take 1
            // cycle every 3 out of 4 iterations, and 2 cycles every 4th iteration).
            "lbu {tmp}, 0({data_ptr})", // 32-bit instruction
            "add {data_ptr}, {data_ptr}, {tmp}", // 16-bit instruction
            "nop", // 16-bit instruction
            "bne {data_ptr}, {data_end_ptr}, 1b", // Hence, 4 byte-aligned.
            data_ptr = inout(reg) data_ptr_range.start => _,
            data_end_ptr = in(reg) data_ptr_range.end,
            tmp = out(reg) _
        )
    };
    let predicted_iter_last =
        cycles::LOAD + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first =
        cycles::LOAD + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::LOAD + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (name, predicted, cycles)
}

/// Like [branch_back_usually_taken_w_lbu_align1], but with dedicated GPIO instructions inserted
/// between some of the each instructions in the loop. This allows us to inspect the per-iteration
/// timing with an oscilloscope. When we do so, we can confirm that the load instruction takes the
/// same number of CPU cycles in all cases, and does not have differing behavior between the first
/// execution and subsequent executions.
#[ram]
fn branch_back_usually_taken_w_lbu_align1_w_gpio(iters: u32) -> (&'static str, u32, u32) {
    // Ensure that the data is filled with ones, since we rely on that to end the loop iteration.
    let data_ptr_range;
    unsafe {
        DATA_U8_ALIGN1.data.fill(1u8);
        data_ptr_range = DATA_U8_ALIGN1.data[0..iters as usize].as_ptr_range();
    }

    // Make sure the pin is set low at the start of the benchmark, without including this
    // instruction in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    // Run the benchmark, setting and clearing the pin between each instruction.
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            // Note that just like in _branch_back_usually_taken_w_lbu, we must use an `add`
            // instruction to introduce a data dependency. Even more importantly in this benchmark:
            // the `add` instruction must immediately follow the `lbu` instruction, we must not
            // place the `csrr` instruction in between them, otherwise the CPU can seemingly make
            // use of the instruction pipelining, and then the `lbu` instruction only takes a single
            // CPU cycle three out of four times it's executed.
            "lbu {tmp}, 0({data_ptr})", // 32-bit instruction
            "add {data_ptr}, {data_ptr}, {tmp}", // 16-bit instruction
            "nop", // 16-bit instruction
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction
            "bne {data_ptr}, {data_end_ptr}, 1b",  // Hence, 4 byte-aligned.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
            data_ptr = inout(reg) data_ptr_range.start => _,
            data_end_ptr = in(reg) data_ptr_range.end,
            tmp = out(reg) _
        )
    };
    // Make sure the pin is set low at the end of the benchmark, without including this instruction
    // in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };

    let predicted_iter_last = cycles::CSRR
        + cycles::LOAD
        + cycles::ADD
        + cycles::NOP
        + cycles::CSRR
        + cycles::BRANCH_BACK_NOT_TAKEN
        + cycles::CSRR;
    let predicted_iter_first = cycles::CSRR
        + cycles::LOAD
        + cycles::ADD
        + cycles::NOP
        + cycles::CSRR
        + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::CSRR
        + cycles::LOAD
        + cycles::ADD
        + cycles::NOP
        + cycles::CSRR
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (
        "BNE BACK usually taken w/ LBU align(1) w/ GPIO",
        predicted,
        cycles,
    )
}

/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 1-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align1(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken w/ LW align(1)", unsafe {
        &mut DATA_U32_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken w/ LW align(2)", unsafe {
        &mut DATA_U32_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 4 byte-aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken w/ LW align(4)", unsafe {
        &mut DATA_U32_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken w/ LW align(8)", unsafe {
        &mut DATA_U32_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken w/ LW align(256) ", unsafe {
        &mut DATA_U32_ALIGN256.data
    })
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a load word (`lw`) instruction, which
/// takes a constant number of CPU cycles.
#[ram]
fn _branch_back_usually_taken_w_lw(
    iters: u32,
    name: &'static str,
    data: &mut [u32],
) -> (&'static str, u32, u32) {
    // Ensure that the data is filled with fours, since we rely on that to end the loop iteration.
    data.fill(4u32);
    let data_ptr_range = data[0..iters as usize].as_ptr_range();
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "lw {tmp}, 0({data_ptr})", // 32-bit instruction
            "add {data_ptr}, {data_ptr}, {tmp}", // 16-bit instruction
            "nop", // 16-bit instruction
            "bne {data_ptr}, {data_end_ptr}, 1b", // Hence, 4 byte-aligned.
            data_ptr = inout(reg) data_ptr_range.start => _,
            data_end_ptr = in(reg) data_ptr_range.end,
            tmp = out(reg) _
        )
    };
    let predicted_iter_last =
        cycles::LOAD + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first =
        cycles::LOAD + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::LOAD + cycles::ADD + cycles::NOP + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (name, predicted, cycles)
}

/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 1-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align1(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken w/ SB align(1)", unsafe {
        &DATA_U8_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken w/ SB align(2)", unsafe {
        &DATA_U8_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 4 byte-aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken w/ SB align(4)", unsafe {
        &DATA_U8_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken w/ SB align(8)", unsafe {
        &DATA_U8_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken w/ SB align(256)", unsafe {
        &DATA_U8_ALIGN256.data
    })
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a store byte (`sb`) instruction,
/// which takes a varying number of CPU cycles.
///
/// Note that the behavior of the first and/or second iteration seems to be quite dependent on
/// either the location and/or alignment of the array, as well as the other instructions in the
/// iteration (e.g. without the `nop` instructions the behavior becomes much less predictable).
#[ram]
fn _branch_back_usually_taken_w_sb(
    iters: u32,
    name: &'static str,
    data: &[u8],
) -> (&'static str, u32, u32) {
    let data_ptr_range = data[0..iters as usize].as_ptr_range();
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            // This seems to introduce a data dependency/hazard that ensures more consistent
            // benchmark results regardless of alignment/location of the data.
            "xor {tmp}, {tmp}, zero", // 32-bit instruction
            "sb {tmp}, 0({data_ptr})", // 32-bit instruction
            // without these nop every fourth `sb` takes more than one cycle and the loop becomes
            // less predictable.
            "nop", // 16-bit instruction
            "nop", // 16-bit instruction
            "addi {data_ptr}, {data_ptr}, 1", // 16-bit instruction
            "nop", // 16-bit instruction, to ensure next instruction is 4 byte-aligned.
            "bne {data_ptr}, {data_end_ptr}, 1b", // Hence, 4 byte-aligned.
            data_ptr = inout(reg) data_ptr_range.start => _,
            data_end_ptr = in(reg) data_ptr_range.end,
            tmp = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::STORE
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first = cycles::STORE
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::STORE
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (name, predicted, cycles)
}

/// Like [branch_back_usually_taken_w_sb_align1], but with dedicated GPIO instructions inserted
/// between some of the each instructions in the loop. This allows us to inspect the per-iteration
/// timing with an oscilloscope. When we do so, we can confirm that the store instruction takes the
/// same number of CPU cycles in all cases, and does not have differing behavior between the first
/// execution and subsequent executions.
#[ram]
fn branch_back_usually_taken_w_sb_align1_w_gpio(iters: u32) -> (&'static str, u32, u32) {
    // Ensure that the data is filled with ones, since we rely on that to end the loop iteration.
    let data_ptr_range;
    unsafe {
        DATA_U8_ALIGN1.data.fill(1u8);
        data_ptr_range = DATA_U8_ALIGN1.data[0..iters as usize].as_ptr_range();
    }

    // Make sure the pin is set low at the start of the benchmark, without including this
    // instruction in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    // Run the benchmark, setting and clearing the pin between each instruction.
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set, 32-bit instruction
            // This seems to introduce a data dependency/hazard that ensures more consistent
            // benchmark results regardless of alignment/location of the data.
            "xor {tmp}, {tmp}, zero", // 32-bit instruction
            "sb {tmp}, 0({data_ptr})", // 32-bit instruction
            "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear, 32-bit instruction
            // without these nop every fourth `sb` takes more than one cycle and the loop becomes
            // less predictable.
            "nop", // 16-bit instruction
            "nop", // 16-bit instruction
            "addi {data_ptr}, {data_ptr}, 1", // 16-bit instruction
            "nop", // 16-bit instruction, to ensure next instruction is 4 byte-aligned.
            "bne {data_ptr}, {data_end_ptr}, 1b", // Hence, 4 byte-aligned.
            "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Set
            csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
            cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
            data_ptr = inout(reg) data_ptr_range.start => _,
            data_end_ptr = in(reg) data_ptr_range.end,
            tmp = inout(reg) 0 => _
        )
    };
    // Make sure the pin is set low at the end of the benchmark, without including this instruction
    // in the cycle count.
    unsafe {
        asm!(
        "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // Clear
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX))
    };
    let predicted_iter_last = cycles::CSRR
        + cycles::XOR
        + cycles::STORE
        + cycles::CSRR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_NOT_TAKEN
        + cycles::CSRR;
    let predicted_iter_first = cycles::CSRR
        + cycles::XOR
        + cycles::STORE
        + cycles::CSRR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::CSRR
        + cycles::XOR
        + cycles::STORE
        + cycles::CSRR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (
        "BNE BACK usually taken w/ SB align(1) w/ GPIO",
        predicted,
        cycles,
    )
}

/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 1-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align1(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken w/ SW align(1)", unsafe {
        &DATA_U32_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken w/ SW align(2)", unsafe {
        &DATA_U32_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 4 byte-aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken w/ SW align(4)", unsafe {
        &DATA_U32_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken w/ SW align(8)", unsafe {
        &DATA_U32_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken w/ SW align(256)", unsafe {
        &DATA_U32_ALIGN256.data
    })
}

// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a store word (`sw`) instruction,
/// which takes a varying number of CPU cycles.
#[ram]
fn _branch_back_usually_taken_w_sw(
    iters: u32,
    name: &'static str,
    data: &[u32],
) -> (&'static str, u32, u32) {
    let data_ptr_range = data[0..iters as usize].as_ptr_range();
    let cycles = unsafe {
        asm_with_perf_counter!(
            "1:",
            // This seems to introduce a data dependency/hazard that ensures more consistent
            // benchmark results regardless of alignment/location of the data.
            "xor {tmp}, {tmp}, zero", // 32-bit instruction
            "sw zero, 0({data_ptr})", // 32-bit instruction
            // without these nop every fourth `sw` takes more than one cycle and the loop becomes
            // less predictable.
            "nop", // 16-bit instruction
            "nop", // 16-bit instruction
            "addi {data_ptr}, {data_ptr}, 4", // 16-bit instruction
            "nop", // 16-bit instruction, to ensure next instruction is 4 byte-aligned.
            "bne {data_ptr}, {data_end_ptr}, 1b", // Hence, 4 byte-aligned.
            data_ptr = inout(reg) data_ptr_range.start => _,
            data_end_ptr = in(reg) data_ptr_range.end,
            tmp = inout(reg) 0 => _
        )
    };
    let predicted_iter_last = cycles::STORE
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter_first = cycles::STORE
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::STORE
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::NOP
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter_last,
        2 => predicted_iter_last + predicted_iter_first,
        3.. => predicted_iter_last + predicted_iter_first + (predicted_iter_rest) * (iters - 2),
    };
    (name, predicted, cycles)
}

#[repr(align(1))]
struct WriteDataU8Align1 {
    data: [u8; 1000],
}
static mut DATA_U8_ALIGN1: WriteDataU8Align1 = WriteDataU8Align1 { data: [0u8; 1000] };

#[repr(align(2))]
struct WriteDataU8Align2 {
    data: [u8; 1000],
}
static mut DATA_U8_ALIGN2: WriteDataU8Align2 = WriteDataU8Align2 { data: [0u8; 1000] };

#[repr(align(4))]
struct WriteDataU8Align4 {
    data: [u8; 1000],
}
static mut DATA_U8_ALIGN4: WriteDataU8Align4 = WriteDataU8Align4 { data: [0u8; 1000] };

#[repr(align(8))]
struct WriteDataU8Align8 {
    data: [u8; 1000],
}
static mut DATA_U8_ALIGN8: WriteDataU8Align8 = WriteDataU8Align8 { data: [0u8; 1000] };

#[repr(align(256))]
struct WriteDataU8Align256 {
    data: [u8; 1000],
}
static mut DATA_U8_ALIGN256: WriteDataU8Align256 = WriteDataU8Align256 { data: [0u8; 1000] };

#[repr(align(1))]
struct WriteDataU32Align1 {
    data: [u32; 1000],
}
static mut DATA_U32_ALIGN1: WriteDataU32Align1 = WriteDataU32Align1 { data: [0u32; 1000] };

#[repr(align(2))]
struct WriteDataU32Align2 {
    data: [u32; 1000],
}
static mut DATA_U32_ALIGN2: WriteDataU32Align2 = WriteDataU32Align2 { data: [0u32; 1000] };

#[repr(align(4))]
struct WriteDataU32Align4 {
    data: [u32; 1000],
}
static mut DATA_U32_ALIGN4: WriteDataU32Align4 = WriteDataU32Align4 { data: [0u32; 1000] };

#[repr(align(8))]
struct WriteDataU32Align8 {
    data: [u32; 1000],
}
static mut DATA_U32_ALIGN8: WriteDataU32Align8 = WriteDataU32Align8 { data: [0u32; 1000] };

#[repr(align(256))]
struct WriteDataU32Align256 {
    data: [u32; 1000],
}
static mut DATA_U32_ALIGN256: WriteDataU32Align256 = WriteDataU32Align256 { data: [0u32; 1000] };
