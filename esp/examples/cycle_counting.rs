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
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay};
use log::{info, log};
use niccle_proc_macros::asm_with_perf_counter;

/// The address of the Machine Performance Counter Event Register.
const MPCER: usize = 0x7E0;
/// The address of the Machine Performance Counter Mode Register.
const MPCMR: usize = 0x7E1;

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

    let mut delay = Delay::new(&clocks);
    loop {
        do_bench(branch_fwd_usually_taken, &mut delay);
        do_bench(branch_fwd_usually_taken_w_extra_instr, &mut delay);
        do_bench(branch_fwd_rarely_taken, &mut delay);
        do_bench(branch_fwd_rarely_taken_w_unaligned_jmp_back, &mut delay);
        do_bench(branch_back_rarely_taken, &mut delay);
        do_bench(branch_back_usually_taken, &mut delay);
        do_bench(branch_back_usually_taken_w_dead_branch_fwd, &mut delay);
        do_bench(branch_back_usually_taken_w_aligned_jmp_fwd, &mut delay);
        do_bench(branch_back_usually_taken_w_unaligned_jmp_fwd, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align1, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align2, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align4, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align8, &mut delay);
        do_bench(branch_back_usually_taken_w_lbu_align256, &mut delay);
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
        do_bench(branch_back_usually_taken_w_sw_align1, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align2, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align4, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align8, &mut delay);
        do_bench(branch_back_usually_taken_w_sw_align256, &mut delay);
        info!("--------------------------------------------------");
    }
}

/// Runs a given benchmark for 1, 2, 3, 10, and 1000 iterations, to validate that the predictions
/// are valid for any number of iterations.
fn do_bench(bench: fn(u32) -> (&'static str, u32, u32), delay: &mut Delay) {
    do_bench_single(bench, delay, 1);
    do_bench_single(bench, delay, 2);
    do_bench_single(bench, delay, 3);
    do_bench_single(bench, delay, 10);
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
        ({instrs_actual:4} INS, {idle_actual:4} IDL, {load_hazards_actual:4} LH, {jump_hazards_actual} JH)"
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
    /// The `add` and `addi` instructions.
    pub const ADD: u32 = 1;
    /// The `xor` instructions.
    pub const XOR: u32 = 1;

    /// The `j` instruction. On ESP32-C6 most jumps take only one cycle. On ESP32-C3 they all take two cycles.
    pub const JUMP: u32 = chip_dependent!(esp32c3 = 2, esp32c6 = 1);
    /// The `j` instruction. On ESP32-C6 sometimes jumps take two cycles instead of one. At first I
    /// thought it happened with all unaligned jumps, but that doesn't seem be the case
    /// consistently, at least not consistently.
    pub const JUMP_EXTRA_SLOW: u32 = 2;

    /// The `lb` and `lbu` instructions. Initial load byte instructions take three cycles on
    /// ESP32-C6, rather than two cycles for subsequent executions. On ESP32-C3 they all take two
    /// cycles.
    pub const LOAD_BYTE_INITIAL: u32 = chip_dependent!(esp32c3 = LOAD_BYTE_SUBSEQUENT, esp32c6 = 3);
    /// The `lb` and `lbu` instructions. Subsequent executions after the first execution take only
    /// two cycles.
    pub const LOAD_BYTE_SUBSEQUENT: u32 = 2;
    /// The `lw` instruction. Load word instructions take two cycles. On ESP32-C6, contrary to the
    /// load byte instruction, the first/initial instruction doesn't take an extra cycle.
    pub const LOAD_WORD: u32 = 2;

    /// The `sb` and `sw` instructions. Initial store byte instructions take two cycles on ESP32-C6,
    /// rather than a single cycle for subsequent executions. They all take a single cycle on
    /// ESP32-C3.
    pub const STORE_INITIAL: u32 = chip_dependent!(esp32c3 = STORE_SUBSEQUENT, esp32c6 = 2);
    /// The `sb` and `sw` instructions. Subsequent executions after the first execution take only
    /// one cycle.
    pub const STORE_SUBSEQUENT: u32 = 1;

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
    /// The `beq`, `bne`, and similar instructions with a backward target label. On ESP32-C6
    /// sometimes an initial taken backward branch takes four cycles instead of the usual two. It's
    /// not super clear why. This doesn't happen on ESP32-C3.
    pub const BRANCH_BACK_TAKEN_INITIAL_EXTRA_SLOW: u32 =
        chip_dependent!(esp32c3 = BRANCH_BACK_TAKEN_INITIAL, esp32c6 = 4);

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
    /// backward non-taken branch is encountered it takes three cycles. This is the same as a taken
    /// forward branch. It is also longer than a non-taken branch, probably because they are
    /// expected to be taken in the usual case, as per the RISC-V spec. One ESP32-C3 backward
    /// non-taken branches take a single cycle, just like non-taken forward branches.
    pub const BRANCH_BACK_NOT_TAKEN: u32 = chip_dependent!(esp32c3 = 1, esp32c6 = 3);
}

/// Measures the case of a loop with a forward branch instruction that is taken for all but the last
/// iteration. This goes against the RISC-V manual guidance that forward branch instructions will be
/// predicted not-taken initiallyy
#[ram]
fn branch_fwd_usually_taken(iters: u32) -> (&'static str, u32, u32) {
    let cycles = asm_with_perf_counter!(
      ".align 2",
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
    );
    // Two single-cycle instructions (addi, j) plus a non-taken forward branch.
    let predicted_iter0 = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::JUMP;
    // Two single-cycle instructions plus a taken forward branch.
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_TAKEN + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2.. => predicted_iter0 + (predicted_iter_rest) * (iters - 1),
    };

    ("BNE FWD usually taken", predicted, cycles)
}

/// Like [branch_fwd_usually_taken] but with an extra `nop` instruction after the `bne` instruction,
/// which seems to slow the 2nd iteration down a bit (see [cycles::BRANCH_FWD_TAKEN_EXTRA_SLOW]). I
/// can't really explain why.
#[ram]
fn branch_fwd_usually_taken_w_extra_instr(iters: u32) -> (&'static str, u32, u32) {
    let cycles = asm_with_perf_counter!(
      ".align 2",
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
    );
    // Three single-cycle instructions (addi, nop, j) plus a non-taken forward branch.
    let predicted_iter0 =
        cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::NOP + cycles::JUMP;
    // Two single-cycle instructions (addi, j) plus a taken forward branch with a nop behind it, which makes it seem to take extra long.
    let predicted_iter1 = cycles::ADD + cycles::BRANCH_FWD_TAKEN_EXTRA_SLOW + cycles::JUMP;
    // Two single-cycle instructions (addi, j) plus a predicted taken forward branch.
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_TAKEN + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
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
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      "addi {y}, {y}, 1",
      "beq {y}, {iters}, 2f",
      "j 1b",
      ".align 2",
      "2:",
      iters = in(reg) iters,
      y = inout(reg) 0 => _
    );
    // One single-cycle instruction (addi) and a taken forward branch.
    let predicted_iter0 = cycles::ADD + cycles::BRANCH_FWD_TAKEN;
    // Two single-cycle instructions (addi, j) and a non-taken forward branch.
    let predicted_iter1 = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::JUMP;
    // Two single-cycle instructions (addi, j) and a predicted non-taken forward branch.
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
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
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "nop",
      "1:", // Not 2-byte aligned, due to the 16-bit instruction right before this one.
      "addi {y}, {y}, 1",
      "beq {y}, {iters}, 2f",
      "j 1b",
      ".align 2",
      "2:",
      iters = in(reg) iters,
      y = inout(reg) 0 => _
    );
    let predicted_iter0 = cycles::NOP + cycles::ADD + cycles::BRANCH_FWD_TAKEN;
    let predicted_iter1 = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::JUMP;
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
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
///
/// Note that the branch instruction in this benchmark exhibits some behavior that differs from most
/// other benchmarks, which I can't really explain (see
/// [cycles::BRANCH_BACK_TAKEN_INITIAL_EXTRA_SLOW]).
#[ram]
fn branch_back_rarely_taken(iters: u32) -> (&'static str, u32, u32) {
    let cycles = asm_with_perf_counter!(
      "j 2f",
      ".align 2",
      "1:",
      "j 3f",
      ".align 2",
      "2:",
      "addi {y}, {y}, 1",
      "beq {y}, {iters}, 1b",
      "nop", // Without this nop the iterations take an extra cycle anyway.
      "j 2b",
      ".align 2",
      "3:",
      iters = in(reg) iters,
      y = inout(reg) 0 => _
    );
    let predicted_iter0 =
        cycles::JUMP + cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL_EXTRA_SLOW + cycles::JUMP;
    let predicted_iter_rest =
        cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN + cycles::NOP + cycles::JUMP;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2.. => predicted_iter0 + (predicted_iter_rest) * (iters - 1),
    };
    ("BEQ BACK rarely taken", predicted, cycles)
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. This is in line with the RISC-V manual guidance that backward branch
/// instructions will be predicted taken initially, and is a type of loop you might actually
/// encounter in real code (e.g. a simple for loop with a counter as the exit condition).
#[ram]
fn branch_back_usually_taken(iters: u32) -> (&'static str, u32, u32) {
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      "addi {y}, {y}, 1",
      "bne {y}, {iters}, 1b",
      "nop", // Without this nop the last iteration takes an extra cycle anyway.
      iters = in(reg) iters,
      y = inout(reg) 0 => _
    );
    // Two single-cycle instructions (addi, nop) and a non-taken backward branch.
    let predicted_iter0 = cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN + cycles::NOP;
    // Two single-cycle instructions (addi, j) and a non-taken forward branch.
    let predicted_iter1 = cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    // Two single-cycle instructions (addi, j) and a predicted non-taken forward branch.
    let predicted_iter_rest = cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken", predicted, cycles)
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a forward branch instruction that is
/// never taken. This represents a common scenario of a conditional loop with a rarely-taken bail-out
/// break condition.
#[ram]
fn branch_back_usually_taken_w_dead_branch_fwd(iters: u32) -> (&'static str, u32, u32) {
    let cycles = asm_with_perf_counter!(
      "1:",
      "addi {y}, {y}, 1", // 16 bit instruction
      "beq zero, {tmp}, 2f", // never taken, i.e. dead branch, 32bit instruction
      "bne {y}, {iters}, 1b", // 32 bit instruction
      "2:", // Guaranteed to be aligned
      iters = in(reg) iters,
      y = inout(reg) 0 => _,
      tmp = inout(reg) 1 => _
    );
    let predicted_iter0 =
        cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_INITIAL + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter1 =
        cycles::ADD + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::ADD
        + cycles::BRANCH_FWD_NOT_TAKEN_SUBSEQUENT
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
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
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      "j 2f",
      ".align 2",
      "2:",
      "addi {y}, {y}, 1",
      "bne {y}, {iters}, 1b",
      "nop", // Without this nop the last iteration takes an extra cycle anyway.
      iters = in(reg) iters,
      y = inout(reg) 0 => _
    );
    let predicted_iter0 =
        cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN + cycles::NOP;
    let predicted_iter1 = cycles::JUMP + cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
    };
    ("BNE BACK usually taken w/ aligned J FWD", predicted, cycles)
}

/// Measures the case of a loop with a backward branch instruction that is taken in for all but the
/// last iteration of the loop. The loop in turn also contains a forward jump instruction to an
/// unaligned target.
///
/// This is meant to gauge the cost of the unconditional jump instructions.
#[ram]
fn branch_back_usually_taken_w_unaligned_jmp_fwd(iters: u32) -> (&'static str, u32, u32) {
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      "j 2f",
      ".align 2",
      "nop",
      "2:", // Guaranteed to be unaligned.
      "addi {y}, {y}, 1",
      "bne {y}, {iters}, 1b",
      "nop", // Without this nop the last iteration takes an extra cycle anyway.
      iters = in(reg) iters,
      y = inout(reg) 0 => _
    );
    let predicted_iter0 =
        cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN + cycles::NOP;
    let predicted_iter1 = cycles::JUMP + cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::JUMP_EXTRA_SLOW + cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
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
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken with LBU align(1)", unsafe {
        &mut DATA_U8_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken with LBU align(2)", unsafe {
        &mut DATA_U8_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 4-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken with LBU align(4)", unsafe {
        &mut DATA_U8_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(iters, "BNE BACK usually taken with LBU align(8)", unsafe {
        &mut DATA_U8_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_lbu]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lbu_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lbu(
        iters,
        "BNE BACK usually taken with LBU align(256) ",
        unsafe { &mut DATA_U8_ALIGN256.data },
    )
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
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      // LBU takes 2 or 3 cycles unconditionally. However, we must ensure there's actually a data
      // dependency on the result of the load instruction, otherwise the CPU can just ignore it, and
      // then it will often take just a single cycle (or more precisely, it seems to take 1 cycle
      // every 3 out of 4 iterations, and 2 cycles every 4th iteration).
      "lbu {tmp}, 0({data_ptr})",
      "add {data_ptr}, {data_ptr}, {tmp}",
      "bne {data_ptr}, {data_end_ptr}, 1b",
      data_ptr = inout(reg) data_ptr_range.start => _,
      data_end_ptr = in(reg) data_ptr_range.end,
      tmp = out(reg) _
    );
    let predicted_iter0 = cycles::LOAD_BYTE_INITIAL + cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter1 =
        cycles::LOAD_BYTE_SUBSEQUENT + cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::LOAD_BYTE_SUBSEQUENT + cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
    };
    (name, predicted, cycles)
}

/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 1-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align1(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken with LW align(1)", unsafe {
        &mut DATA_U32_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken with LW align(2)", unsafe {
        &mut DATA_U32_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 4-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken with LW align(4)", unsafe {
        &mut DATA_U32_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(iters, "BNE BACK usually taken with LW align(8)", unsafe {
        &mut DATA_U32_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_lw]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_lw_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_lw(
        iters,
        "BNE BACK usually taken with LW align(256) ",
        unsafe { &mut DATA_U32_ALIGN256.data },
    )
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
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      "lw {tmp}, 0({data_ptr})",
      "add {data_ptr}, {data_ptr}, {tmp}",
      "bne {data_ptr}, {data_end_ptr}, 1b",
      data_ptr = inout(reg) data_ptr_range.start => _,
      data_end_ptr = in(reg) data_ptr_range.end,
      tmp = out(reg) _
    );
    let predicted_iter0 = cycles::LOAD_WORD + cycles::ADD + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter1 = cycles::LOAD_WORD + cycles::ADD + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest =
        cycles::LOAD_WORD + cycles::ADD + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
    };
    (name, predicted, cycles)
}

/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 1-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align1(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken with SB align(1)", unsafe {
        &DATA_U8_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken with SB align(2)", unsafe {
        &DATA_U8_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 4-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken with SB align(4)", unsafe {
        &DATA_U8_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(iters, "BNE BACK usually taken with SB align(8)", unsafe {
        &DATA_U8_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_sb]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sb_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sb(
        iters,
        "BNE BACK usually taken with SB align(256) ",
        unsafe { &DATA_U8_ALIGN256.data },
    )
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
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      // This seems to introduce a data dependency/hazard that ensures more consistent benchmark
      // results regardless of alignment/location of the data.
      "xor {tmp}, {tmp}, zero",
      "sb {tmp}, 0({data_ptr})",
      // without these nop every fourth `sb` takes more than one cycle and the loop becomes less
      // predictable.
      "nop",
      "nop",
      "addi {data_ptr}, {data_ptr}, 1",
      "bne {data_ptr}, {data_end_ptr}, 1b",
      data_ptr = inout(reg) data_ptr_range.start => _,
      data_end_ptr = in(reg) data_ptr_range.end,
      tmp = inout(reg) 0 => _
    );
    let predicted_iter0 = cycles::STORE_INITIAL
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter1 = cycles::STORE_SUBSEQUENT
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::STORE_SUBSEQUENT
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
    };
    (name, predicted, cycles)
}

/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 1-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align1(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken with SW align(1)", unsafe {
        &DATA_U32_ALIGN1.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 2-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align2(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken with SW align(2)", unsafe {
        &DATA_U32_ALIGN2.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 4-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align4(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken with SW align(4)", unsafe {
        &DATA_U32_ALIGN4.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 8-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align8(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(iters, "BNE BACK usually taken with SW align(8)", unsafe {
        &DATA_U32_ALIGN8.data
    })
}
/// See [_branch_back_usually_taken_w_sw]. This runs the benchmark with 256-byte aligned data.
#[ram]
fn branch_back_usually_taken_w_sw_align256(iters: u32) -> (&'static str, u32, u32) {
    _branch_back_usually_taken_w_sw(
        iters,
        "BNE BACK usually taken with SW align(256) ",
        unsafe { &DATA_U32_ALIGN256.data },
    )
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
    let cycles = asm_with_perf_counter!(
      ".align 2",
      "1:",
      // This seems to introduce a data dependency/hazard that ensures more consistent benchmark
      // results regardless of alignment/location of the data.
      "xor {tmp}, {tmp}, zero",
      "sw zero, 0({data_ptr})",
      // without these nop every fourth `sw` takes more than one cycle and the loop becomes less
      // predictable.
      "nop",
      "nop",
      "addi {data_ptr}, {data_ptr}, 4",
      "bne {data_ptr}, {data_end_ptr}, 1b",
      data_ptr = inout(reg) data_ptr_range.start => _,
      data_end_ptr = in(reg) data_ptr_range.end,
      tmp = inout(reg) 0 => _
    );
    let predicted_iter0 = cycles::STORE_INITIAL
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::BRANCH_BACK_NOT_TAKEN;
    let predicted_iter1 = cycles::STORE_SUBSEQUENT
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::BRANCH_BACK_TAKEN_INITIAL;
    let predicted_iter_rest = cycles::STORE_SUBSEQUENT
        + cycles::XOR
        + cycles::NOP
        + cycles::NOP
        + cycles::ADD
        + cycles::BRANCH_BACK_TAKEN_SUBSEQUENT;
    let predicted = match iters {
        0 => 0,
        1 => predicted_iter0,
        2 => predicted_iter0 + predicted_iter1,
        3.. => predicted_iter0 + predicted_iter1 + (predicted_iter_rest) * (iters - 2),
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
