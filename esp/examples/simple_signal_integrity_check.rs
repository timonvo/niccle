//! This example binary performs a very basic signal integrity check, by emitting a link test pulse
//! every ~16ms, as well emitting a waveform that mimics a packet transmission.
//!
//! As long as the circuit used for transmitting the signal is functional, then the link test pulse
//! will make other devices think that a link has been established.
//!
//! The simulated packet signal can be inspected with an oscilloscope to validate that the circuit
//! passes the waveform correctly. E.g. it can be used to check whether the signal isn't attenuated
//! too strongly overall, whether the 5MHz and 10MHz frequencies aren't filtered too strongly
//! specifically, whether the TP_IDL signal isn't filtered too strongly, and whether the
//! differential voltage level returns to 0V sufficiently quickly after the line is left idle after
//! a transmission.

#![no_std]
#![no_main]
// We use this feature to write inline assembly with readable register names.
#![feature(asm_const)]

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use core::arch::asm;
use esp_backtrace as _;
use hal::{clock::ClockControl, gpio::IO, peripherals::Peripherals, prelude::*, Delay};
use log::info;

// This CSR isn't defined by the esp-rs/esp-hal framework yet (nor by the espressif/svd files), so
// we hardcode their addresses. See the "1.14 Dedicated IO" chapter of the technical reference
// manual.
const CSR_CPU_GPIO_OUT: u32 = 0x805;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    // Use the maximum available clock speed, since we want to maximize the GPIO speed we can
    // achieve.
    let clocks = ClockControl::max(system.clock_control).freeze();
    assert_eq!(clocks.cpu_clock.to_MHz(), 160);

    esp_println::logger::init_logger_from_env();
    info!("Booted up!");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // Set GPIO5 as an output. As per ESP-IDF's docs, we must do this even though we'll access them
    // via the dedicated GPIO feature.
    let mut gpio5 = io.pins.gpio5.into_push_pull_output();

    // Enable dedicated GPIO for GPIO5.
    gpio5.connect_peripheral_to_output_with_options(
        #[cfg(feature = "esp32c3")]
        hal::gpio::OutputSignal::CPU_GPIO_0,
        #[cfg(feature = "esp32c6")]
        hal::gpio::OutputSignal::CPU_GPIO_OUT0,
        false,
        false,
        true,
        false,
    );

    let mut delay = Delay::new(&clocks);
    loop {
        do_signal_check(&mut delay);
    }
}

#[ram]
fn do_signal_check(delay: &mut Delay) {
    unsafe {
        emit_ltp();
    }
    delay.delay_ms(8u32);
    unsafe {
        emit_simulated_packet();
    }
    delay.delay_ms(8u32);
}

/// Emits a link test pulse (LTP).
#[ram]
unsafe fn emit_ltp() {
    asm!(
        // Set the output high, and keep it high for 62.5ns (10 CPU cycles).
        "csrrsi zero, {csr_cpu_gpio_out}, {gpio_tx}", // 1 cycle
        // 9 cycles
        "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        // Set the output low.
        "csrrci zero, {csr_cpu_gpio_out}, {gpio_tx}",
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        // Use CPU output signal 0.
        gpio_tx = const(1 << 0),
    );
}

/// Emits a waveform that mimics an Ethernet packet transmission, without it actually containing any
/// valid data. It consists of a waveform that is similar to the preamble + SFD, a waveform that
/// simulates a payload, and a TP_IDL-like waveform.
#[ram]
unsafe fn emit_simulated_packet() {
    asm!(
        // --- START OF MACRO DEFINITIONS.
        // A macro that toggles a pin high for 100ns and low for 100ns, resulting in a 5MHz waveform
        // if repeated.
        ".macro toggle_pin_5MHz",
        // Set the output high, and keep it high for 100ns (16 CPU cycles).
        "csrrsi zero, {csr_cpu_gpio_out}, {gpio_tx}", // 1 cycle.
        // 15 cycles
        "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        // Set the output low, and keep it low for 100ns.
        "csrrci zero, {csr_cpu_gpio_out}, {gpio_tx}", // 1 cycle.
        // 15 cycles
        "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        ".endm",
        //
        //
        // A macro that generates a 5MHz waveform that is 8 bit times long (800ns).
        ".macro toggle_pin_5MHz_for_800ns",
        // Since `toggle_pin_5MHz` takes 200ns, repeating it four times produces a 5MHz waveform
        // that is 800ns long.
        "toggle_pin_5MHz",
        "toggle_pin_5MHz",
        "toggle_pin_5MHz",
        "toggle_pin_5MHz",
        ".endm",
        //
        //
        // A macro that toggles a pin high for 50ns and low for 50ns, resulting in a 10MHz waveform
        // if repeated.
        ".macro toggle_pin_10MHz",
        // Set the output high, and keep it high for 50ns (8 CPU cycles).
        "csrrsi zero, {csr_cpu_gpio_out}, {gpio_tx}", // 1 cycle.
        // 7 cycles
        "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        // Set the output low, and keep it low for 50ns.
        "csrrci zero, {csr_cpu_gpio_out}, {gpio_tx}", // 1 cycle.
        // 7 cycles
        "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        ".endm",
        //
        //
        // A macro that generates a waveform with an alternating frequency of 10MHz and 5MHz.
        ".macro toggle_pin_alternating_freq",
        "toggle_pin_10MHz",
        "toggle_pin_10MHz",
        "toggle_pin_5MHz",
        "toggle_pin_5MHz",
        ".endm",
        // --- END OF MACRO DEFINITIONS.
        //
        // --- SIMULATED PREAMBLE:
        // Simulate a preamble and SFD by sending a 5MHz waveform for 8 * 800ns long.
        "toggle_pin_5MHz_for_800ns",
        "toggle_pin_5MHz_for_800ns",
        "toggle_pin_5MHz_for_800ns",
        "toggle_pin_5MHz_for_800ns",
        "toggle_pin_5MHz_for_800ns",
        "toggle_pin_5MHz_for_800ns",
        "toggle_pin_5MHz_for_800ns",
        "toggle_pin_5MHz_for_800ns",
        //
        // --- SIMULATED PAYLOAD:
        // Simulate a payload transmission consisting of alternating 10MHz and 5MHz waveforms.
        "toggle_pin_alternating_freq",
        "toggle_pin_alternating_freq",
        "toggle_pin_alternating_freq",
        "toggle_pin_alternating_freq",
        //
        // --- SIMULATED TP_IDL:
        // Send a TP_IDL signal by setting the pin high and keeping it high 250ns (40 cycles), then
        // setting it low.
        "csrrsi zero, {csr_cpu_gpio_out}, {gpio_tx}", // 1 cycle.
        // 39 cycles
        "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
        // Set the output low.
        "csrrci zero, {csr_cpu_gpio_out}, {gpio_tx}",
        csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
        // Use CPU output signal 0.
        gpio_tx = const(1 << 0),
    );
}
