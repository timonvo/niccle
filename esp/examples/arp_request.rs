//! This example binary uses the `niccle_esp::eth_phy` module to transmit a hardcoded ARP request
//! and receive any incoming (response) packets. Incoming packets are handled by the
//! [niccle::eth_mac] crate, which currently validates them and prints some diagnostics.
//! Make sure the device can reach a peer at IP 169.254.172.115!
//!
//! It is currently only tested on ESP32-C6 chips.

#![no_std]
#![no_main]

#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use esp_backtrace as _;
use hal::{
    clock::ClockControl, peripheral::Peripheral, peripherals::Peripherals, prelude::*,
    timer::TimerGroup, Delay, IO,
};
use log::{debug, info};
use niccle::{debug_util::FormatEthernetFrame, eth_mac};
use niccle_esp::eth_phy;

static ETH_INTERRUPT_HANDLER: eth_phy::InterruptHandler<
    hal::timer::Timer0<hal::peripherals::TIMG0>,
    hal::gpio::Gpio5<hal::gpio::Unknown>,
    hal::gpio::Gpio6<hal::gpio::Unknown>,
    hal::gpio::Gpio10<hal::gpio::Unknown>,
> = eth_phy::InterruptHandler::new();

const MAC_RX_BUFFER_CAPACITY: usize = 10;
static ETH_MAC_RX: eth_mac::MacRx<MAC_RX_BUFFER_CAPACITY> =
    eth_mac::MacRx::<MAC_RX_BUFFER_CAPACITY>::new();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    // Use the maximum available clock speed, since we want to maximize the GPIO speed we can
    // achieve.
    let clocks = ClockControl::max(system.clock_control).freeze();
    // Set up the logger.
    esp_println::logger::init_logger_from_env();
    info!("Booted up!");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    // Configure the Ethernet Phy instance, linking it with the static ETH_INTERRUPT_HANDLER and
    // ETH_MAC_RX we defined above, and making it use GPIO5 for TX, GPIO6 for RX and GPIO10 for the
    // RX debug output signal.
    let eth_phy = eth_phy::Phy::new(eth_phy::PhyConfig {
        clocks: &clocks,
        interrupt_handler: &ETH_INTERRUPT_HANDLER,
        timer: timg0.timer0,
        tx_pin: io.pins.gpio5.into_ref(),
        rx_pin: io.pins.gpio6.into_ref(),
        // Note that we invert the RX input signal, because we assume that the electrical circuit
        // used leaves the signal high when idle. See
        // https://ctrlsrc.io/posts/2023/niccle-ethernet-circuit-design/#inverted-rx-signals for why
        // this is the case.
        rx_invert_signal: true,
        rx_mac_callback: &ETH_MAC_RX,
        rx_debug_pin: io.pins.gpio10.into_ref(),
    })
    .unwrap();

    // Create a MAC TX instance that uses the Phy instance we configured above.
    let mut eth_mac_tx = eth_mac::MacTx::new(&eth_phy);

    configure_interrupts();

    let mut delay = Delay::new(&clocks);

    // Wait for enough time to allow enough LTPs to be emitted by the interrupt handler for the
    // receiver to consider the link to be up. According to Clause 14.2.1.7 "Link Integrity Test
    // function requirements" receivers must consider a link to be "up" after they've received at
    // least 2 and at most 10 LTPs. So waiting for 200ms should be sufficient, enough time for at
    // least 12 LTPs to have been sent.
    delay.delay_ms(200u32);

    let test_frame = &niccle::example_data::TEST_FRAME_ARP_REQUEST_RAW;

    // Print the packet contents to see what we're about to send out on the wire. Note that this is
    // still the unencoded representation of the data, but that it does include the preamble and SFD
    // already.
    info!("------ Outgoing frame contents, in network bit order:");
    niccle::debug_util::log_data_binary_hex(log::Level::Info, test_frame);

    loop {
        // Print some stats about our progress so far.
        niccle_esp::debug_util::log_phy_mac_stats(&eth_phy.stats(), &ETH_MAC_RX.stats());

        // Transmit the packet (note that the MacTx will pad the frame and calculate its FCS, while
        // the Phy will Manchester-encode it as it is transmitted).
        eth_mac_tx.transmit_frame(test_frame);

        delay.delay_ms(2000u32);

        // Consume any frames that may have arrived since we last checked from the MacRx buffer.
        while let Some(mut frame) = ETH_MAC_RX.receive() {
            debug!("<<< Consumed RX {}", FormatEthernetFrame(frame.data()));
        }
    }
}

#[cfg(not(feature = "direct-vectoring"))]
fn configure_interrupts() {
    // Note that without the "direct-vectoring" feature this example binary doesn't actually work
    // correctly. See the note on the `GPIO`` interrupt function below for more info.
    hal::interrupt::enable(
        hal::peripherals::Interrupt::TG0_T0_LEVEL,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();
}

#[cfg(feature = "direct-vectoring")]
fn configure_interrupts() {
    unsafe {
        // Enable the interrupt routines for the timer and incoming packets (see cpu_int_30_handler
        // and cpu_int_31_handler below).
        {
            hal::interrupt::enable(
                hal::peripherals::Interrupt::TG0_T0_LEVEL,
                hal::interrupt::Priority::Priority1,
                hal::interrupt::CpuInterrupt::Interrupt30,
            )
            .unwrap();
            hal::interrupt::enable(
                hal::peripherals::Interrupt::GPIO,
                hal::interrupt::Priority::Priority1,
                hal::interrupt::CpuInterrupt::Interrupt31,
            )
            .unwrap();
        }
    }
}

// This interrupt will fire every 16ms. We need to forward the interrupt calls to the handler.
#[cfg(not(feature = "direct-vectoring"))]
#[interrupt]
fn TG0_T0_LEVEL() {
    ETH_INTERRUPT_HANDLER.on_timer_interrupt();
}

// This interrupt will fire every 16ms. We need to forward the interrupt calls to the handler.
#[cfg(feature = "direct-vectoring")]
#[no_mangle]
#[ram]
fn cpu_int_30_handler() {
    ETH_INTERRUPT_HANDLER.on_timer_interrupt();
}

// This interrupt will fire when a falling edge is detected on the GPIO RX pin.
//
// Without the "direct-vectoring" feature, it takes on the order of 7100ns from when the first
// signal edge arrives to invoke this interrupt routine. This is too late, as by then all 6400ns of
// preamble signal will have already passed by, and the interrupt handler won't be able to observe
// any of it. Hence, this configuration isn't actually functional.
//
// This code is left in place merely to facilitate comparing the latency of the interrupt
// dispatching implementations.
#[cfg(not(feature = "direct-vectoring"))]
#[interrupt]
fn GPIO() {
    ETH_INTERRUPT_HANDLER.on_rx_interrupt();
}

// This interrupt will fire when a falling edge is detected on the GPIO RX pin.
//
// With the "direct-vectoring" feature, it takes on the order of 750ns from when the first signal
// edge arrives to invoke this interrupt routine. This leave almost 5600ns of the preamble signal to
// be observed by the interrupt handler.
#[cfg(feature = "direct-vectoring")]
#[no_mangle]
#[ram]
fn cpu_int_31_handler() {
    ETH_INTERRUPT_HANDLER.on_rx_interrupt();
}
