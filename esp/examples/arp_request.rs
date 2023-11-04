//! This example binary uses the `niccle_esp::eth_phy` module to transmit a hardcoded ARP request.
//! It is currently still a work in progress, and is only tested on ESP32-C6 chips.

#![no_std]
#![no_main]

#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use esp_backtrace as _;
use hal::{
    clock::ClockControl, peripheral::Peripheral, peripherals::Peripherals, prelude::*,
    timer::TimerGroup, Delay, IO,
};
use log::info;
use niccle_esp::eth_phy;

static ETH_INTERRUPT_HANDLER: eth_phy::InterruptHandler<
    hal::timer::Timer0<hal::peripherals::TIMG0>,
    hal::gpio::Gpio5<hal::gpio::Unknown>,
> = eth_phy::InterruptHandler::new();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    // Use the maximum available clock speed, since we want to maximize the GPIO speed we can
    // achieve.
    let clocks = ClockControl::max(system.clock_control).freeze();
    // Set up the logger.
    esp_println::logger::init_logger_from_env();
    info!("Booted up!");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let timg0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    // Configure the Ethernet Phy instance, linking it with the static ETH_INTERRUPT_HANDLER we
    // defined above, and making it use GPIO5 for TX.
    let mut eth_phy = eth_phy::Phy::new(eth_phy::PhyConfig {
        clocks: &clocks,
        interrupt_handler: &ETH_INTERRUPT_HANDLER,
        timer: timg0.timer0,
        tx_pin: io.pins.gpio5.into_ref(),
    })
    .unwrap();

    // Enable the interrupt routine (see TG0_T0_LEVEL below) for the timer.
    hal::interrupt::enable(
        hal::peripherals::Interrupt::TG0_T0_LEVEL,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();
    // Enable global interrupts.
    unsafe {
        hal::riscv::interrupt::enable();
    }

    let mut delay = Delay::new(&clocks);

    // Wait for enough time to allow enough LTPs to be emitted by the interrupt handler for the
    // receiver to consider the link to be up. According to Clause 14.2.1.7 "Link Integrity Test
    // function requirements" receivers must consider a link to be "up" after they've received at
    // least 2 and at most 10 LTPs. So waiting for 200ms should be sufficient, enough time for at
    // least 12 LTPs to have been sent.
    delay.delay_ms(200u32);

    // Construct the packet by prepending the preamble and SFD to the frame data.
    const PREAMBLE_W_SFD: [u8; 8] = [
        // Preamble
        0b01010101, 0b01010101, 0b01010101, 0b01010101, 0b01010101, 0b01010101, 0b01010101,
        // SFD
        0b11010101,
    ];
    let mut packet =
        [0u8; (PREAMBLE_W_SFD.len() + niccle::example_data::TEST_FRAME_ARP_REQUEST_RAW.len())];
    packet[0..PREAMBLE_W_SFD.len()].copy_from_slice(&PREAMBLE_W_SFD);
    packet[PREAMBLE_W_SFD.len()..]
        .copy_from_slice(niccle::example_data::TEST_FRAME_ARP_REQUEST_RAW);

    // Print the packet contents to see what we're about to send out on the wire. Note that this is
    // still the unencoded representation of the data.
    info!("Packet contents:");
    info!("----------------");
    for (i, val) in packet.iter().enumerate() {
        info!("{i:08}: {val:08b}")
    }

    loop {
        info!("LTPs sent: {}", eth_phy.stats().ltps_sent);

        // Transmit the packet (note that the Phy will Manchester-encode it as it is transmitted).
        eth_phy.transmit_packet(&packet);
        info!("Packets sent: {}", eth_phy.stats().packets_sent);

        delay.delay_ms(2000u32);
    }
}

// This interrupt will fire every 16ms. We need to forward the interrupt calls to the handler.
#[interrupt]
fn TG0_T0_LEVEL() {
    ETH_INTERRUPT_HANDLER.on_timer_interrupt();
}
