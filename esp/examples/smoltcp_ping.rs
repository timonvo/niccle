//! This example binary uses the `niccle_esp::eth_phy` and `niccle::eth_mac` modules and the
//! `smoltcp` crate to perform a series of `ping` requests. Make sure the device can reach a
//! peer at IP 169.254.172.115!
//!
//! It is currently only tested on ESP32-C6 chips.

#![no_std]
#![no_main]

#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use byteorder::{ByteOrder, NetworkEndian};
use esp_backtrace as _;
use hal::{
    clock::ClockControl, peripheral::Peripheral, peripherals::Peripherals, prelude::*,
    timer::TimerGroup, Delay, Rtc, IO,
};
use heapless::FnvIndexMap;
use log::info;
use niccle::eth_mac;
use niccle::eth_mac::MAX_PACKET_SIZE;
use niccle_esp::eth_phy;
use smoltcp::iface::Config;
use smoltcp::iface::SocketStorage;
use smoltcp::iface::{Interface, SocketSet};
use smoltcp::phy::Device;
use smoltcp::socket::icmp;
use smoltcp::time::{Duration, Instant};
use smoltcp::wire::{EthernetAddress, Icmpv4Packet, Icmpv4Repr, IpAddress, IpCidr, Ipv4Address};

// The buffer size isn't super critical for this example. Enough space for a few packets should be
// sufficient.
const MAC_RX_BUFFER_CAPACITY: usize = 5 * MAX_PACKET_SIZE;

static ETH_INTERRUPT_HANDLER: eth_phy::InterruptHandler<
    hal::timer::Timer0<hal::peripherals::TIMG0>,
    hal::gpio::Gpio5<hal::gpio::Unknown>,
    hal::gpio::Gpio6<hal::gpio::Unknown>,
    hal::gpio::Gpio10<hal::gpio::Unknown>,
    eth_mac::MacRxPacketProducer<'static, MAC_RX_BUFFER_CAPACITY>,
> = eth_phy::InterruptHandler::new();

static ETH_MAC_RX: eth_mac::MacRx<MAC_RX_BUFFER_CAPACITY> =
    eth_mac::MacRx::<MAC_RX_BUFFER_CAPACITY>::new(&|| {
        ETH_INTERRUPT_HANDLER.rx_buffer_space_available();
    });

// This interrupt will fire every 16ms. We need to forward the interrupt calls to the handler.
#[no_mangle]
#[ram]
fn cpu_int_30_handler() {
    ETH_INTERRUPT_HANDLER.on_timer_interrupt();
}

// This interrupt will fire when a falling edge is detected on the GPIO RX pin.
//
// With the "direct-vectoring" feature, it takes on the order of 750ns from when the first signal
// edge arrives to invoke this interrupt routine. This leave almost 5600ns of the preamble signal to
// be observed by the interrupt handler.
#[no_mangle]
#[ram]
fn cpu_int_31_handler() {
    ETH_INTERRUPT_HANDLER.on_rx_interrupt();
}

fn configure_interrupts() {
    unsafe {
        // Enable the interrupt routines for the timer and incoming packets (see cpu_int_30_handler
        // and cpu_int_31_handler above).
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

    let (mac_rx_producer, mac_rx_consumer) = ETH_MAC_RX.split();

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
        rx_mac_callback: mac_rx_producer,
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
    delay.delay_ms(1000u32);

    // Set up an interface connected to the provided SmolMac. We use a hardcoded MAC and IP address,
    // and assume that the device is directly connected to the interface, and hence no routing
    // information is needed.
    let mut smol_mac = eth_mac::smoltcp::SmolMac::new(&mut eth_mac_tx, mac_rx_consumer);
    let rtc = Rtc::new(peripherals.LP_CLKRST);
    let mut iface = configure_iface(&mut smol_mac, &rtc);

    // Perform a sequence of ping requests every second.
    loop {
        do_ping(
            10,
            Ipv4Address::new(169, 254, 172, 115),
            &mut iface,
            &mut smol_mac,
            &rtc,
        );

        // Print some stats about our progress so far.
        niccle_esp::debug_util::log_phy_mac_stats(&eth_phy.stats(), &ETH_MAC_RX.stats());

        delay.delay_ms(1000u32);
    }
}

fn configure_iface<'a, P: eth_mac::PhyTx<'a>, const BUFFER_CAP: usize>(
    smol_mac: &mut eth_mac::smoltcp::SmolMac<'a, P, BUFFER_CAP>,
    rtc: &Rtc<'_>,
) -> Interface {
    let mut iface = Interface::new(
        {
            let mut config =
                Config::new(EthernetAddress([0x12, 0x34, 0x56, 0x78, 0x90, 0x12]).into());
            config.random_seed = rtc.get_time_us();
            config
        },
        smol_mac,
        instant_now(rtc),
    );
    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(IpCidr::new(IpAddress::v4(169, 254, 172, 114), 24))
            .unwrap();
    });
    iface
}

/// The maximum number of pending ping responses that [do_ping] can support. Must be a power of two.
const MAX_COUNT: usize = 32;
/// The amount of time we should leave between sending each ping EchoRequest.
const PING_INTERVAL: Duration = Duration::from_millis(100);
/// The amount of time we should wait for an EchoResponse for a given EchoRequest.
const PING_TIMEOUT: Duration = Duration::from_millis(250);

/// Based on <https://github.com/smoltcp-rs/smoltcp/blob/main/examples/ping.rs>.
fn do_ping<'a, P: eth_mac::PhyTx<'a>, const BUFFER_CAP: usize>(
    count: usize,
    destination: Ipv4Address,
    iface: &mut Interface,
    smol_mac: &mut eth_mac::smoltcp::SmolMac<'a, P, BUFFER_CAP>,
    rtc: &Rtc,
) {
    let checksum_capabilities = smol_mac.capabilities().checksum;

    // Create sockets.
    let mut icmp_rx_buffers_raw = ([icmp::PacketMetadata::EMPTY], [0; 256]);
    let mut icmp_tx_buffers_raw = ([icmp::PacketMetadata::EMPTY], [0; 256]);
    let icmp_rx_buffer = icmp::PacketBuffer::new(
        &mut icmp_rx_buffers_raw.0[..],
        &mut icmp_rx_buffers_raw.1[..],
    );
    let icmp_tx_buffer = icmp::PacketBuffer::new(
        &mut icmp_tx_buffers_raw.0[..],
        &mut icmp_tx_buffers_raw.1[..],
    );
    let icmp_socket = icmp::Socket::new(icmp_rx_buffer, icmp_tx_buffer);
    let mut socket_storage_raw = [SocketStorage::EMPTY];
    let mut sockets = SocketSet::new(&mut socket_storage_raw[..]);
    let icmp_handle = sockets.add(icmp_socket);

    let mut send_at = Instant::from_millis(0);
    let mut seq_no = 0;
    let mut received = 0;
    let mut echo_payload = [0xffu8; 40];
    let mut waiting_queue = FnvIndexMap::<_, _, MAX_COUNT>::new();
    let ident = 0x22b;

    loop {
        // Poll the interface. This call is what actually caused queued-up packets to be sent, and
        // incoming packets to be received.
        iface.poll(instant_now(rtc), smol_mac, &mut sockets);

        let timestamp = instant_now(rtc);
        let socket = sockets.get_mut::<icmp::Socket>(icmp_handle);
        if !socket.is_open() {
            socket.bind(icmp::Endpoint::Ident(ident)).unwrap();
            send_at = timestamp;
        }

        // Emit a EchoRequest packet every PING_INTERVAL.
        if socket.can_send() && seq_no < count as u16 && send_at <= timestamp {
            NetworkEndian::write_i64(&mut echo_payload, timestamp.total_micros());

            let icmp_repr = Icmpv4Repr::EchoRequest {
                ident,
                seq_no,
                data: &echo_payload,
            };
            let icmp_payload = socket
                .send(icmp_repr.buffer_len(), destination.into())
                .expect("Send buffer unexpectedly full");
            let mut icmp_packet = Icmpv4Packet::new_unchecked(icmp_payload);
            icmp_repr.emit(&mut icmp_packet, &checksum_capabilities);

            waiting_queue
                .insert(seq_no, timestamp)
                .expect("Queue unexpectedly full");
            seq_no += 1;
            send_at += PING_INTERVAL;
        }

        // Recieve incoming packets, and print a line if the packet is an EchoResponse.
        if socket.can_recv() {
            let (payload, _) = socket.recv().expect("Unexpected send error");

            let icmp_packet = Icmpv4Packet::new_checked(&payload).unwrap();
            let icmp_repr = Icmpv4Repr::parse(&icmp_packet, &checksum_capabilities).unwrap();
            if let Icmpv4Repr::EchoReply { seq_no, data, .. } = icmp_repr {
                if waiting_queue.get(&seq_no).is_some() {
                    let packet_timestamp_us = NetworkEndian::read_i64(data);
                    info!(
                        "{} bytes from {}: icmp_seq={}, time={:.1}ms",
                        data.len(),
                        destination,
                        seq_no,
                        (timestamp.total_micros() - packet_timestamp_us) as f64 / 1000.0
                    );
                    waiting_queue.remove(&seq_no);
                    received += 1;
                }
            }
        }

        // Update the queue of pending ping responses.
        waiting_queue.retain(|seq, from| {
            if timestamp - *from < PING_TIMEOUT {
                true
            } else {
                info!("From {destination} icmp_seq={seq} timeout");
                false
            }
        });

        // If we've sent `count` number of packets and received all responses, then break out of the
        // loop.
        if seq_no == count as u16 && waiting_queue.is_empty() {
            break;
        }
    }

    // Print overall stats before returning.
    info!("--- {destination} ping statistics ---");
    info!(
        "{} packets transmitted, {} received, {:.0}% packet loss",
        seq_no,
        received,
        100.0 * (seq_no - received) as f64 / seq_no as f64
    );
}

/// Returns an Instant based on the Rtc clock.
fn instant_now(rtc: &Rtc) -> Instant {
    Instant::from_micros(rtc.get_time_us() as i64)
}
