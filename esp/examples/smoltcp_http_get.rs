//! This example binary uses the `niccle_esp::eth_phy` and `niccle::eth_mac` modules and the
//! `smoltcp` crate to perform a series of `HTTP GET` requests. Make sure you have an HTTP server
//! running at 169.254.172.115 that can serve the files in the `testdata/` directory.
//!
//! If you use the ./serve_testdata.py script to serve the files, then the responses the client
//! receives will be deterministic, and the CRC32 for each response for each test file can be
//! printed out with the ./calculate_crc32.py script and compared against the output of the binary.
//!
//! It is currently only tested on ESP32-C6 chips.

#![no_std]
#![no_main]
#![feature(ascii_char)]

#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use crc32fast::Hasher;
use esp_backtrace as _;
use hal::{
    clock::ClockControl, peripheral::Peripheral, peripherals::Peripherals, prelude::*,
    timer::TimerGroup, Delay, Rtc, IO,
};
use log::debug;
use log::info;
use niccle::eth_mac;
use niccle::eth_mac::MAX_PACKET_SIZE;
use niccle_esp::eth_phy;
use smoltcp::iface::Config;
use smoltcp::iface::SocketStorage;
use smoltcp::iface::{Interface, SocketSet};
use smoltcp::socket::tcp;
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};

// Manual tuning indicates it's best if the TCP RX buffer is smaller than the MAC RX buffer. I
// reason that this helps ensure that the SmolTCP layer will start indicating a full receive window
// to the sender *before* the MAC RX buffer is actually full. That way we have a chance of
// indicating the full TCP receive window to the sender without dropping any/too many of the
// sender's already-in-flight packets.
//
// A MAC RX buffer capacity of about 9 packets with a TCP RX buffer capacity of 7 TCP packets seems
// to be the sweet spot, reaching a max throughput speed of about 645kB/sec. Larger buffer sizes
// don't seem to increase throughput any further.
const RX_PACKET_CAPACITY: usize = 9;
const TCP_RX_BUFFER_CAPACITY: usize = (RX_PACKET_CAPACITY - 2) * 1460;
// The bbqueue uses up to 8 bytes of framing metadata per entry, hence why we increase the max
// packet size by 8 here.
//
// TODO: Clean this up so that such implementation details don't have to be exposed to the caller.
const MAC_RX_BUFFER_CAPACITY: usize = RX_PACKET_CAPACITY * (MAX_PACKET_SIZE + 8);

static ETH_INTERRUPT_HANDLER: eth_phy::InterruptHandler<
    hal::timer::Timer0<hal::peripherals::TIMG0>,
    hal::gpio::Gpio5<hal::gpio::Unknown>,
    hal::gpio::Gpio6<hal::gpio::Unknown>,
    hal::gpio::Gpio10<hal::gpio::Unknown>,
    eth_mac::MacRxPacketProducer<'static, MAC_RX_BUFFER_CAPACITY>,
> = eth_phy::InterruptHandler::new();

static ETH_MAC_RX: eth_mac::MacRx<MAC_RX_BUFFER_CAPACITY> =
    eth_mac::MacRx::<MAC_RX_BUFFER_CAPACITY>::new(&|| {
        // When the MAC layer tells us that the RX queue has capacity again we need to forward that
        // information to the interrupt handler so that it can re-enable the RX interrupt and start
        // listening for incoming packets again.
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

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);

    // Configure the Ethernet Phy instance, linking it with the static ETH_INTERRUPT_HANDLER and
    // ETH_MAC_RX we defined above, and making it use GPIO5 for TX, GPIO6 for RX and GPIO10 for the
    // RX debug output signal.
    let (mac_rx_producer, mac_rx_consumer) = ETH_MAC_RX.split();
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
    info!("Created MAC instance!");
    let rtc = Rtc::new(peripherals.LP_CLKRST);
    let mut iface = configure_iface(&mut smol_mac, &rtc);

    // Fetch a 10kB, 100kB, and 1MB, and 3MB file, issuing 5 requests per file size, then loop.
    let num_requests_per_file = 5;
    loop {
        for path in ["/10kB.txt", "/100kB.txt", "/1MB.txt", "/3MB.txt"] {
            info!("=== Issuing {num_requests_per_file} HTTP GET requests for {path}");
            for _ in 0..num_requests_per_file {
                do_http_request(
                    Ipv4Address::new(169, 254, 172, 115),
                    8080,
                    path,
                    &mut iface,
                    &mut smol_mac,
                    &rtc,
                );
            }
            // Print some stats about our progress so far.
            niccle_esp::debug_util::log_phy_mac_stats(&eth_phy.stats(), &ETH_MAC_RX.stats());
            info!("");
        }
        delay.delay_ms(2000u32);
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

/// Based on <https://github.com/smoltcp-rs/smoltcp/blob/main/examples/httpclient.rs>.
fn do_http_request<'a, P: eth_mac::PhyTx<'a>, const BUFFER_CAP: usize>(
    destination: Ipv4Address,
    port: u16,
    request_path: &str,
    iface: &mut Interface,
    smol_mac: &mut eth_mac::smoltcp::SmolMac<'a, P, BUFFER_CAP>,
    rtc: &Rtc,
) {
    // Create sockets
    let mut tcp_rx_buffer_raw = [0; TCP_RX_BUFFER_CAPACITY];
    let mut tcp_tx_buffer_raw = [0; 10240];
    let tcp_rx_buffer = tcp::SocketBuffer::new(&mut tcp_rx_buffer_raw[..]);
    let tcp_tx_buffer = tcp::SocketBuffer::new(&mut tcp_tx_buffer_raw[..]);
    let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);

    let mut socket_storage_raw = [SocketStorage::EMPTY];
    let mut sockets = SocketSet::new(&mut socket_storage_raw[..]);
    let tcp_handle = sockets.add(tcp_socket);

    enum State {
        Connect,
        Request,
        Response,
    }
    let mut state = State::Connect;
    let mut bytes_received = 0usize;

    let start_time = instant_now(rtc);
    let mut response_hasher = Hasher::new();
    loop {
        iface.poll(instant_now(rtc), smol_mac, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(tcp_handle);
        let cx = iface.context();

        state = match state {
            State::Connect if !socket.is_active() => {
                debug!("Connecting socket");
                // Pick a random local port.
                let local_port = 49152 + (rtc.get_time_ms() % 10000) as u16;
                socket.connect(cx, (destination, port), local_port).unwrap();
                State::Request
            }
            State::Request if socket.may_send() => {
                debug!("sending request");
                socket.send_slice(b"GET ").unwrap();
                socket.send_slice(request_path.as_bytes()).unwrap();
                socket.send_slice(b" HTTP/1.1\r\n").unwrap();
                socket.send_slice(b"Host: localhost\r\n").unwrap();
                socket.send_slice(b"Connection: close\r\n").unwrap();
                socket.send_slice(b"\r\n").unwrap();
                State::Response
            }
            State::Response if socket.can_recv() => {
                socket
                    .recv(|data| {
                        bytes_received += data.len();
                        response_hasher.update(data);
                        debug!("Received data: {:?}", data.as_ascii().unwrap());
                        (data.len(), ())
                    })
                    .unwrap();
                State::Response
            }
            State::Response if !socket.may_recv() => {
                debug!("received complete response");
                break;
            }
            _ => state,
        };
    }

    let transfer_duration = instant_now(rtc) - start_time;
    let response_hash = response_hasher.finalize();
    // See the note at the top of this file about comparing the CRC32 output here against expected
    // values. This allows us to confirm that the received data is actually valid.
    info!(
        "Received {bytes_received} bytes over {transfer_duration} ({:.1}kB/sec) \
        with CRC32 {response_hash}",
        (bytes_received as f64 / transfer_duration.total_millis() as f64)
    )
}

/// Returns an Instant based on the Rtc clock.
fn instant_now(rtc: &Rtc) -> Instant {
    Instant::from_micros(rtc.get_time_us() as i64)
}
