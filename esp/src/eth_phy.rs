//! Implements the functionality of an Ethernet 10BASE-T PHY, like emitting periodic test pulses,
//! Manchester-coding of incoming/outgoing data, etc.

use crate::eth_phy_dedicated_io;
use core::cell::RefCell;
use critical_section::{CriticalSection, Mutex};
use hal::{clock::Clocks, peripheral::PeripheralRef, prelude::*};
use niccle::eth_mac;

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

#[derive(Debug)]
pub enum Error {
    /// The CPU clock is misconfigured (it must be set to 160MHz).
    IncorrectCpuClockError,
    /// An attempt to use an [InterruptHandler] with more than on [Phy] instance was detected.
    InterruptHandlerAlreadyAttachedError,
}
pub type Result<T> = core::result::Result<T, Error>;

/// A configuration struct for use with the [Phy] constructor.
pub struct PhyConfig<'a, PTimer: 'static, PTx: 'static, PRx: 'static, PRxDebug: 'static> {
    /// The currently-configured device clocks, used to validate the clock speed.
    pub clocks: &'a Clocks<'a>,
    /// The [InterruptHandler] to attach the [Phy] to.
    pub interrupt_handler: &'static InterruptHandler<PTimer, PTx, PRx, PRxDebug>,
    /// The timer to use with the [Phy] instance, e.g. to periodically transmit link test pulses
    /// (LTPs).
    pub timer: hal::timer::Timer<PTimer>,
    /// The GPIO pin to use as the TX pin.
    pub tx_pin: PeripheralRef<'static, PTx>,
    /// The GPIO pin to use as the RX pin.
    pub rx_pin: PeripheralRef<'static, PRx>,
    /// Indicates whether the RX input signal should be inverted.
    pub rx_invert_signal: bool,
    /// The GPIO pin to use as the RX debug output pin, for monitoring the behavior and timing of
    /// the receive loop.
    // TODO: Make the use of this pin optional.
    pub rx_debug_pin: PeripheralRef<'static, PRxDebug>,
}

/// The main entry point to this module. Enables transmission of outgoing packets, and will be
/// extended to support receiving incoming packets in a later version.
pub struct Phy<PTimer: 'static, PTx: 'static, PRx: 'static, PRxDebug: 'static> {
    interrupt_handler: &'static InterruptHandler<PTimer, PTx, PRx, PRxDebug>,
}

impl<
        PTimer: hal::timer::Instance,
        PTx: hal::gpio::OutputPin,
        PRx: hal::gpio::InputPin,
        PRxDebug: hal::gpio::OutputPin,
    > Phy<PTimer, PTx, PRx, PRxDebug>
{
    /// Creates a new instance and attaches it to the given [InterruptHandler].
    ///
    /// A single InterruptHandler can only be used with one [Phy] instance, and an
    /// [Error::InterruptHandlerAlreadyAttachedError] will be returned if this constraint is
    /// violated.
    ///
    /// Interrupts will be scheduled using the provided timer, and on the provided RX pin. The
    /// timer's and RX pin's corresponding interrupt routines must be defined elsewhere and enabled
    /// by the caller after this method returns. The timer interrupt routine must call
    /// [InterruptHandler::on_timer_interrupt] when it fires, and the RX pin interrupt routine must
    /// call [InterruptHandler::on_rx_interrupt] when it fires.
    pub fn new(
        mut config: PhyConfig<PTimer, PTx, PRx, PRxDebug>,
    ) -> Result<Phy<PTimer, PTx, PRx, PRxDebug>> {
        if config.clocks.cpu_clock.to_MHz() != 160 {
            return Err(Error::IncorrectCpuClockError);
        }
        // Set the TX pin as output in the GPIO matrix, connect it to the CPU's dedicated GPIO
        // output signal TX_CPU_OUTPUT_SIGNAL, and ensure that its configured as an output in the
        // dedicated GPIO peripheral based on the GPIO matrix setting (rather than having to write
        // to the CPU_GPIO_OEN peripheral register to configure it as an output pin, see Table 7­.2
        // in the ESP32C6 Technical Reference Manual).
        config.tx_pin.set_to_push_pull_output();
        config.tx_pin.connect_peripheral_to_output_with_options(
            eth_phy_dedicated_io::TX_CPU_OUTPUT_SIGNAL,
            false,
            false,
            true,
            false,
        );

        // Set the RX pin as an input in the GPIO matrix, and connect it to the CPU's dedicated GPIO
        // input signal 7.
        config.rx_pin.set_to_input();
        config.rx_pin.connect_input_to_peripheral_with_options(
            eth_phy_dedicated_io::RX_CPU_INPUT_SIGNAL,
            config.rx_invert_signal,
            false,
        );

        // Set the RX debug output pin as an output and connect it to the appropriate dedicated IO
        // signal.
        config.rx_debug_pin.set_to_push_pull_output();
        config
            .rx_debug_pin
            .connect_peripheral_to_output_with_options(
                eth_phy_dedicated_io::RX_DEBUG_CPU_OUTPUT_SIGNAL,
                false,
                false,
                true,
                false,
            );

        // Attach to the InterruptHandler. This will schedule one or more interrupts.
        config.interrupt_handler.attach(InterruptSharedResources {
            timer: config.timer,
            tx_pin: config.tx_pin,
            rx_pin: config.rx_pin,
            rx_debug_pin: config.rx_debug_pin,
            stats: Default::default(),
        })?;
        Ok(Self {
            interrupt_handler: config.interrupt_handler,
        })
    }

    /// Returns stats about this instance's activity so far.
    pub fn stats(&self) -> TxRxStats {
        self.interrupt_handler
            .use_attached_resources(|resources, _| resources.stats)
    }

    /// Transmits the given Ethernet packet over the transmission line.
    ///
    /// Note that the packet must consist of the (unencoded) preamble, SFD and Ethernet frame data.
    /// I.e. the caller is more or less responsible for the MAC layer, and this function takes care
    /// of the PHY layer responsibilities like Manchester-encoding and TP_IDL emission at the end of
    /// the transmission.
    ///
    /// Data will be transmitted one byte at a time, with transmission going in LSB-to-MSB order.
    ///
    /// Note that for a maximum-length Ethernet packet of 1530 octets this method will take at least
    /// 1224us to transmit. For a minimum-length Ethernet packet of 72 octets it will take at least
    /// 57.85us.
    ///
    /// This method currently disables all interrupts during transmission (which means that any
    /// incoming packets will not trigger [InterruptHandler::on_rx_interrupt], and hence will be
    /// dropped), but that may be improved in the future.
    pub fn transmit_packet(&self, data: &[u8]) {
        self.interrupt_handler
            .use_attached_resources(|resources, _| {
                // Disable the LTP timer, since we we're about to transmit data and so no LTP will
                // be needed for another 16ms.
                resources.timer.set_alarm_active(false);
                // TODO: Avoid doing the transmission within this `use_attached_resources` call,
                // since that means we're disabling all interrupts during this potentially long
                // period of time, and that may not always be desirable. E.g. perhaps we should
                // allow transmissions to be interrupted, and re-transmit them post-hoc if and when
                // we detect such interrupts have occurred.
                eth_phy_dedicated_io::transmit_packet(data, &mut *resources.tx_pin);
                resources.stats.tx.packets_sent += 1;
                // Re-activate the timer alarm so we get triggered after the next period has passed.
                resources.timer.reset_counter();
                resources.timer.set_alarm_active(true);
            })
    }
}

/// Handles interrupts scheduled by the [Phy], and manages sharing of resources used by code running
/// in interrupt handlers and code running the main thread of execution.
///
/// To use this you generally will define static singleton of this type, which can then be safely
/// used from both the interrupt handling routine and the main thread of execution, since this class
/// is [Sync].
pub struct InterruptHandler<PTimer: 'static, PTx: 'static, PRx: 'static, PRxDebug: 'static> {
    shared_state: Mutex<RefCell<InterruptHandlerState<PTimer, PTx, PRx, PRxDebug>>>,

    // The buffer we use to receive incoming data.
    //
    // Note: this buffer is not a part of [InterruptHandler::shared_state] because it's only ever
    // accessed from within an interrupt routine. By not placing it within InterruptHandlerState we
    // also avoid that enum's variants having significantly differing sizes. We still protect the
    // buffer with a Mutex as a way to enforce exclusive access to it through the type system.
    rx_buffer: Mutex<RefCell<[u8; eth_phy_dedicated_io::RX_BUFFER_SIZE_BYTES]>>,
}

impl<
        PTimer: hal::timer::Instance,
        PTx: hal::gpio::OutputPin,
        PRx: hal::gpio::InputPin,
        PRxDebug: hal::gpio::OutputPin,
    > InterruptHandler<PTimer, PTx, PRx, PRxDebug>
{
    /// Creates a new instance.
    pub const fn new() -> InterruptHandler<PTimer, PTx, PRx, PRxDebug> {
        Self {
            shared_state: Mutex::new(RefCell::new(InterruptHandlerState::Detached)),
            rx_buffer: Mutex::new(RefCell::new(
                [0u8; eth_phy_dedicated_io::RX_BUFFER_SIZE_BYTES],
            )),
        }
    }

    /// Transitions this instance to an attached state where it cooperates with and shares resources
    /// with a [Phy] object.
    ///
    /// Interrupts will be scheduled using the timer/pin resources provided, but the actual
    /// interrupt routines and the global interrupt flag must be enabled elsewhere.
    ///
    /// An instance can only be attached to one [Phy] at a time. An
    /// [Error::InterruptHandlerAlreadyAttachedError] will be returned if this constraint is
    /// violated.
    fn attach(
        &'static self,
        mut shared_resources: InterruptSharedResources<PTimer, PTx, PRx, PRxDebug>,
    ) -> Result<()> {
        critical_section::with(|cs| {
            let mut state = self.shared_state.borrow_ref_mut(cs);
            match &mut *state {
                InterruptHandlerState::Attached(_) => {
                    Err(Error::InterruptHandlerAlreadyAttachedError)
                }
                InterruptHandlerState::Detached => {
                    // Set the timer to run every 16ms so we can issue LTPs.
                    shared_resources.timer.start(16u64.millis());
                    shared_resources.timer.listen();

                    // Set up an edge-triggered interrupt on the RX pin. This will be the trigger
                    // for us to try and read an incoming package from the line.
                    shared_resources.rx_pin.clear_interrupt();
                    shared_resources.rx_pin.listen(hal::gpio::Event::AnyEdge);

                    *state = InterruptHandlerState::Attached(shared_resources);
                    Ok(())
                }
            }
        })
    }

    /// Obtains exclusive access to the [InterruptSharedResources] and invokes the given callback
    /// with a mutable reference to them.
    ///
    /// Panics if not called on an [InterruptHandler] instance in the
    /// [InterruptHandlerState::Attached] state.
    #[ram]
    fn use_attached_resources<F, T>(&'static self, mut callback: F) -> T
    where
        F: FnMut(&mut InterruptSharedResources<PTimer, PTx, PRx, PRxDebug>, CriticalSection) -> T,
    {
        critical_section::with(|cs| match &mut *self.shared_state.borrow_ref_mut(cs) {
            InterruptHandlerState::Attached(ref mut shared_resources) => {
                callback(shared_resources, cs)
            }
            InterruptHandlerState::Detached => {
                panic!("Trying to use the attached resource but currently detached!")
            }
        })
    }

    /// Callback to be invoked from the timer interrupt handler when timer interrupt fires.
    #[ram]
    pub fn on_timer_interrupt(&'static self) {
        self.use_attached_resources(|resources, _| {
            eth_phy_dedicated_io::transmit_ltp(&mut *resources.tx_pin);
            resources.stats.tx.ltps_sent += 1;
            resources.timer.clear_interrupt();
            // Re-activate the timer alarm so we get triggered after the next period has passed.
            resources.timer.set_alarm_active(true);
        });
    }

    /// Callback to be invoked from the timer interrupt handler when timer interrupt fires.
    ///
    /// Note: you must use the "direct-vectoring" feature of the ESP crate in order to ensure that
    /// the RX interrupt routine is invoked fast enough (I measured it being invoked within about
    /// 750ns of the first incoming signal edge). Without it, it takes too long for the
    /// `#[interrupt]` function to be invoked, which means that by the time this method is invoked
    /// the packet's preamble will already have passed by.
    ///
    /// This method currently disables all interrupts while it's receiving an incoming packet, but
    /// that may be improved in the future (e.g. to allow higher-priority interrupts to interrupt
    /// the receive loop).
    #[ram]
    pub fn on_rx_interrupt(&'static self) {
        // The very first thing we should do is to emit a debug signal to the RX debug pin, to
        // indicate that the interrupt handler was invoked. We do this before we acquire the
        // critical section in use_attached_resources below, to reflect as accurately as possible
        // the time at which this method is invoked.
        eth_phy_dedicated_io::emit_receive_interrupt_debug_signal();

        self.use_attached_resources(|resources, cs| {
            // Try to receive an incoming packet, if there is one.
            //
            // TODO: Avoid running the receive loop within this `use_attached_resources` call, since
            // that means we're disabling all interrupts during this potentially long period of
            // time, and that may not always be desirable. E.g. perhaps we should allow
            // receipts to be interrupted by higher-priority interrupts.
            match eth_phy_dedicated_io::receive_packet(
                &mut self.rx_buffer.borrow_ref_mut(cs),
                &mut *resources.rx_pin,
                &mut *resources.rx_debug_pin,
            ) {
                eth_phy_dedicated_io::ReceivedTransmission::Packet(ref mut data) => {
                    resources.stats.rx.probable_packets_received += 1;

                    // Pass the received packet to the MAC layer for further processing.
                    eth_mac::on_rx_packet_received(data);
                }
                eth_phy_dedicated_io::ReceivedTransmission::LinkTestPulse => {
                    resources.stats.rx.ltps_received += 1;
                }
                eth_phy_dedicated_io::ReceivedTransmission::TruncatedTransmission => {
                    resources.stats.rx.truncated_packets_received += 1;
                }
            };

            resources.rx_pin.clear_interrupt();
        });
    }
}

/// Resources that are shared between the main thread of execution and code running in interrupt
/// handlers.
struct InterruptSharedResources<PTimer: 'static, PTx: 'static, PRx: 'static, PRxDebug: 'static> {
    /// The timer instance is used by the interrupt handler to clear the interrupt and restart the
    /// timer, and by the main thread to schedule the initial timer, so it is a shared resource.
    timer: hal::Timer<PTimer>,
    /// The TX pin used by the interrupt handler to emit LTPs, and by the main thread to transmit
    /// packets, so it is shared resource. In practice the use of the pin is logical in nature,
    /// since we don't actually mutate the pin directly and instead use the dedicated IO mechanism
    /// to mutate the CPU output signal attached to this pin, but by expressing the resource in the
    /// type system we can enforce mutually-exclusive access.
    tx_pin: PeripheralRef<'static, PTx>,
    /// The RX pin used by the interrupt handler to receive data. The pin is only used by the
    /// interrupt handler, but must be passed in at configuration time and hence is shared state.
    rx_pin: PeripheralRef<'static, PRx>,
    /// The pin used to output debug signals on for use in monitoring the receive loop behavior and
    /// timing.
    rx_debug_pin: PeripheralRef<'static, PRxDebug>,
    /// The stats are updated and read during interrupt routine invocations as well as in the main
    /// thread of execution.
    stats: TxRxStats,
}

/// Reflects the initialization state of the [InterruptHandler].
enum InterruptHandlerState<PTimer: 'static, PTx: 'static, PRx: 'static, PRxDebug: 'static> {
    /// Initial state, where it is not attached to any peripheral yet and is effectively
    /// idle/unused.
    Detached,
    /// Attached state, where interrupts are being listened to and may fire at any moment, and where
    /// the peripheral is available for use in the ISR. The interrupt handler is also given a handle
    /// with which it can detect whether a TX operation is currently ongoing.
    Attached(InterruptSharedResources<PTimer, PTx, PRx, PRxDebug>),
}

/// Various transmission-related stats.
#[derive(Default, Debug, Clone, Copy)]
pub struct TxRxStats {
    // Stats about outgoing transmissions.
    pub tx: TxStats,
    // Stats about incoming transmissions.
    pub rx: RxStats,
}

/// Various transmission-related stats.
#[derive(Default, Debug, Clone, Copy)]
pub struct TxStats {
    /// The number of Ethernet packets that have been sent successfully (as far as we can tell from
    /// our side of the line).
    pub packets_sent: u32,
    /// The number of link test pulses that have been transmitted.
    pub ltps_sent: u32,
}

/// Various receipt-related stats.
#[derive(Default, Debug, Clone, Copy)]
pub struct RxStats {
    /// The number of likely link test pulses that have been received.
    pub ltps_received: u32,
    /// The number of probably Ethernet packets that have been received. Note that these packets
    /// have not yet been parsed or validated. These merely reflect received transmissions of a
    /// length that could correspond to a valid data packet.
    pub probable_packets_received: u32,
    /// The number of transmissions that could not be classified as link test pulses, but also
    /// weren't long enough to contain valid packet data.
    pub truncated_packets_received: u32,
}
