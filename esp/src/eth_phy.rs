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
pub struct PhyConfig<'a, PTimer: 'static, PTx: 'static, PRx: 'static, PRxDebug: 'static, M: 'static>
{
    /// The currently-configured device clocks, used to validate the clock speed.
    pub clocks: &'a Clocks<'a>,
    /// The [InterruptHandler] to attach the [Phy] to.
    pub interrupt_handler: &'static InterruptHandler<PTimer, PTx, PRx, PRxDebug, M>,
    /// The timer to use with the [Phy] instance, e.g. to periodically transmit link test pulses
    /// (LTPs).
    pub timer: hal::timer::Timer<PTimer>,
    /// The GPIO pin to use as the TX pin.
    pub tx_pin: PeripheralRef<'static, PTx>,
    /// The GPIO pin to use as the RX pin.
    pub rx_pin: PeripheralRef<'static, PRx>,
    /// Indicates whether the RX input signal should be inverted.
    pub rx_invert_signal: bool,
    /// The callback through which to pass received packets to the MAC layer for further processing.
    pub rx_mac_callback: M,
    /// The GPIO pin to use as the RX debug output pin, for monitoring the behavior and timing of
    /// the receive loop.
    // TODO: Make the use of this pin optional.
    pub rx_debug_pin: PeripheralRef<'static, PRxDebug>,
}

/// The main entry point to this module. Enables transmission of outgoing packets, and will be
/// extended to support receiving incoming packets in a later version.
pub struct Phy<PTimer: 'static, PTx: 'static, PRx: 'static, PRxDebug: 'static, M: 'static> {
    interrupt_handler: &'static InterruptHandler<PTimer, PTx, PRx, PRxDebug, M>,
}

impl<
        PTimer: hal::timer::Instance,
        PTx: hal::gpio::OutputPin,
        PRx: hal::gpio::InputPin,
        PRxDebug: hal::gpio::OutputPin,
        M: eth_mac::PhyRxToMacRxBridge,
    > Phy<PTimer, PTx, PRx, PRxDebug, M>
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
        mut config: PhyConfig<PTimer, PTx, PRx, PRxDebug, M>,
    ) -> Result<Phy<PTimer, PTx, PRx, PRxDebug, M>> {
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
            rx_mac_callback: config.rx_mac_callback,
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
}

impl<
        'a,
        PTimer: hal::timer::Instance,
        PTx: hal::gpio::OutputPin,
        PRx: hal::gpio::InputPin,
        PRxDebug: hal::gpio::OutputPin,
        M: eth_mac::PhyRxToMacRxBridge,
    > eth_mac::PhyTx<'a> for Phy<PTimer, PTx, PRx, PRxDebug, M>
{
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
    fn transmit_packet(&self, data: &[u8]) {
        self.interrupt_handler
            .use_attached_resources(|resources, _| {
                // Disable the LTP timer, since we we're about to transmit data and so no LTP will
                // be needed for another 16ms.
                resources.timer.set_alarm_active(false);
            });
        // TODO: Avoid doing the transmission within this `use_attached_resources` call,
        // since that means we're disabling all interrupts during this potentially long
        // period of time, and that may not always be desirable. E.g. perhaps we should
        // allow transmissions to be interrupted, and re-transmit them post-hoc if and when
        // we detect such interrupts have occurred.
        self.interrupt_handler
            .use_attached_resources(|resources, _| {
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
#[repr(align(16))]
pub struct InterruptHandler<
    PTimer: 'static,
    PTx: 'static,
    PRx: 'static,
    PRxDebug: 'static,
    M: 'static,
> {
    // TODO: simplify the many layers of generics.
    #[allow(clippy::type_complexity)]
    shared_state: Mutex<RefCell<InterruptHandlerState<PTimer, PTx, PRx, PRxDebug, M>>>,
}

impl<
        PTimer: hal::timer::Instance,
        PTx: hal::gpio::OutputPin,
        PRx: hal::gpio::InputPin,
        PRxDebug: hal::gpio::OutputPin,
        M: eth_mac::PhyRxToMacRxBridge,
    > InterruptHandler<PTimer, PTx, PRx, PRxDebug, M>
{
    /// Creates a new instance.
    pub const fn new() -> InterruptHandler<PTimer, PTx, PRx, PRxDebug, M> {
        Self {
            shared_state: Mutex::new(RefCell::new(InterruptHandlerState::Detached)),
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
        mut shared_resources: InterruptSharedResources<PTimer, PTx, PRx, PRxDebug, M>,
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
        F: FnMut(
            &mut InterruptSharedResources<PTimer, PTx, PRx, PRxDebug, M>,
            CriticalSection,
        ) -> T,
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

        self.use_attached_resources(
            |resources, _| {
                // TODO: Avoid running the receive loop within this `use_attached_resources` call,
                // since that means we're disabling all interrupts during this potentially long
                // period of time, and that may not always be desirable. E.g. perhaps we should
                // allow receipts to be interrupted by higher-priority interrupts.

                // First, ask the MAC layer to reserve a new packet buffer for the PHY receive loop
                // to write data into.
                let buffer_added =
                    resources
                        .rx_mac_callback
                        .with_new_packet_buffer(|buffer: &mut [u8]| {
                            // We've received a buffer from the MAC layer, so now run the receive loop,
                            // which may or may not actually write data to this buffer, depending on
                            // whether we got interrupted due an incoming link test pulse vs. an actual
                            // incoming packet.
                            //
                            // Note that the `receive_packet` function will emit a debug signal once
                            // more, at the very start of the receive loop, all throughout the loop, and
                            // more more at the very end of the receive loop as well.
                            let mut result = eth_phy_dedicated_io::receive_packet(
                                buffer,
                                &mut *resources.rx_pin,
                                &mut *resources.rx_debug_pin,
                            );

                            // We now need to determine how much data we actually received, based on the
                            // result of the receive loop.
                            let received_data;
                            match result {
                            eth_phy_dedicated_io::ReceivedTransmission::Packet(ref mut data) => {
                                // We received a probable (but so far unvalidated!) packet, which
                                // was written into the buffer provided by the MAC layer.
                                received_data = data.len();
                                resources.stats.rx.probable_packets_received += 1;
                            }
                            eth_phy_dedicated_io::ReceivedTransmission::LinkTestPulse => {
                                received_data = 0;
                                resources.stats.rx.ltps_received += 1;
                            }
                            eth_phy_dedicated_io::ReceivedTransmission::TruncatedTransmission => {
                                // Note that some data may have actually been written to the buffer
                                // in this case, but we already can tell it wasn't valid data, and
                                // hence it should just be discarded.
                                received_data = 0;

                                // One of the situations in which a truncated transmission might be
                                // detected is when the RX buffer was full and hence the RX
                                // interrupt as disabled.  We might then end up re-enabling the
                                // interrupt right as we're in the middle of another incoming packet
                                // transmission, which would then look like a truncated read.

                                resources.stats.rx.truncated_packets_received += 1;
                            }
                        };

                            // Return the amount of received data to the MAC layer, which will commit
                            // that data to the RX queue.
                            received_data
                        });
                // If the MAC layer failed to give us buffer space then that indicates that its RX
                // queue is full. In this case we should stop listening to the interrupt until we've
                // some space becomes available again (as indicated by the MAC layer calling our
                // [InterruptHandler::restart_rx_interrupt]).
                //
                // This ensures that ongoing incoming packets (which we won't be able to store
                // anyway) won't keep waking up the interrupt repeatedly, thereby preventing us from
                // actually consuming the already-received packets on the main thread of execution.
                if !buffer_added {
                    resources.stats.rx.out_of_buffer_space_events += 1;
                    resources.rx_pin.unlisten();
                }
                resources.rx_pin.clear_interrupt();
            },
        );

        // Lastly, let's emit a double debug signal at the very end, to signify we're about to
        // return from the interrupt routine.
        eth_phy_dedicated_io::emit_receive_interrupt_debug_signal();
        eth_phy_dedicated_io::emit_receive_interrupt_debug_signal();
    }

    // Callback to be invoked when RX buffer space has become available again at the MAC layer.
    #[ram]
    pub fn rx_buffer_space_available(&'static self) {
        self.use_attached_resources(
            |resources, _| resources.rx_pin.listen(hal::gpio::Event::AnyEdge),
        );
    }
}

/// Resources that are shared between the main thread of execution and code running in interrupt
/// handlers.
struct InterruptSharedResources<
    PTimer: 'static,
    PTx: 'static,
    PRx: 'static,
    PRxDebug: 'static,
    M: 'static,
> {
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
    /// The callback through which to pass received packets to the MAC layer for further processing.
    rx_mac_callback: M,
    /// The pin used to output debug signals on for use in monitoring the receive loop behavior and
    /// timing.
    rx_debug_pin: PeripheralRef<'static, PRxDebug>,
    /// The stats are updated and read during interrupt routine invocations as well as in the main
    /// thread of execution.
    stats: TxRxStats,
}

/// Reflects the initialization state of the [InterruptHandler].
enum InterruptHandlerState<
    PTimer: 'static,
    PTx: 'static,
    PRx: 'static,
    PRxDebug: 'static,
    M: 'static,
> {
    /// Initial state, where it is not attached to any peripheral yet and is effectively
    /// idle/unused.
    Detached,
    /// Attached state, where interrupts are being listened to and may fire at any moment, and where
    /// the peripheral is available for use in the ISR. The interrupt handler is also given a handle
    /// with which it can detect whether a TX operation is currently ongoing.
    Attached(InterruptSharedResources<PTimer, PTx, PRx, PRxDebug, M>),
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
    /// The number of times the RX interrupt was invoked but no buffer space could be allocated.
    /// These events may mean packets were dropped, if the interrupt fired due to an incoming packet
    /// transmission that could then not be handled.
    pub out_of_buffer_space_events: u32,
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
