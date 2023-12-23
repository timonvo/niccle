//! Implements the functionality of an Ethernet 10BASE-T PHY, like emitting periodic test pulses,
//! Manchester-coding of incoming/outgoing data, etc.

#[cfg(feature = "esp32c3")]
use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
use esp32c6_hal as hal;

use crate::eth_phy_dedicated_io;
use core::cell::RefCell;
use critical_section::Mutex;
use hal::{clock::Clocks, peripheral::PeripheralRef, prelude::*};

#[derive(Debug)]
pub enum Error {
    /// The CPU clock is misconfigured (it must be set to 160MHz).
    IncorrectCpuClockError,
    /// An attempt to use an [InterruptHandler] with more than on [Phy] instance was detected.
    InterruptHandlerAlreadyAttachedError,
}
pub type Result<T> = core::result::Result<T, Error>;

/// A configuration struct for use with the [Phy] constructor.
pub struct PhyConfig<'a, PTimer: 'static, PTx: 'static> {
    /// The currently-configured device clocks, used to validate the clock speed.
    pub clocks: &'a Clocks<'a>,
    /// The [InterruptHandler] to attach the [Phy] to.
    pub interrupt_handler: &'static InterruptHandler<PTimer, PTx>,
    /// The timer to use with the [Phy] instance, e.g. to periodically transmit link test pulses
    /// (LTPs).
    pub timer: hal::timer::Timer<PTimer>,
    /// The GPIO pin to use as the TX pin.
    pub tx_pin: PeripheralRef<'static, PTx>,
}

/// The main entry point to this module. Enables transmission of outgoing packets, and will be
/// extended to support receiving incoming packets in a later version.
pub struct Phy<PTimer: 'static, PTx: 'static> {
    interrupt_handler: &'static InterruptHandler<PTimer, PTx>,
}

impl<PTimer: hal::timer::Instance, PTx: hal::gpio::OutputPin> Phy<PTimer, PTx> {
    /// Creates a new instance and attaches it to the given [InterruptHandler].
    ///
    /// A single InterruptHandler can only be used with one [Phy] instance, and an
    /// [Error::InterruptHandlerAlreadyAttachedError] will be returned if this constraint is
    /// violated.
    ///
    /// Interrupts will be scheduled using the provided timer. The timer's corresponding interrupt
    /// routine must be defined elsewhere and enabled by the caller after this method returns, and
    /// it must call [InterruptHandler::on_timer_interrupt] when it fires.
    pub fn new(mut config: PhyConfig<PTimer, PTx>) -> Result<Phy<PTimer, PTx>> {
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
        // Attach to the InterruptHandler. This will schedule one or more interrupts.
        config.interrupt_handler.attach(InterruptSharedResources {
            timer: config.timer,
            tx_pin: config.tx_pin,
            tx_stats: Default::default(),
        })?;
        Ok(Self {
            interrupt_handler: config.interrupt_handler,
        })
    }

    /// Returns stats about this instance's activity so far.
    pub fn stats(&mut self) -> TxStats {
        self.interrupt_handler
            .use_attached_resources(|resources| resources.tx_stats)
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
    /// This method currently disables all interrupts during transmission, but that may be improved
    /// in the future.
    pub fn transmit_packet(&mut self, data: &[u8]) {
        self.interrupt_handler.use_attached_resources(|resources| {
            // Disable the LTP timer, since we we're about to transmit data and so no LTP will be
            // needed for another 16ms.
            resources.timer.set_alarm_active(false);
            // TODO: Avoid doing the transmission within this `use_attached_resources` call, since
            // that means we're disabling all interrupts during this potentially long period of
            // time, and that may not always be desirable. E.g. perhaps we should allow
            // transmissions to be interrupted, and re-transmit them post-hoc if and when we detect
            // such interrupts have occurred.
            eth_phy_dedicated_io::transmit_packet(data, &mut *resources.tx_pin);
            resources.tx_stats.packets_sent += 1;
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
pub struct InterruptHandler<PTimer: 'static, PTx: 'static> {
    state: Mutex<RefCell<InterruptHandlerState<PTimer, PTx>>>,
}

impl<PTimer: hal::timer::Instance, PTx: hal::gpio::OutputPin> InterruptHandler<PTimer, PTx> {
    /// Creates a new instance.
    pub const fn new() -> InterruptHandler<PTimer, PTx> {
        Self {
            state: Mutex::new(RefCell::new(InterruptHandlerState::Detached)),
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
        mut shared_resources: InterruptSharedResources<PTimer, PTx>,
    ) -> Result<()> {
        critical_section::with(|cs| {
            let mut state = self.state.borrow_ref_mut(cs);
            match &mut *state {
                InterruptHandlerState::Attached(_) => {
                    Err(Error::InterruptHandlerAlreadyAttachedError)
                }
                InterruptHandlerState::Detached => {
                    // Set the timer to run every 16ms so we can issue LTPs.
                    shared_resources.timer.start(16u64.millis());
                    shared_resources.timer.listen();
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
        F: FnMut(&mut InterruptSharedResources<PTimer, PTx>) -> T,
    {
        critical_section::with(|cs| match &mut *self.state.borrow_ref_mut(cs) {
            InterruptHandlerState::Attached(ref mut shared_resources) => callback(shared_resources),
            InterruptHandlerState::Detached => {
                panic!("Trying to use the attached resource but currently detached!")
            }
        })
    }

    /// Callback to be invoked from the timer interrupt handler when timer interrupt fires.
    #[ram]
    pub fn on_timer_interrupt(&'static self) {
        self.use_attached_resources(|resources| {
            eth_phy_dedicated_io::transmit_ltp(&mut *resources.tx_pin);
            resources.tx_stats.ltps_sent += 1;
            resources.timer.clear_interrupt();
            // Re-activate the timer alarm so we get triggered after the next period has passed.
            resources.timer.set_alarm_active(true);
        });
    }
}

/// Resources that are shared between the main thread of execution and code running in interrupt
/// handlers.
struct InterruptSharedResources<PTimer: 'static, PTx: 'static> {
    /// The timer instance is used by the interrupt handler to clear the interrupt and restart the
    /// timer, and by the main thread to schedule the initial timer, so it is a shared resource.
    timer: hal::Timer<PTimer>,
    /// The TX pin used by the interrupt handler to emit LTPs, and by the main thread to transmit
    /// packets, so it is shared resource. In practice the use of the pin is logical in nature,
    /// since we don't actually mutate the pin directly and instead use the dedicated IO mechanism
    /// to mutate the CPU output signal attached to this pin, but by expressing the resource in the
    /// type system we can enforce mutually-exclusive access.
    tx_pin: PeripheralRef<'static, PTx>,
    /// The TX stats are updated by the interrupt handler when emitting LTPs, and by the main thread
    /// when transmitting packets, so it is a shared resource.
    tx_stats: TxStats,
}

/// Reflects the initialization state of the [InterruptHandler].
enum InterruptHandlerState<PTimer: 'static, PTx: 'static> {
    /// Initial state, where it is not attached to any peripheral yet and is effectively
    /// idle/unused.
    Detached,
    /// Attached state, where interrupts are being listened to and may fire at any moment, and where
    /// the peripheral is available for use in the ISR. The interrupt handler is also given a handle
    /// with which it can detect whether a TX operation is currently ongoing.
    Attached(InterruptSharedResources<PTimer, PTx>),
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
