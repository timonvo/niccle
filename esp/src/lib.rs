#![no_std]
// We use this feature to write inline assembly with readable register names.
#![feature(asm_const)]

// This will contain ESP chipset-specific code.

/// Implements the functionality of an Ethernet 10BASE-T PHY, like emitting periodic test pulses,
/// Manchester-coding of incoming/outgoing data, etc.
pub mod eth_phy {
    #[cfg(feature = "esp32c3")]
    use esp32c3_hal as hal;
    #[cfg(feature = "esp32c6")]
    use esp32c6_hal as hal;

    use core::arch::asm;
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

    /// For now the implementation is hardcoded to always use CPU output signal 0 when using
    /// dedicated IO on the TX pin. In the future we could try to make this more flexible, e.g.
    /// using an argument, or perhaps even a const generic parameter with a const expression bound.
    #[cfg(feature = "esp32c3")]
    pub const TX_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal = hal::gpio::OutputSignal::CPU_GPIO_0;
    #[cfg(feature = "esp32c6")]
    pub const TX_CPU_OUTPUT_SIGNAL: hal::gpio::OutputSignal =
        hal::gpio::OutputSignal::CPU_GPIO_OUT0;
    /// The dedicated IO output signal index to use (from 0 to 7, where 0 corresponds to
    /// CPU_GPIO_OUT0). Based on whatever [TX_CPU_OUTPUT_SIGNAL] is set to.
    #[cfg(feature = "esp32c3")]
    const TX_CPU_OUTPUT_SIGNAL_CSR_IDX: isize =
        (TX_CPU_OUTPUT_SIGNAL as isize) - hal::gpio::OutputSignal::CPU_GPIO_0 as isize;
    #[cfg(feature = "esp32c6")]
    const TX_CPU_OUTPUT_SIGNAL_CSR_IDX: isize =
        (TX_CPU_OUTPUT_SIGNAL as isize) - hal::gpio::OutputSignal::CPU_GPIO_OUT0 as isize;

    impl<PTimer: hal::timer::Instance, PTx: hal::gpio::OutputPin> Phy<PTimer, PTx> {
        /// Creates a new instance and attaches it to the given [InterruptHandler].
        ///
        /// A single InterruptHandler can only be used with one [Phy] instance, and an
        /// [Error::InterruptHandlerAlreadyAttachedError] will be returned if this constraint is
        /// violated.
        ///
        /// Interrupts will be scheduled using the provided timer. The timer's corresponding
        /// interrupt routine must be defined elsewhere and enabled by the caller after this method
        /// returns, and it must call [InterruptHandler::on_timer_interrupt] when it fires.
        pub fn new(mut config: PhyConfig<PTimer, PTx>) -> Result<Phy<PTimer, PTx>> {
            if config.clocks.cpu_clock.to_MHz() != 160 {
                return Err(Error::IncorrectCpuClockError);
            }
            // Set the TX pin as output in the GPIO matrix, connect it to the CPU's dedicated GPIO
            // output signal TX_CPU_OUTPUT_SIGNAL, and ensure that its configured as an output in
            // the dedicated GPIO peripheral based on the GPIO matrix setting (rather than having to
            // write to the CPU_GPIO_OEN peripheral register to configure it as an output pin, see
            // Table 7­.2 in the ESP32C6 Technical Reference Manual).
            config.tx_pin.set_to_push_pull_output();
            config.tx_pin.connect_peripheral_to_output_with_options(
                TX_CPU_OUTPUT_SIGNAL,
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
        /// Note that the packet must consist of the (unencoded) preamble, SFD and Ethernet frame
        /// data. I.e. the caller is more or less responsible for the MAC layer, and this function
        /// takes care of the PHY layer responsibilities like Manchester-encoding and TP_IDL
        /// emission at the end of the transmission.
        ///
        /// Data will be transmitted one byte at a time, with transmission going in LSB-to-MSB
        /// order.
        ///
        /// Note that for a maximum-length Ethernet packet of 1530 octets this method will take at
        /// least 1224us to transmit. For a minimum-length Ethernet packet of 72 octets it will take
        /// at least 57.85us.
        ///
        /// This method currently disables all interrupts during transmission, but that may be
        /// improved in the future.
        pub fn transmit_packet(&mut self, data: &[u8]) {
            self.interrupt_handler.use_attached_resources(|resources| {
                // Disable the LTP timer, since we we're about to transmit data and so no LTP will
                // be needed for another 16ms.
                resources.timer.set_alarm_active(false);
                // TODO: Avoid doing the transmission within this `use_attached_resources` call,
                // since that means we're disabling all interrupts during this potentially long
                // period of time, and that may not always be desirable. E.g. perhaps we should
                // allow transmissions to be interrupted, and re-transmit them post-hoc if and when
                // we detect such interrupts have occurred.
                transmit_packet(data, &mut *resources.tx_pin);
                resources.tx_stats.packets_sent += 1;
                // Re-activate the timer alarm so we get triggered after the next period has passed.
                resources.timer.reset_counter();
                resources.timer.set_alarm_active(true);
            })
        }
    }

    /// Handles interrupts scheduled by the [Phy], and manages sharing of resources used by code
    /// running in interrupt handlers and code running the main thread of execution.
    ///
    /// To use this you generally will define static singleton of this type, which can then be
    /// safely used from both the interrupt handling routine and the main thread of execution, since
    /// this class is [Sync].
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

        /// Transitions this instance to an attached state where it cooperates with and shares
        /// resources with a [Phy] object.
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

        /// Obtains exclusive access to the [InterruptSharedResources] and invokes the given
        /// callback with a mutable reference to them.
        ///
        /// Panics if not called on an [InterruptHandler] instance in the
        /// [InterruptHandlerState::Attached] state.
        fn use_attached_resources<F, T>(&'static self, mut callback: F) -> T
        where
            F: FnMut(&mut InterruptSharedResources<PTimer, PTx>) -> T,
        {
            critical_section::with(|cs| match &mut *self.state.borrow_ref_mut(cs) {
                InterruptHandlerState::Attached(ref mut shared_resources) => {
                    callback(shared_resources)
                }
                InterruptHandlerState::Detached => {
                    panic!("Trying to use the attached resource but currently detached!")
                }
            })
        }

        /// Callback to be invoked from the timer interrupt handler when timer interrupt fires.
        pub fn on_timer_interrupt(&'static self) {
            self.use_attached_resources(|resources| {
                transmit_ltp(&mut *resources.tx_pin);
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
        /// The timer instance is used by the interrupt handler to clear the interrupt and restart
        /// the timer, and by the main thread to schedule the initial timer, so it is a shared
        /// resource.
        timer: hal::Timer<PTimer>,
        /// The TX pin used by the interrupt handler to emit LTPs, and by the main thread to
        /// transmit packets, so it is shared resource. In practice the use of the pin is logical in
        /// nature, since we don't actually mutate the pin directly and instead use the dedicated IO
        /// mechanism to mutate the CPU output signal attached to this pin, but by expressing the
        /// resource in the type system we can enforce mutually-exclusive access.
        tx_pin: PeripheralRef<'static, PTx>,
        /// The TX stats are updated by the interrupt handler when emitting LTPs, and by the main
        /// thread when transmitting packets, so it is a shared resource.
        tx_stats: TxStats,
    }

    /// Reflects the initialization state of the [InterruptHandler].
    enum InterruptHandlerState<PTimer: 'static, PTx: 'static> {
        /// Initial state, where it is not attached to any peripheral yet and is effectively
        /// idle/unused.
        Detached,
        /// Attached state, where interrupts are being listened to and may fire at any moment, and
        /// where the peripheral is available for use in the ISR. The interrupt handler is also
        /// given a handle with which it can detect whether a TX operation is currently ongoing.
        Attached(InterruptSharedResources<PTimer, PTx>),
    }

    /// Various transmission-related stats.
    #[derive(Default, Debug, Clone, Copy)]
    pub struct TxStats {
        /// The number of Ethernet packets that have been sent successfully (as far as we can tell
        /// from our side of the line).
        pub packets_sent: u32,
        /// The number of link test pulses that have been transmitted.
        pub ltps_sent: u32,
    }

    /// This CSR isn't defined by the esp-rs/esp-hal framework yet (nor by the espressif/svd files),
    /// so we hardcode their addresses. See the "1.14 Dedicated IO" chapter of the technical
    /// reference manual.
    const CSR_CPU_GPIO_OUT: u32 = 0x805;

    /// Transmits a single link test pulse.
    ///
    /// Note that `_tx_periph` is technically unused, but by requiring a reference to it we ensure
    /// mutually exclusive use of the pin's corresponding `TX_CPU_OUTPUT_SIGNAL`.
    ///
    // Note: the use of #[ram] is important, as it it ensures that there won't be any instruction
    // execution delays, and each instruction will take 6.25ns. See `A note on writing precisely
    // timed assembly code` on [transmit_packet] below.
    #[ram]
    fn transmit_ltp(_tx_periph: &mut impl hal::gpio::OutputPin) {
        unsafe {
            asm!(
                // Set the output high, and keep it high for 62.5ns (10 CPU cycles).
                "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // 1 cycle
                // 9 cycles
                "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
                // Set the output low.
                "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}",
                csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
                cpu_signal_idx= const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
            );
        }
    }

    /// Transmits the given Ethernet packet over the transmission line.
    ///
    /// See [Phy::transmit_packet] for more info.
    ///
    /// # A note on writing precisely timed assembly code
    ///
    /// When transmitting data, each half of a Manchester-encoded bit symbol should take 50ns, for a
    /// total of 100ns per bit symbol. Therefore, given a clock speed of 160MHz (enforced above), we
    /// must use exactly eight CPU cycles between each signal transition. We can't really rely on
    /// the compiler to generate code for us that adheres to this requirement, so we must write the
    /// whole transmission control loop in assembly code instead, using the dedicated IO feature to
    /// emit signals using just a single CPU cycle.
    ///
    /// Unsurprisingly that has its own perils. At 160MHz, each clock cycle takes 6.25ns. Some
    /// instructions take just a single cycle to execute, but some can take more, and the behavior
    /// is chip dependent and situation dependent. The `cycle_counting` example binary contains a
    /// lot more useful information on this topic, as does the discussion at
    /// https://ctrlsrc.io/posts/2023/counting-cpu-cycles-on-esp32c3-esp32c6/.
    ///
    /// For this particular function, the following insights gained from the `cycle_counting`
    /// example binary are most relevant (focusing on the behavior on ESP32-C6 for now):
    ///
    /// * `csrr`, `andi`, xori`, and other simple instructions not otherwise called out below take 1
    ///   CPU cycle.
    /// * `lbu` instructions take 2 CPU cycles, as long as there is a data dependency immediately
    ///   after them (i.e. an instruction that uses the result of the load instruction).
    /// * `bne` instructions in this type of loop takes 3 CPU cycles when not taken (at the end of
    ///   the iteration) if the instruction falls on a 4 byte-aligned address, and 4 CPU cycles when
    ///   not taken and unaligned. They take 2 CPU cycles when taken for the first time (in the
    ///   first iteration), and 1 CPU cycle when taken after having been taken before already (in
    ///   the second to second-to-last iterations).
    ///
    /// See the [cycle_counting::branch_back_usually_taken] and
    /// [cycle_counter::_branch_back_usually_taken_w_lbu] for the benchmarks showing this behavior
    /// for the ESP32-C6 CPU.
    ///
    /// Note: the use of #[ram] is also important, as it it ensures that there won't be any
    /// instruction execution delays, and each instruction will take 6.25ns.
    #[ram]
    fn transmit_packet(data: &[u8], _tx_periph: &mut impl hal::gpio::OutputPin) {
        let data_ptr_range = data.as_ptr_range();

        // Given the concerns listed above, the easiest approach to writing a transmission control
        // loop with deterministic timing is to write an unrolled loop that transmits an octet (8
        // bits) of data at a time, with only a single jump at the end. That way we can most easily
        // ensure that the loop takes exactly the right amount of CPU cycles, by ensuring that the
        // single branch instruction is placed at a 4 byte-aligned address.

        // Note: the following loop performs Manchester encoding as part of the iteration, by taking
        // each bit to transmit and then XOR'ing it with 1, emitting that value as the first
        // bit-half and then emitting the original bit value as the second bit-half. This means we
        // don't have to encode the input data first, before then transmitting the encoded data,
        // saving on CPU time and avoiding the need for a 2nd buffer to hold the encoded data.
        unsafe {
            asm!(
                // Read the current CSR value.
                "csrr {csr_base_value}, {csr_cpu_gpio_out}",
                // Clear the bit corresponding to the CPU output signal we will use.
                "andi {csr_base_value}, {csr_base_value},~(1 << {cpu_signal_idx})",
                // Writing {csr_base_value} to the CSR will now preserve the state of all the other
                // CPU output signals that we aren't using in this function (just in case some other
                // piece of code is using the dedicated IO feature as well..)

                // This ensures that the label is 4 byte-aligned, and hence that all instructions'
                // alignments are deterministic, regardless of the changes that might occur around
                // this code. This is important because the alignment of the `bne` instruction all
                // the way at the end determines whether it takes 3 or 4 CPU cycles in the final
                // iteration.
                ".align 4",
                // Start of next byte processing logic. There should be exactly 7 CPU cycles up to
                // and including the `csrw` instruction, so that there's 8 cycles in total when
                // accounting for the `bne` instruction at the end of the previous iteration.
                "1:",
                // Load the next byte into the {data_byte} register.
                "lbu {data_byte}, 0({data_ptr})", // 1st and 2nd cycle
                // Process bit 1, starting by extracting the data bit from the LSB.
                //
                // Note that the `andi` instruction *must* follow the `lbu` instruction to ensure
                // that the `lbu` instruction consistently takes 2 cycles each time it is executed.
                // See the comment in [cycle_counting::_branch_back_usually_taken_w_lbu] for more
                // info on why this is important.
                "andi {data_bit}, {data_byte}, 1", // 3rd cycle
                // XOR with 1 to generate the first half of the Manchester-encoded bit symbol.
                "xori {tmp}, {data_bit}, 1", // 4th cycle
                // Shift the XOR'd data bit to the right CPU output signal index.
                "slli {tmp}, {tmp}, {cpu_signal_idx}", // 5th cycle
                // Apply it to the base CSR value to produce the target CSR value (writing to our
                // target bit but leaving the other bits unchanged from the base CSR value).
                "or {tmp}, {tmp}, {csr_base_value}", // 6th cycle
                // Emit the first half of the bit symbol. Note that when we execute this instruction
                // the very first time, the timing doesn't really matter. After that, this
                // instruction needs to be executed on the 8th cycle since the last `csrw` at the
                // end of the previous iteration. It's 7 CPU cycles away from the "1:" label, which
                // means it's 8-9 cycles away from the previous iteration's `csrw`, when we take the
                // `bne` instruction into account.
                //
                // TODO: This means that the timing for the 1st byte (which will be part of the
                // preamble) will be off by 6.25ns. We should fix that.
                "csrw {csr_cpu_gpio_out}, {tmp}", // 7th cycle
                // Now proceed to the second half of the Manchester-encoded bit symbol, by again
                // shifting the (non-XOR'd) data bit to the right CPU output signal index.
                "slli {tmp}, {data_bit}, {cpu_signal_idx}", // 1st cycle
                // Apply it to the CSR value to produce the target CSR value.
                "or {tmp}, {tmp}, {csr_base_value}", // 2nd cycle
                "nop", // 3rd cycle
                "nop", // 4th cycle
                "nop", // 5th cycle
                "nop", // 6th cycle
                "nop", // 7th cycle
                // Emit the second half of the bit symbol. This must happen in the 8th cycle since
                // the last `csrw`.
                "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle

                // A macro that shifts the data one to the right and writes the two bit halves
                // corresponding the LSB to the CPU output signal, taking exactly 8 CPU cycles for
                // each bit half.
                ".macro write_single_bit bit_idx",
                // Shift the data byte by one and extract the new data bit from the LSB.
                "srl {data_byte}, {data_byte}, 1", // 1st cycle
                "andi {data_bit}, {data_byte}, 1", // 2nd cycle
                // XOR with 1 to generate the first half of the bit symbol.
                "xori {tmp}, {data_bit}, 1", // 3rd cycle
                // Create the new CSR value.
                "slli {tmp}, {tmp}, {cpu_signal_idx}", // 4th cycle
                "or {tmp}, {tmp}, {csr_base_value}", // 5th cycle
                "nop", // 6th cycle
                "nop", // 7th cycle
                // Output the first half of the bit symbol.
                "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle
                // Now proceed to the second half of the bit symbol, by again shifting the
                // (non-XOR'd) data bit to the right CPU output signal index.
                "slli {tmp}, {data_bit}, {cpu_signal_idx}", // 1st cycle
                // Create the new CSR value.
                "or {tmp}, {tmp}, {csr_base_value}", // 2nd cycle
                "nop", // 3rd cycle
                "nop", // 4th cycle
                "nop", // 5th cycle
                "nop", // 6th cycle
                "nop", // 7th cycle
                // Output the second half of the bit symbol.
                "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle
                ".endm",

                // Process bits 2 through 7.
                "write_single_bit",
                "write_single_bit",
                "write_single_bit",
                "write_single_bit",
                "write_single_bit",
                "write_single_bit",

                // Process bit 8.
                // Shift the data byte by one and extract the new data bit from the LSB.
                "srl {data_byte}, {data_byte}, 1", // 1st cycle
                "andi {data_bit}, {data_byte}, 1", // 2nd cycle
                // XOR with 1 to generate the first half of the bit symbol.
                "xori {tmp}, {data_bit}, 1", // 3rd cycle
                // Create the new CSR value.
                "slli {tmp}, {tmp}, {cpu_signal_idx}", // 4th cycle
                "or {tmp}, {tmp}, {csr_base_value}", // 5th cycle
                "nop", // 6th cycle
                "nop", // 7th cycle
                // Emit the first half of the bit symbol
                "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle
                // Now proceed to the second half of the bit symbol, by again shifting the
                // (non-XOR'd) data bit to the right CPU output signal index.
                "slli {tmp}, {data_bit}, {cpu_signal_idx}", // 1st cycle
                // Create the new CSR value.
                "or {tmp}, {tmp}, {csr_base_value}", // 2nd cycle
                "nop", // 3rd cycle
                "nop", // 4th cycle
                "nop", // 5th cycle
                // 6th cycle, this is equivalent to `nop`, but is encoded as a 32-bit instruction.
                // We use it instead of `nop` to ensure that the `bne` instruction below is at a
                // 4-byte aligned address.
                "andi zero, zero, 0",
                // Advance the data pointer by one in anticipation of the branch check below.
                "addi {data_ptr}, {data_ptr}, 1", // 7th cycle
                // Output the bit via the CSR.
                "csrw {csr_cpu_gpio_out}, {tmp}", // 8th cycle

                // Loop back to process the next data byte, if there's any more. The first time we
                // hit this instruction it'll take 2 cycles, then it'll take 1 cycle for taken
                // branches in the future, and 3 cycles for the final non-taken branch. Note that
                // the non-taken branch takes 3 cycles because we've ensured, above, that this
                // instruction falls on a 4 byte-aligned address.
                "bne {data_ptr}, {data_end_ptr}, 1b",
                // If we reach here then we've reached the end of the data transmission and we need
                // to proceed to the start-of-idle signal. These `nop`` instructions ensure we hold
                // the line high or low for long a total of at least 8 cycles on behalf of the last
                // bit sent before we do so.
                "nop", // 4th cycle
                "nop", // 5th cycle
                "nop", // 6th cycle
                "nop", // 7th cycle

                // Emit a TP_IDL signal by setting the pin high and keeping it high 250ns (40
                // cycles), then setting it low.
                "csrrsi zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // 8th cycle
                // 39 cycles
                "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
                "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
                "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
                "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop", "nop",
                // Set the output low.
                "csrrci zero, {csr_cpu_gpio_out}, 1<<{cpu_signal_idx}", // 40th cycle
                // The specifier of the CSR we have to write to to perform direct IO.
                csr_cpu_gpio_out = const(CSR_CPU_GPIO_OUT),
                cpu_signal_idx = const(TX_CPU_OUTPUT_SIGNAL_CSR_IDX),
                // The address of the byte array we're processing. Must be `inout(...)` since we
                // modify the register during execution.
                data_ptr = inout(reg) data_ptr_range.start => _,
                // The end address (exclusive, i.e. the address one past the last valid address).
                data_end_ptr = in(reg) data_ptr_range.end,
                // Will hold the byte we're processing.
                data_byte = out(reg) _,
                // Will hold the data bit being processed.
                data_bit = out(reg) _,
                // Will hold the original CSR value, to which we'll apply the specific GPIO's bit
                // value.
                csr_base_value = out(reg) _,
                // A general scratch register.
                tmp = out(reg) _,
            );
        };
    }
}
