//! Implements the functionality of an Ethernet 10BASE-T MAC layer, like preparing outgoing Ethernet
//! packets and passing them to the PHY layer, and validating incoming packets' checksums.

pub mod smoltcp;

use crate::debug_util;
use bitvec::prelude::*;
use core::cell::RefCell;
use crc32fast;
use critical_section::Mutex;
use log::{debug, log, trace};

/// The minimum size of an Ethernet frame. 64 equals 12 bytes of destination and source addresses, 2
/// bytes of EtherType, 46 bytes of payload, and 4 bytes of FCS (we don't support 802.1q-tagged
/// frames).
pub const MIN_FRAME_SIZE: usize = 64;
/// The maximum size of an Ethernet frame. 1518 equals 12 bytes of destination and source addresses,
/// 2 bytes of EtherType, 1500 bytes of payload, and 4 bytes of FCS (we don't support 802.1q-tagged
/// frames).
pub const MAX_FRAME_SIZE: usize = 1518;
/// The length of a frame's FCS sequence, in bytes.
pub const FCS_SIZE: usize = 4;
/// The maximum size of an Ethernet packet (8 bytes of preamble & SFD plus the frame data).
pub const MAX_PACKET_SIZE: usize = MAX_FRAME_SIZE + 8;

/// A trait that the PHY layer should implement to allow the MAC layer to transmit outgoing packets.
pub trait PhyTx<'a> {
    /// Transmits the given packet, blocking until the transmission has finished.
    fn transmit_packet(&self, data: &[u8]);
}

/// A trait implemented by the MAC layer to allow the PHY layer to pass received packets to the MAC
/// layer.
pub trait PhyRxToMacRxBridge {
    /// Called by the PHY layer when it needs a new buffer to be allocated for it to write packet
    /// data to.
    ///
    /// The MAC layer must allocate a new buffer of [MAX_PACKET_SIZE] and pass it to
    /// `buffer_fn`, which will return how much (unaligned and unvalidated) packet data was
    /// actually written to the buffer, after which the MAC layer must commit that buffer to the RX
    /// queue for later validation and consumption.
    ///
    /// The committed packet data will contain part of the preamble, the SFD, the frame data, and up
    /// to two bits of TP_IDL signal. Each byte in the buffer will contain at least one bit of
    /// packet data (i.e. there must be no trailing bytes of bogus data). The callee is responsible
    /// for aligning and validating the data.
    ///
    /// This method must return `true` if `buffer_fn` was actually called, and `false` if no
    /// buffer space could be allocated (e.g. the RX queue is full) and hence `buffer_fn` was
    /// not called.
    ///
    /// Note: since this code may be invoked from within an interrupt routine it should ideally not
    /// block for very long.
    fn with_new_packet_buffer<F>(&mut self, buffer_fn: F) -> bool
    where
        F: FnOnce(&mut [u8]) -> usize;
}

/// A MAC layer implementation for sending outgoing Ethernet frames.
pub struct MacTx<'a, P: PhyTx<'a>> {
    phy: &'a P,
    packet_buffer: [u8; MAX_PACKET_SIZE],
}

/// A full Ethernet packet preamble plus an SFD sequence.
const PREAMBLE_AND_SFD: [u8; 8] = [
    // Preamble
    0b01010101, 0b01010101, 0b01010101, 0b01010101, 0b01010101, 0b01010101, 0b01010101,
    // SFD
    0b11010101,
];

impl<'a, P: PhyTx<'a>> MacTx<'a, P> {
    /// Constructs a new [MacTx] instance.
    pub fn new(phy: &'a P) -> MacTx<'a, P> {
        let mut mac_tx = MacTx {
            phy,
            packet_buffer: [0; MAX_PACKET_SIZE],
        };
        // Add the preamble to the TX buffer since it'll be the same for every packet we transmit.
        mac_tx.packet_buffer[0..PREAMBLE_AND_SFD.len()].copy_from_slice(&PREAMBLE_AND_SFD);
        mac_tx
    }

    /// Transmits the given frame. See [MacTx::transmit_frame_with] for more details.
    pub fn transmit_frame(&mut self, frame_data: &[u8]) {
        self.transmit_frame_with(frame_data.len(), |buffer| {
            buffer.copy_from_slice(frame_data)
        });
    }

    /// Transmits a frame of given length `len` and using `f` to produce the data to transmit.
    ///
    /// This method blocks until the transmission has finished.
    ///
    /// The provided data may or may not be padded already, but must not contain an FCS. A preamble
    /// and SFD will be prepended, and the FCS will be calculated by this method before it passes
    /// the data on to the PHY layer for transmission.
    pub fn transmit_frame_with<F, R>(&mut self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        const DATA_OFFSET: usize = PREAMBLE_AND_SFD.len();

        // Get a subslice of the packet buffer into which we'll write the frame data, padding, and
        // FCS.
        let frame_buffer =
            &mut self.packet_buffer[DATA_OFFSET..][..(len + FCS_SIZE).max(MIN_FRAME_SIZE)];

        // Copy the unpadded data into the frame slice.
        let result = f(&mut frame_buffer[..len]);

        // Add padding to the frame slice.
        let padding_len = MIN_FRAME_SIZE - FCS_SIZE - len.min(MIN_FRAME_SIZE - FCS_SIZE);
        trace!("TX adding {padding_len} bytes of padding");
        let padding = &mut frame_buffer[len..len + padding_len];
        padding.fill(0);

        // Calculate the CRC and place it at the end of the frame slice.
        let fcs_offset = frame_buffer.len() - FCS_SIZE;
        let fcs = crc32fast::hash(&frame_buffer[0..fcs_offset]);
        frame_buffer[fcs_offset] = (fcs & 0xFF) as u8;
        frame_buffer[fcs_offset + 1] = ((fcs >> 8) & 0xFF) as u8;
        frame_buffer[fcs_offset + 2] = ((fcs >> (8 * 2)) & 0xFF) as u8;
        frame_buffer[fcs_offset + 3] = ((fcs >> (8 * 3)) & 0xFF) as u8;

        debug!(">>> TX {}", debug_util::FormatEthernetFrame(frame_buffer));
        debug_util::log_data_hex(log::Level::Trace, frame_buffer);

        let packet_len = DATA_OFFSET + frame_buffer.len();
        self.phy.transmit_packet(&self.packet_buffer[0..packet_len]);
        result
    }
}

/// A MAC layer implementation for receiving incoming Ethernet packets.
///
/// This implementation will buffer up to `BUFFER_CAP` bytes worth of data.
pub struct MacRx<const BUFFER_CAP: usize> {
    packet_queue: bbqueue::BBBuffer<BUFFER_CAP>,
    stats: Mutex<RefCell<RxStats>>,
    rx_queue_has_capacity_again_fn: &'static (dyn Fn() + Sync),
}

/// Produces (unvalidated) packet data, by reserving buffer space for the PHY to write packet data
/// into, and then committing that data for later consumption.
pub struct MacRxPacketProducer<'a, const BUFFER_CAP: usize> {
    packet_producer: bbqueue::framed::FrameProducer<'a, BUFFER_CAP>,
}

/// Provides access to the previously (unvalidated) packet data previously queued by the
/// [MacRxPacketProducer] corresponding to this instance.
pub struct MacRxPacketConsumer<'a, const BUFFER_CAP: usize> {
    packet_consumer: bbqueue::framed::FrameConsumer<'a, BUFFER_CAP>,
    stats: &'a Mutex<RefCell<RxStats>>,
    rx_queue_has_capacity_again_fn: &'static (dyn Fn() + Sync),
}

impl<const BUFFER_CAP: usize> MacRx<BUFFER_CAP> {
    /// Constructs a new [MacRx] instance.
    ///
    /// Once the [MacRx] instance's queue has filled up for the first time, it will call
    /// `rx_queue_has_capacity_again_fn` whenever its RX buffer has capacity again.
    pub const fn new(
        rx_queue_has_capacity_again_fn: &'static (dyn Fn() + Sync),
    ) -> MacRx<BUFFER_CAP> {
        MacRx {
            packet_queue: bbqueue::BBBuffer::new(),
            // We can't use Default::default() since we're in a `const fn`.`
            stats: Mutex::new(RefCell::new(RxStats {
                invalid_sfd_packets_received: 0,
                invalid_crc_packets_received: 0,
                valid_frames_received: 0,
            })),
            rx_queue_has_capacity_again_fn,
        }
    }

    /// Returns stats about this instance's activity so far.
    pub fn stats(&self) -> RxStats {
        critical_section::with(|cs| *self.stats.borrow_ref(cs))
    }

    /// Splits this instance into handles that can be used to produces and consume data,
    /// respectively. Must only be called once.
    pub fn split(
        &self,
    ) -> (
        MacRxPacketProducer<'_, BUFFER_CAP>,
        MacRxPacketConsumer<'_, BUFFER_CAP>,
    ) {
        let (producer, consumer) = self.packet_queue.try_split_framed().unwrap();
        (
            MacRxPacketProducer {
                packet_producer: producer,
            },
            MacRxPacketConsumer {
                packet_consumer: consumer,
                stats: &self.stats,
                rx_queue_has_capacity_again_fn: self.rx_queue_has_capacity_again_fn,
            },
        )
    }
}

impl<'a, const BUFFER_CAP: usize> MacRxPacketConsumer<'a, BUFFER_CAP> {
    /// Consumes one of the previously-received packets, if there is one. Note that the buffer
    /// holding the frame data will not be released until the returned [FrameRef] is
    /// [dropped][Drop::drop].
    pub fn receive(&mut self) -> Option<FrameRef<'a, BUFFER_CAP>> {
        loop {
            let packet = self.packet_consumer.read();
            if let Some(mut packet) = packet {
                // Release the frame data once the frame instance is dropped.
                packet.auto_release(true);

                let Some(aligned_frame_data) = self.align_packet(&mut packet) else {
                    critical_section::with(|cs| {
                        self.stats.borrow_ref_mut(cs).invalid_sfd_packets_received += 1;
                    });
                    // This packet couldn't be aligned, so skip over it and move on to the next
                    // packet in the RX queue.
                    continue;
                };
                let Some(validated_frame_data) = self.validate_frame(aligned_frame_data) else {
                    critical_section::with(|cs| {
                        self.stats.borrow_ref_mut(cs).invalid_crc_packets_received += 1;
                    });
                    // This frame couldn't be validated, so skip over it and move on to the next
                    // packet in the RX queue.
                    continue;
                };
                let frame_ref = FrameRef {
                    length: validated_frame_data.len(),
                    // Thanks to our use of `auto_release(true)` the packet buffer will be released
                    // once this field is dropped, and hence, once the `FrameRef` is dropped.
                    internal_ref: packet,
                };
                // We successfully aligned the frame data, and the CRC check passed, so we can
                // return the frame to the caller.
                critical_section::with(|cs| {
                    let mut stats = self.stats.borrow_ref_mut(cs);
                    stats.valid_frames_received += 1;
                });

                debug!(
                    "<<< RX {}",
                    debug_util::FormatEthernetFrame(&frame_ref.internal_ref)
                );
                return Some(frame_ref);
            } else {
                // Once we've depleted the MAC RX buffer we should indicate that the RX interrupt
                // can be turned on again to start receiving new packets, if it was previously
                // turned off because of the RX buffer being full.
                //
                // TODO(optimization): should we turn the RX interrupt on earlier? E.g. after the
                // very first packet has been consumed? Perhaps only once enough capacity is
                // available for a whole new frame? We don't have access to the available capacity
                // at this point though...
                (self.rx_queue_has_capacity_again_fn)();
                return None;
            }
        }
    }

    /// Attempts to align the given packet data by finding the SFD.
    ///
    /// Returns a slice to the aligned data incl. the FCS, or None if the data could not be aligned.
    fn align_packet<'b>(&self, packet_data: &'b mut [u8]) -> Option<&'b [u8]> {
        if log::log_enabled!(log::Level::Trace) {
            trace!("--- Unaligned packet from PHY, in network bit order");
            debug_util::log_data_binary(log::Level::Trace, packet_data);
        }

        // To align the data we need to find the SFD sequence in the data bit stream. It can occur
        // at any point in the stream, not just at octet boundaries.
        let sfd = bits![u8, Lsb0; 1, 0, 1, 0, 1, 0, 1, 1];
        // The farthest bit index at which the SFD can possibly start. The SFD must occur within the
        // first 8 octets of the data stream, in general. If the data stream is shorter than
        // MIN_FRAME_SIZE + 8, then we know the SFD must occur even earlier than that (since
        // otherwise there wouldn't be enough aligned data to make up a minimum-sized frame).
        let max_sfd_bit_idx = 8
            * (PREAMBLE_AND_SFD
                .len()
                .min(packet_data.len() - MIN_FRAME_SIZE));
        // The data we receive from the PHY is in LSB-received-first order.
        let data_bits = packet_data.view_bits_mut::<Lsb0>();
        // Find the SFD in the data bit stream, by looking at 8 bits at a time.
        let Some(sfd_bit_idx) = data_bits[..max_sfd_bit_idx]
            .windows(8)
            .position(|octet| octet == sfd)
        else {
            // If we couldn't find the SFD (and therefore the start of the frame data), then we've got
            // an invalid packet (either one without an SFD at all, or one with an SFD but where the
            // remaining data was truncated).
            debug!(
                "Invalid packet: could not find SFD (unaligned length: {}, max SFD bit idx: \
                    {max_sfd_bit_idx})",
                packet_data.len(),
            );
            return None;
        };
        // We found the SFD in the data, and hence we know where the frame data starts.
        let data_start_bit_idx = sfd_bit_idx + 8;

        // Shift the raw received data to the left so that the frame data (which starts right after
        // the SFD) is at bit/byte index 0 in the data buffer.
        data_bits.shift_left(data_start_bit_idx);

        // At this point the data buffer should have properly aligned frame data in it, but we still
        // have to determine where the frame data ends.
        //
        // The end index always falls on a full byte boundary, and we also know that the data buffer
        // may have contained some extra bits that correspond to the TP_IDL signal, rather than
        // actual frame data. There will never be more than two bits of that type of data in there
        // though.
        //
        // Hence, if we shifted the data by:
        // - 1 bit, then the last byte of data after shifting will contain a partial 7 bits, and
        //   should be discarded (those 7 bits will contain some TP_IDL signal and empty data).
        // - ...
        // - 7 bits, then the last byte of data after shifting will contain a partial single bit,
        //   and should be discarded (that single bit will contain some TP_IDL signal).
        // - 8 bits, then the last byte of data after shifting will contain a full 8 bits of data,
        //   and should be used (the PHY layer will not have passed us any of the TP_IDL signal, in
        //   this case).
        //
        // We can use this knowledge to infer the end of the frame data.
        let data_end_byte_idx = (data_bits.len() - data_start_bit_idx) / 8;
        trace!(
            "Start of frame data detected at bit index {data_start_bit_idx}, \
          data ends at aligned byte index {data_end_byte_idx} (0x{data_end_byte_idx:04x})"
        );

        let aligned_data = &packet_data[0..data_end_byte_idx];

        // If the slice we constructed is somehow shorter than a minimum frame size, then something
        // went wrong, because our SFD discovery mechanism above should only allow for aligned
        // frames of MIN_FRAME_SIZE or larger.
        assert!(aligned_data.len() >= MIN_FRAME_SIZE);
        Some(aligned_data)
    }

    /// Attempts to validate the given frame data using its FCS.
    ///
    /// Returns a slice to the validated frame data without the FCS, or None is the data could not
    /// be validated.
    fn validate_frame<'b>(&mut self, data: &'b [u8]) -> Option<&'b [u8]> {
        // Calculate the CRC over the frame data, and compare it with the received CRC.
        let calculated_crc = crc32fast::hash(&data[..data.len() - 4]);
        let received_crc: u32 = {
            let crc_data = &data[data.len() - 4..];
            (crc_data[3] as u32) << (8 * 3)
                | (crc_data[2] as u32) << (8 * 2)
                | (crc_data[1] as u32) << 8
                | (crc_data[0] as u32)
        };
        let crc_ok = calculated_crc == received_crc;

        // Log the aligned packet data for debugging purposes, using a higher log level if the CRC
        // didn't match.
        let aligned_packet_log_level = if crc_ok {
            log::Level::Trace
        } else {
            log::Level::Debug
        };
        if log::log_enabled!(aligned_packet_log_level) {
            log!(
                aligned_packet_log_level,
                "--- Aligned packet, in network bit order",
            );
            debug_util::log_data_binary_hex(aligned_packet_log_level, data);
        }

        // Log a diagnostic log for the received package, and indicate whether the CRC matched.
        log!(
            log::Level::Debug,
            "<<< RX {} CRC {} (calculated: {calculated_crc:08X}, received: {received_crc:08X})",
            debug_util::FormatEthernetFrame(data),
            if crc_ok { "ok" } else { "NOT OK!" },
        );

        if crc_ok {
            // Return the validated frame data, without the FCS.
            Some(&data[0..data.len() - FCS_SIZE])
        } else {
            None
        }
    }
}

impl<'a, const BUFFER_CAP: usize> PhyRxToMacRxBridge for MacRxPacketProducer<'a, BUFFER_CAP> {
    fn with_new_packet_buffer<F>(&mut self, buffer_fn: F) -> bool
    where
        F: FnOnce(&mut [u8]) -> usize,
    {
        // Try to reserve a large enough buffer.
        let Ok(mut grant) = self.packet_producer.grant(MAX_PACKET_SIZE) else {
            return false;
        };

        // Pass the buffer to the caller.
        let len = buffer_fn(&mut grant);

        // Commit however many bytes of the buffer that `buffer_fn` indicates.
        if len > 0 {
            grant.commit(len);
        }
        true
    }
}

/// A reference to frame data. Only once the reference is [dropped](Drop::drop) will the underlying
/// data buffer be released for reuse.
pub struct FrameRef<'a, const BUFFER_CAP: usize> {
    internal_ref: bbqueue::framed::FrameGrantR<'a, BUFFER_CAP>,
    length: usize,
}
impl<'a, const BUFFER_CAP: usize> FrameRef<'a, BUFFER_CAP> {
    /// Returns the frame data slice.
    pub fn data(&mut self) -> &mut [u8] {
        &mut self.internal_ref[0..self.length]
    }
}

/// Various receipt-related stats.
#[derive(Default, Debug, Clone, Copy)]
pub struct RxStats {
    /// The number of received packets that were invalid because they couldn't be aligned correctly,
    /// either because they didn't contain an SFD sequence, or not enough data was left after the
    /// SFD sequence to make up a minimum-sized frame.
    pub invalid_sfd_packets_received: u32,
    /// The number of received packets whose CRC was invalid.
    pub invalid_crc_packets_received: u32,
    /// The number of received packets that contained valid frames.
    pub valid_frames_received: u32,
}

// TODO: Add tests for MacRxPacketConsumer::receive (e.g. testing packet validation etc.).
// TODO: Add tests for transmit_frame (e.g. testing CRC generation etc.).
