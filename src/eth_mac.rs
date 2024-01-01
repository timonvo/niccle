//! Implements the functionality of an Ethernet 10BASE-T MAC layer, like preparing outgoing Ethernet
//! packets and passing them to the PHY layer, and validating incoming packets' checksums.

use crate::debug_util;
use bitvec::prelude::*;
use core::cell::RefCell;
use crc32fast;
use critical_section::Mutex;
use log::{debug, log, trace, warn};

/// The minimum size of an Ethernet frame. 64 equals 12 bytes of destination and source addresses, 2
/// bytes of EtherType, 46 bytes of payload, and 4 bytes of FCS (we don't support 802.1q-tagged
/// frames).
pub const MIN_FRAME_SIZE: usize = 64;
/// The maximum size of an Ethernet frame. 1518 equals 12 bytes of destination and source addresses,
/// 2 bytes of EtherType, 1500 bytes of payload, and 4 bytes of FCS (we don't support 802.1q-tagged
/// frames).
pub const MAX_FRAME_SIZE: usize = 1518;
/// The maximum size of an Ethernet packet (8 bytes of preamble & SFD plus the frame data).
pub const MAX_PACKET_SIZE: usize = MAX_FRAME_SIZE + 8;

/// A trait that the PHY layer should implement to allow the MAC layer to transmit outgoing packets.
pub trait PhyTx<'a> {
    /// Transmits the given packet, blocking until the transmission has finished.
    fn transmit_packet(&self, data: &[u8]);
}

/// A trait implemented by the MAC layer to allow the PHY layer to pass received packets to the MAC
/// layer.
pub trait PhyRxCallback {
    /// Should be called when a packet has been received at the PHY layer. The `data` slice must
    /// contain part of the preamble, the SFD, the frame data, and up to two bits of TP_IDL signal.
    /// Each byte in the slice must contain at least one bit of packet data (i.e. there must be no
    /// trailing bytes of bogus data). The callee is responsible for aligning and validating the
    /// data.
    ///
    /// Note: since this code may be invoked from within an interrupt routine it should ideally not
    /// block for very long.
    fn on_packet_received(&self, data: &mut [u8]);
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

    /// Transmits the given frame, blocking until the transmission has finished.
    ///
    /// The provided data may or may not be padded already, but must not contain an FCS. A preamble
    /// and SFD will be prepended, and the FCS will be calculated by this method before it passes
    /// the data on to the PHY layer for transmission.
    pub fn transmit_frame(&mut self, frame_data: &[u8]) {
        const DATA_OFFSET: usize = PREAMBLE_AND_SFD.len();
        const FCS_LEN: usize = 4;

        // Get a subslice of the packet buffer into which we'll write the frame data, padding, and FCS.
        let frame_buffer = &mut self.packet_buffer[DATA_OFFSET..]
            [..(frame_data.len() + FCS_LEN).max(MIN_FRAME_SIZE)];

        // Copy the unpadded data into the frame slice.
        frame_buffer[..frame_data.len()].copy_from_slice(frame_data);

        // Add padding to the frame slice.
        let padding = &mut frame_buffer[frame_data.len()..MIN_FRAME_SIZE - FCS_LEN];
        padding.fill(0);
        trace!("TX adding {} bytes of padding", padding.len());

        // Calculate the CRc and place it at the end of the frame slice.
        let fcs_offset = frame_buffer.len() - FCS_LEN;
        let fcs = crc32fast::hash(&frame_buffer[0..fcs_offset]);
        frame_buffer[fcs_offset] = (fcs & 0xFF) as u8;
        frame_buffer[fcs_offset + 1] = ((fcs >> 8) & 0xFF) as u8;
        frame_buffer[fcs_offset + 2] = ((fcs >> (8 * 2)) & 0xFF) as u8;
        frame_buffer[fcs_offset + 3] = ((fcs >> (8 * 3)) & 0xFF) as u8;

        debug!(">>> TX {}", debug_util::FormatEthernetFrame(frame_buffer));
        debug_util::log_data_hex(log::Level::Trace, frame_buffer);

        let packet_len = DATA_OFFSET + frame_buffer.len();
        self.phy.transmit_packet(&self.packet_buffer[0..packet_len]);
    }
}

/// A MAC layer implementation for receiving incoming Ethernet frames.
///
/// This implementation will buffer up to `BUFFER_CAP` frames, which can then be consumed by calling
/// [MacRx::receive]. If the frame is full then any further incoming frames will be dropped until
/// one of the previously received frames has been consumed.
pub struct MacRx<const BUFFER_CAP: usize> {
    rx_inbox: RxInbox<BUFFER_CAP>,
    stats: Mutex<RefCell<RxStats>>,
}

impl<const BUFFER_CAP: usize> MacRx<BUFFER_CAP> {
    /// Constructs a new [MacRx] instance.
    pub const fn new() -> MacRx<BUFFER_CAP> {
        MacRx {
            rx_inbox: RxInbox::new(),
            // We can't use Default::default() since we're in a `const fn`.`
            stats: Mutex::new(RefCell::new(RxStats {
                invalid_sfd_packets_received: 0,
                invalid_crc_packets_received: 0,
                valid_frames_received: 0,
                valid_frames_dropped: 0,
                valid_frames_consumed: 0,
            })),
        }
    }

    /// Consumes one of the previously-received frames, if there is one. Note that the buffer
    /// holding the frame data will not be released until the returned [FrameRef] is
    /// [dropped][Drop::drop].
    pub fn receive(&self) -> Option<FrameRef> {
        let frame = self.rx_inbox.pop();
        if frame.is_some() {
            critical_section::with(|cs| self.stats.borrow_ref_mut(cs).valid_frames_consumed += 1)
        }
        frame
    }

    /// Returns stats about this instance's activity so far.
    pub fn stats(&self) -> RxStats {
        critical_section::with(|cs| *self.stats.borrow_ref(cs))
    }

    /// Attempts to align the given packet data by finding the SFD.
    ///
    /// Returns a slice to the aligned data incl. the FCS, or None if the data could not be aligned.
    fn align_packet<'a>(&self, packet_data: &'a mut [u8]) -> Option<&'a [u8]> {
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
        let max_sfd_bit_idx = 8 * (PREAMBLE_AND_SFD.len().min(packet_data.len() - MIN_FRAME_SIZE));
        // The data we receive from the PHY is in LSB-received-first order.
        let data_bits = packet_data.view_bits_mut::<Lsb0>();
        // Find the SFD in the data bit stream, by looking at 8 bits at a time.
        let sfd_bit_idx = data_bits[..max_sfd_bit_idx]
            .windows(8)
            .position(|octet| octet == sfd);
        // If we couldn't find the SFD (and therefore the start of the frame data), then we've got
        // an invalid packet (either one without an SFD at all, or one with an SFD but where the
        // remaining data was truncated).
        if sfd_bit_idx.is_none() {
            warn!(
                "Invalid packet: could not find SFD (unaligned length: {}, max SFD bit idx: \
                    {max_sfd_bit_idx})",
                packet_data.len(),
            );
            return None;
        }
        // We found the SFD in the data, and hence we know where the frame data starts.
        let data_start_bit_idx = sfd_bit_idx.unwrap() + 8;

        // Shift the raw received data to the left so that the frame data (which starts right after
        // the SFD) is at bit/byte index 0 in the data buffer.
        //
        // TODO(optimization): Instead of shifting the data here and then copying the aligned data
        // into the RxInbox entry later on, we could instead reserve an RxInbox buffer entry first,
        // and then shift the aligned data into that buffer directly. That would reduce the number
        // of data copies by one.
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
    fn validate_frame<'a>(&self, data: &'a [u8]) -> Option<&'a [u8]> {
        // Calculate the CRC over the frame data, and compare it with the received CRC.
        let calculated_crc = crc32fast::hash(&data[0..data.len() - 4]);
        let received_crc: u32 = (data[data.len() - 1] as u32) << (8 * 3)
            | (data[data.len() - 2] as u32) << (8 * 2)
            | (data[data.len() - 3] as u32) << 8
            | (data[data.len() - 4] as u32);
        let crc_ok = calculated_crc == received_crc;

        // Log the aligned packet data for debugging purposes, using a higher log level if the CRC
        // didn't match.
        let aligned_packet_log_level = if crc_ok {
            log::Level::Trace
        } else {
            log::Level::Warn
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
            if crc_ok {
                log::Level::Debug
            } else {
                log::Level::Warn
            },
            "<<< RX {} CRC {} (calculated: {calculated_crc:08X}, received: {received_crc:08X})",
            debug_util::FormatEthernetFrame(data),
            if crc_ok { "ok" } else { "NOT OK!" },
        );

        if crc_ok {
            // Return the validated frame data, without the FCS.
            Some(&data[0..data.len() - 4])
        } else {
            None
        }
    }
}

impl<const BUFFER_CAP: usize> PhyRxCallback for MacRx<BUFFER_CAP> {
    /// Handles (potential) packets received by the PHY layer.
    ///
    /// Strips the preamble and SFD, aligns the frame data, performs FCS validation, and stores
    /// valid frames away for later consumption.
    fn on_packet_received(&self, packet_data: &mut [u8]) {
        let aligned_frame_data = self.align_packet(packet_data);
        if aligned_frame_data.is_none() {
            critical_section::with(|cs| {
                self.stats.borrow_ref_mut(cs).invalid_sfd_packets_received += 1;
            });
            return;
        }
        let validated_frame_data = self.validate_frame(aligned_frame_data.unwrap());
        if validated_frame_data.is_none() {
            critical_section::with(|cs| {
                self.stats.borrow_ref_mut(cs).invalid_crc_packets_received += 1;
            });
            return;
        }
        // We successfully aligned the frame data, and the CRC check passed, so we must store the
        // frame away for later consumption.
        critical_section::with(|cs| {
            let mut stats = self.stats.borrow_ref_mut(cs);
            stats.valid_frames_received += 1;

            // If we can't add the frame to the inbox then we have no choice but to drop it.
            if !self.rx_inbox.push(validated_frame_data.unwrap()) {
                // TODO: Consider sending a pause frame here, to throttle the rate of incoming data
                // down to what we are able to process.
                stats.valid_frames_dropped += 1;
                warn!("<<< Dropped RX frame");
            }
        });
    }
}

/// A container holding received frames.
///
/// This implementation can hold up to `BUFFER_CAP` received frames, and returns frames in FIFO
/// order.
struct RxInbox<const BUFFER_CAP: usize> {
    // We could've also used a `heapless::Deque` here, but StaticThingBuf allows us to avoid copies
    // through its `push_ref` and `pop_ref` methods.
    //
    // TODO(optimization): Instead of a queue of a fixed number of constant-sized frames, we could
    // use another data structure that pools all frame data in one large buffer (like a ring
    // buffer), and allocates only as much data as needed from the buffer for each frame. That could
    // allow us to queue many small packets without incurring 1518 bytes of memory per packet.
    frames: thingbuf::StaticThingBuf<FrameEntry, BUFFER_CAP, FrameRecycle>,
}

impl<const BUFFER_CAP: usize> RxInbox<BUFFER_CAP> {
    const fn new() -> RxInbox<BUFFER_CAP> {
        RxInbox {
            frames: thingbuf::StaticThingBuf::with_recycle(FrameRecycle),
        }
    }

    /// Pushes a new frame into the back of the frame queue. Returns true if the operation
    /// succeeded, or false if it failed because there was no room for the frame.
    fn push(&self, data: &[u8]) -> bool {
        // Grab a slot for the buffer, if there's room.
        let Ok(mut entry) = self.frames.push_ref() else {
            return false;
        };

        // Copy the frame data into the inbox buffer.
        entry.length = data.len();
        entry.data[..data.len()].copy_from_slice(data);
        true
    }

    /// Pops a frame from the front of the frame queue, if one is available.
    fn pop(&self) -> Option<FrameRef> {
        self.frames.pop_ref().map(|entry| FrameRef {
            internal_ref: entry,
        })
    }
}

/// Frame data held in an underlying buffer. The buffer may be larger than the length of frame data.
struct FrameEntry {
    data: [u8; MAX_FRAME_SIZE],
    length: usize,
}
impl FrameEntry {
    /// Return the slice of the buffer that actually holds valid frame data.
    fn live_slice(&self) -> &[u8] {
        &self.data[..self.length]
    }
}

/// A reference to frame data. Only once the reference is [dropped](Drop::drop) will the underlying
/// data buffer be released for reuse.
pub struct FrameRef<'a> {
    internal_ref: thingbuf::Ref<'a, FrameEntry>,
}
impl<'a> FrameRef<'a> {
    /// Returns a slice containing the frame data.
    pub fn live_data(&self) -> &[u8] {
        self.internal_ref.live_slice()
    }
}

/// A [thingbuf] recycling strategy for reusing [FrameEntry] instances, which avoids unnecessary
/// copies.
///
/// We'll never read more data from a [FrameEntry] than was written to it, so we never actually have
/// to clear or reset the underlying data buffer (which the default
/// [thingbuf::recycling::DefaultRecycle] strategy would do, wasting unnecessary CPU cycles in the
/// process).
struct FrameRecycle;
impl thingbuf::Recycle<FrameEntry> for FrameRecycle {
    fn new_element(&self) -> FrameEntry {
        FrameEntry {
            data: [0u8; MAX_FRAME_SIZE],
            length: 0,
        }
    }

    fn recycle(&self, _: &mut FrameEntry) {
        // Do nothing.
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
    /// The number of valid frames reported in `valid_frames_received` that had to be dropped
    /// because there was no buffer space.
    pub valid_frames_dropped: u32,
    /// The number of valid frames reported in `valid_frames_received` that were later successfully
    /// consumed.
    pub valid_frames_consumed: u32,
}

// TODO: Add tests for on_packet_received.
// TODO: Add tests for transmit_frame.
