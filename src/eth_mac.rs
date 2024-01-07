//! Implements the functionality of an Ethernet 10BASE-T MAC layer, like preparing outgoing Ethernet
//! packets and passing them to the PHY layer, and validating incoming packets' checksums.

use crate::debug_util;
use bitvec::prelude::*;
use crc32fast;
use log::{debug, log, warn};

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

/// A very basic implementation of MAC layer functionality for handling a packet received by the
/// PHY.
///
/// Strips the preamble and SFD, aligns the frame data, perform CRC validation, and logs diagnostic
/// info.
///
/// Note: this code is invoked from within an interrupt routine and should ideally not block for too
/// long.
pub fn on_rx_packet_received(data: &mut [u8]) {
    if log::log_enabled!(log::Level::Debug) {
        debug!("--- Unaligned packet from PHY, in network bit order");
        debug_util::log_data_binary(log::Level::Debug, data);
    }

    // Find the SFD by iterating bit by bit through the data (the SFD sequence could happen to align
    // with a byte boundary, but also could span two sequential bytes).
    let mut data_start_bit_idx = None;
    // The data we receive from the PHY is in LSB-received-first order.
    let result_bv = data.view_bits_mut::<Lsb0>();
    // The SFD pattern we're looking for.
    let sfd = bits![u8, Lsb0; 1, 0, 1, 0, 1, 0, 1, 1];
    for bit_idx in 0..result_bv.len() - 8 {
        let val = &result_bv[bit_idx..bit_idx + 8];
        // At each index check whether we see the SFD pattern. If we've found the pattern then set
        // the inferred bit index of the start of the frame data and terminate the loop.
        if val == sfd {
            data_start_bit_idx = Some(bit_idx + 8);
            break;
        }
    }

    // If we couldn't find the SFD (and therefore the start of the frame data), then we've got an
    // invalid packet.
    if data_start_bit_idx.is_none() {
        warn!(
            "Invalid packet: could not find SFD (unaligned length: {})",
            data.len(),
        );
        return;
    }
    // We found the SFD in the data, and hence we know where the frame data starts.
    let data_start_bit_idx = data_start_bit_idx.unwrap();
    // Shift the raw received data to the left so that the frame data (which starts right after the
    // SFD) is at bit/byte index 0 in the data buffer.
    result_bv.shift_left(data_start_bit_idx);

    // At this point the data buffer should have properly aligned frame data in it, but we still
    // have to determine where the frame data ends.
    //
    // The end index always falls on a full byte boundary, and we also know that the data buffer may
    // have contained some extra bits that correspond to the TP_IDL signal, rather than actual frame
    // data. There will never be more than two bits of that type of data in there though.
    //
    // Hence, if we shifted the data by:
    // - 1 bit, then the last byte of data after shifting will contain a partial 7 bits, and should
    //   be discarded (those 7 bits will contain some TP_IDL signal and empty data).
    // - ...
    // - 7 bits, then the last byte of data after shifting will contain a partial single bit, and
    //   should be discarded (that single bit will contain some TP_IDL signal).
    // - 8 bits, then the last byte of data after shifting will contain a full 8 bits of data, and
    //   should be used (the PHY layer will not have passed us any of the TP_IDL signal, in this
    //   case).
    //
    // We can use this knowledge to infer the end of the frame data.
    let data_end_byte_idx = (result_bv.len() - data_start_bit_idx) / 8;
    debug!(
        "Start of frame data detected at bit index {data_start_bit_idx}, \
          data ends at aligned byte index {data_end_byte_idx} (0x{data_end_byte_idx:04x})"
    );

    // We can now construct a slice containing exactly the frame data (incl. the FCS).
    let aligned_data = &data[0..data_end_byte_idx];
    // If the slice we inferred is shorter than a minimum frame size, then we must have received an
    // invalid packet that the PHY did not detect as invalid yet.
    if aligned_data.len() < MIN_FRAME_SIZE {
        warn!(
            "Invalid packet: not enough data for a full frame (unaligned length: {}, \
              data start bit index: {data_start_bit_idx}, aligned length: {})",
            data.len(),
            aligned_data.len()
        );
        return;
    }

    // Calculate the CRC over the frame data, and compare it with the received CRC.
    let calculated_crc = crc32fast::hash(&aligned_data[0..aligned_data.len() - 4]);
    let received_crc: u32 = (aligned_data[aligned_data.len() - 1] as u32) << (8 * 3)
        | (aligned_data[aligned_data.len() - 2] as u32) << (8 * 2)
        | (aligned_data[aligned_data.len() - 3] as u32) << 8
        | (aligned_data[aligned_data.len() - 4] as u32);
    let crc_ok = calculated_crc == received_crc;

    // Log the aligned packet data for debugging purposes, using a higher log level if the CRC
    // didn't match.
    if !crc_ok || log::log_enabled!(log::Level::Debug) {
        let log_level = if crc_ok {
            log::Level::Debug
        } else {
            log::Level::Info
        };
        log!(log_level, "--- Aligned packet, in network bit order",);
        debug_util::log_data_binary_hex(log_level, aligned_data);
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

    // TODO: Expand this implementation to actually store valid frames for later consumption by a
    // higher layer (e.g. a TCP implementation). Also make this implementation more lighter weight
    // so it's more appropriate to invoke from an interrupt routine (e.g. by avoiding emitting any
    // logs under normal circumstances).
}
