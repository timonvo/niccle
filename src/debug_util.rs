//! Contains debug utilities that may be useful for users of this crate.

use bitvec::prelude::*;

/// A wrapper struct whose [core::fmt::Display] implementation prints the provided data in rows,
/// with each row containing 16 bytes of data in up to two groups of up to 8 bytes each,
/// with each byte formatted by the provided formatting function.
struct FormatDataInRowsAndGroups<'a, F> {
    data: &'a [u8],
    elem_formatter: F,
}
impl<'a, F, R> core::fmt::Display for FormatDataInRowsAndGroups<'a, F>
where
    F: Fn(&'a u8) -> R,
    R: core::fmt::Display,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (row_idx, row_values) in self.data.chunks(16).enumerate() {
            if row_idx > 0 {
                writeln!(f)?;
            }
            let offset = row_idx * 16;
            write!(f, "{offset:04x}: ")?;

            // Print values in groups of 8, each group separated by two spaces, and each element within
            // a group separated by a space.
            for (group_idx, group_values) in row_values.chunks(8).enumerate() {
                if group_idx > 0 {
                    write!(f, "  ")?;
                }
                for (elem_idx, elem) in group_values.iter().enumerate() {
                    if elem_idx > 0 {
                        write!(f, " ")?;
                    }
                    write!(f, "{}", (self.elem_formatter)(elem))?;
                }
            }
        }
        Ok(())
    }
}

fn log_data<'a, F, R>(log_level: log::Level, data: &'a [u8], fmt: F)
where
    F: Fn(&'a u8) -> R,
    R: core::fmt::Display,
{
    log::log!(
        log_level,
        "Length: {} bytes\n{}",
        data.len(),
        FormatDataInRowsAndGroups {
            data,
            elem_formatter: &fmt
        }
    );
}

/// A byte formatter that prints the value in hexadecimal format.
struct HexFormatter<'a>(&'a u8);
impl<'a> core::fmt::Display for HexFormatter<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:02x}", self.0)
    }
}

/// Logs the given data buffer in a human-readable format, with each byte printed in its hexadecimal
/// representation, similar to how Wireshark prints network packets.
pub fn log_data_hex(log_level: log::Level, data: &[u8]) {
    log_data(log_level, data, HexFormatter)
}

// Converts a byte which would be transmitted from LSB-to-MSB, to an equivalent byte where the
// transmission order would be MSB-to-LSB.
fn in_network_bit_order(value: u8) -> u8 {
    let mut network_order_elem = value;
    network_order_elem.view_bits_mut::<Lsb0>().reverse();
    network_order_elem.as_raw_slice()[0]
}

/// A byte formatter that prints the value in network bit order binary format.
struct BinaryFormatter<'a>(&'a u8);
impl<'a> core::fmt::Display for BinaryFormatter<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:08b}", in_network_bit_order(*self.0))
    }
}

/// Logs the given data buffer in a human-readable format, with each byte printed in its binary
/// representation, similar to how Wireshark prints network packets. Note that the provided data is
/// assumed to be transmitted from LSB-to-MSB, but that the printed binary representation will use
/// the network bit order: the most significant (rightmost) printed bit will be the bit that would
/// be transmitted first. This ensures that the printed bit stream can easily be read from left to
/// right.
pub fn log_data_binary(log_level: log::Level, data: &[u8]) {
    log_data(log_level, data, BinaryFormatter)
}
/// A byte formatter that prints the value in both network bit order binary as well as hexadecimal
/// format.
struct BinaryAndHexFormatter<'a>(&'a u8);
impl<'a> core::fmt::Display for BinaryAndHexFormatter<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:08b} ({:02x})", in_network_bit_order(*self.0), self.0)
    }
}

/// Logs the given data buffer in a human-readable format, with each byte printed in both its binary
/// and hexadecimal representation, similar to how Wireshark prints network packets. Note that the
/// provided data is assumed to be transmitted from LSB-to-MSB, but that the printed binary
/// representation will use the network bit order (see [log_data_binary] for more info).
pub fn log_data_binary_hex(log_level: log::Level, data: &[u8]) {
    log_data(log_level, data, BinaryAndHexFormatter)
}

pub struct FormatEthernetFrame<'a>(pub &'a [u8]);
impl<'a> core::fmt::Display for FormatEthernetFrame<'a> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.0.len() < 14 {
            panic!("Frame data too short!")
        }

        let data = self.0;
        write!(
            f,
            "EthFrame(len: {}, type 0x{:02X}{:02X}, \
                dst {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}, \
                src {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X})",
            data.len(),
            data[12],
            data[13],
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
            data[6],
            data[7],
            data[8],
            data[9],
            data[10],
            data[11],
        )?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Tests the hexadecimal output format.
    #[test]
    fn format_hex_two_rows_four_groups() {
        let test_data = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
            0x1c, 0x1d, 0x1e, 0x1f,
        ];
        assert_eq!(
            "0000: 00 01 02 03 04 05 06 07  08 09 0a 0b 0c 0d 0e 0f\n\
             0010: 10 11 12 13 14 15 16 17  18 19 1a 1b 1c 1d 1e 1f",
            FormatDataInRowsAndGroups {
                data: &test_data,
                elem_formatter: HexFormatter
            }
            .to_string()
        );
    }

    // Tests the binary output format.
    #[test]
    fn format_hex_two_rows_three_groups() {
        let test_data = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
            0x0e, 0x0f, 0x10, 0x11, 0x12,
        ];
        assert_eq!(
            "0000: 00000000 10000000 01000000 11000000 00100000 10100000 01100000 11100000  \
                   00010000 10010000 01010000 11010000 00110000 10110000 01110000 11110000\n\
             0010: 00001000 10001000 01001000",
            FormatDataInRowsAndGroups {
                data: &test_data,
                elem_formatter: BinaryFormatter
            }
            .to_string()
        );
    }

    // Tests the binary + hexadecimal output format.
    #[test]
    fn format_binary_and_hex_one_row_two_groups() {
        let test_data = [
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
        ];
        assert_eq!(
            "0000: 00000000 (00) 10000000 (01) 01000000 (02) 11000000 (03) 00100000 (04) 10100000 (05) 01100000 (06) 11100000 (07)  \
                   00010000 (08) 10010000 (09) 01010000 (0a) 11010000 (0b) 00110000 (0c) 10110000 (0d)",
            FormatDataInRowsAndGroups {
                data: &test_data,
                elem_formatter: BinaryAndHexFormatter
            }
            .to_string()
        );
    }
}
