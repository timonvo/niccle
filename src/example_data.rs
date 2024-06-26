//! Provides fake data for use in tests or example binaries.

/// A test frame containing an ARP request, without padding or FCS. We define it as a struct to
/// improve readability. The final padded frame will be 64 bytes long, the smallest size an Ethernet
/// frame can have.
// The `packed` may be redundant, since we only use u8 arrays which will already be
// single-byte-aligned, but let's add it for good measure.
#[repr(C, packed)]
pub struct EthernetArpFrame {
    _dst: [u8; 6],
    _src: [u8; 6],
    _ethertype: [u8; 2],
    // ARP payload fields start here.
    _hardware_type: [u8; 2],
    _protocol_type: [u8; 2],
    _hardware_len: [u8; 1],
    _protocol_len: [u8; 1],
    _arp_type: [u8; 2],
    _sender_hardware_addr: [u8; 6],
    _sender_protocol_addr: [u8; 4],
    _target_hardware_addr: [u8; 6],
    _target_protocol_addr: [u8; 4],
}
impl EthernetArpFrame {
    /// Returns an immutable slice of bytes containing the data in this frame.
    pub const fn as_raw_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                (self as *const EthernetArpFrame) as *const u8,
                core::mem::size_of::<EthernetArpFrame>(),
            )
        }
    }
}

/// A test Ethernet frame containing an ARP request. See [TEST_FRAME_ARP_REQUEST_RAW].
const EXAMPLE_FRAME_ARP_REQUEST: EthernetArpFrame = EthernetArpFrame {
    _dst: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF], // This is the broadcast MAC address.
    _src: [0x12, 0x34, 0x56, 0x78, 0x90, 0x12], // This is just a made up source MAC address.
    _ethertype: [0x08, 0x06],                   // 0x806 == ARP
    _hardware_type: [0x00, 0x01],               // 0x1 == Ethernet
    _protocol_type: [0x08, 0x00],               // 0x800 == IPv4
    _hardware_len: [0x06],
    _protocol_len: [0x04],
    _arp_type: [0x00, 0x01], // 0x1 == ARP request
    _sender_hardware_addr: [0x12, 0x34, 0x56, 0x78, 0x90, 0x12], // This is the same as _src.
    _sender_protocol_addr: [0xA9, 0xFE, 0xAC, 0x72], // 169.254.172.114
    _target_hardware_addr: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    _target_protocol_addr: [0xA9, 0xFE, 0xAC, 0x73], // 169.254.172.115
};
/// A test Ethernet frame containing an ARP request asking for the MAC address of the device with
/// link-local IPv4 address 169.254.172.115, sent by a device with MAC address 12:34:56:78:90:12 (a
/// "locally-administered" address) and IPv4 address 169.254.172.114.
pub const TEST_FRAME_ARP_REQUEST_RAW: &[u8] = EXAMPLE_FRAME_ARP_REQUEST.as_raw_bytes();
