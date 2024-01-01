#![no_std]
// We use this feature to write inline assembly with readable register names.
#![feature(asm_const)]

// This crate contains ESP chipset-specific code. None of this code is compatible with the ESP32-C3
// chip right now.

#[cfg(not(feature="esp32c3"))]
pub mod eth_phy;
#[cfg(not(feature="esp32c3"))]
mod eth_phy_dedicated_io;
