#![no_std]
// We use this feature to write inline assembly with readable register names.
#![feature(asm_const)]

// This crate contains ESP chipset-specific code.

pub mod eth_phy;
mod eth_phy_dedicated_io;
