# Niccle, a bit-banged Ethernet implementation in Rust.

## Code structure

| Directory    | Description                                                       |
| ------------ | ----------------------------------------------------------------- |
| `<toplevel>` | Contains chipset-agnostic code (which can run on ESP32, ARM etc.) |
| `esp/`       | Contains ESP32-specific code.                                     |


## Running on ESP32C6
The ESP32-related code focuses on RISC-V-based chipsets, and uses the `esp-hal` crate (rather than `esp-idf-hal`).

To run the code, run the following command from the top-level repo directory:

```sh
cargo run -p niccle_esp
```

Or, run

```sh
cargo run
```

from within the `esp` directory.