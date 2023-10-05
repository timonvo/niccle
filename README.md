# Niccle, a bit-banged Ethernet implementation in Rust.

This is a hobby project. See https://ctrlsrc.io/posts/2023/niccle-project-intro/
for more info.

## Code structure

| Directory    | Description                                                       |
| ------------ | ----------------------------------------------------------------- |
| `<toplevel>` | Contains chipset-agnostic code (which can run on ESP32, ARM etc.) |
| `esp/`       | Contains ESP32-specific code.                                     |

## Formatting the code

We try to use `cargo fix --workspace` and `taplo fmt`
(`cargo install taplo-cli`) to keep the code and .toml files formatted
consistently.

## Running on ESP32C6

The ESP32-related code focuses on RISC-V-based chipsets (currently the ESP32-C6
chip in particular), and uses the `esp-hal` crate (rather than `esp-idf-hal`).

To run the code, run the following command from the top-level repo directory:

```sh
 cargo run -p niccle_esp --release --target riscv32imac-unknown-none-elf --example {some_example}
```

Or, run

```sh
 cargo run --release --target riscv32imac-unknown-none-elf --example {some_example}
```

from within the `esp` directory.
