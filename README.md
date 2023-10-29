# Niccle, a bit-banged Ethernet implementation in Rust.

This is a hobby project. See https://ctrlsrc.io/posts/2023/niccle-project-intro/ for more info.
Currently this project targets only the ESP32-C6 chip.

## Code structure

| Directory      | Description                                                       |
| -------------- | ----------------------------------------------------------------- |
| `<toplevel>`   | Contains chipset-agnostic code (which can run on ESP32, ARM etc.) |
| `esp/`         | Contains ESP32-specific code.                                     |
| `proc_macros/` | Contains procedural macros used by the rest of the code.          |

## Formatting the code

I try to use the code and .toml files formatted with `cargo fix` and `taplo fmt`
(`cargo install taplo-cli`), using the `fix.sh` script.

## Running on ESP32-C6

The ESP32-related code focuses on RISC-V-based chipsets, and the ESP32-C6 chip
in particular. It uses the `esp-hal` crate (rather than `esp-idf-hal`).

To run the code, run the following command from the top-level repo directory:

```sh
cargo run -p niccle_esp --release --target riscv32imac-unknown-none-elf --example {some_example}
```

Or, run

```sh
cargo run --release --target riscv32imac-unknown-none-elf --example {some_example}
```

from within the `esp` directory.

## Running on ESP32-C3

Some but not all of the code (e.g. the
[cycle_counting](esp/examples/cycle_counting.rs) example) can compile and run on
ESP32-C3 as well. To target that chip, use the following command:

```sh
cargo run --release --target riscv32imc-unknown-none-elf --no-default-features --feature esp32c3 \
   --example {some_example}
```
