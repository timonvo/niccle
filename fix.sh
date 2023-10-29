#!/bin/sh

set -e # Fail on first error
set -x # Print the commands run

# Run cargo fix.
cargo fix --workspace
cargo fix -p niccle_esp \
    --target riscv32imc-unknown-none-elf \
    --no-default-features --features esp32c3

# Run cargo clippy.
cargo clippy --workspace --all-targets --fix -- -Dwarnings
cargo clippy -p niccle_esp --all-targets --fix \
    --target riscv32imc-unknown-none-elf \
    --no-default-features --features esp32c3  \
    -- -Dwarnings

# Format TOML files (`cargo install taplo-cli`).
taplo fmt