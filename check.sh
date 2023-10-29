#!/bin/sh

set -e # Fail on first error
set -x # Print the commands run

# Enter the top-level directory so we always execute commands from there.
cd $(dirname $0)

# Ensure everything builds by default.
cargo build --release --workspace --all-targets

# Ensure everything builds in the `niccle_esp` package when targeting the
# ESP32-C3 chip.
cargo build -p niccle_esp --release --all-targets \
    --target riscv32imc-unknown-none-elf \
    --no-default-features --features esp32c3

# Run all tests.
cargo test --all-targets

# Run cargo check on everything.
cargo check --workspace --all-targets
cargo check -p niccle_esp --all-targets \
    --target riscv32imc-unknown-none-elf \
    --no-default-features --features esp32c3

# Run cargo clippy.
cargo clippy --workspace --all-targets -- -Dwarnings
cargo clippy -p niccle_esp --all-targets \
    --target riscv32imc-unknown-none-elf \
    --no-default-features --features esp32c3  \
    -- -Dwarnings

echo "SUCCESS!"
