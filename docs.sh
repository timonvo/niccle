#!/bin/sh

set -e # Fail on first error
set -x # Print the commands run

# Enter the top-level directory so we always execute commands from there.
cd $(dirname $0)

# Run `cargo doc`, failing if we hit any warnings.
#
# Note: you can use "./docs.sh --open" to open the generated docs.
RUSTDOCFLAGS="-D warnings" \
cargo doc --target riscv32imac-unknown-none-elf \
    --document-private-items --workspace --lib --examples --bins --no-deps $@
