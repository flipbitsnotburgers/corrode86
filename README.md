# corrode86

An 8086 instruction decoder/disassembler and simulator written in Rust.

## Features

- Disassembles 8086 binary files
- Simulates instruction execution with register state tracking
- Verifies disassembly matches original assembly source

## Quick Start

```bash
# Build and run all test programs
make run-all

# Run a specific assembly file
make run FILE=asm/calc_add.asm

# Clean build artifacts
make clean
```

## Project Structure

- `asm/` - Assembly source files
- `src/` - Rust source code
  - `cpu.rs` - CPU emulation and instruction decoding
  - `main.rs` - CLI interface
- `build_and_run.py` - Build and verification script

## Reference

[8086 User's Manual](https://codeberg.org/bolt/8086-Users-Manual.git)
