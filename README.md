# corrode86

An 8086 instruction decoder/disassembler and simulator written in Rust.

![Asteroids Demo](docs/asteroids_demo.gif)

## Features

- **Complete 8086 segmented memory model** with 1MB address space and segment:offset addressing
- **Video memory mapping** at physical address 0xB8000 with terminal output
- **Full instruction simulation** with register state tracking and flag handling
- **Binary disassembly** with segment register display and source verification

## Quick Start

```bash
# Build the project
cargo build --release

# Disassemble a binary file
./target/release/simx86 asm/calc_add.bin

# Execute a binary file
./target/release/simx86 asm/calc_add.bin --execute

# Run the animated asteroids demo with video display
./target/release/simx86 asm/asteroids_simple.bin --execute --video

# Assemble and run any assembly file
nasm -f bin asm/asteroids_simple.asm -o asm/asteroids_simple.bin
./target/release/simx86 asm/asteroids_simple.bin --execute --video
```

## Project Structure

- `asm/` - Assembly source files
  - `calc_*.asm` - Basic arithmetic calculation examples
  - `asteroids_simple.asm` - Animated demo with moving ship
- `src/` - Rust source code
  - `cpu.rs` - CPU emulation and instruction decoding
  - `main.rs` - CLI interface
  - `video.rs` - Terminal-based video display with ANSI colors
  - `lib.rs` - Library interface

## Video Display

The emulator maps video memory at physical address 0xB8000 (standard text mode location). Characters are displayed in an 80x25 terminal window using ANSI escape codes for colors:

- White text for letters (A-Z)
- Yellow for asterisks (*)
- Green for the ship (^)

Video memory layout: Physical Address = 0xB8000 + (Row * 80 + Column) * 2

## Implemented Instructions

- MOV (register/memory/immediate)
- ADD, SUB (arithmetic operations)
- CMP (comparison)
- JMP, JLE, JL, JNE (jumps)
- INC, DEC (increment/decrement)
- HLT (halt)

## TODO
- **Graphical video display**: Add option to use minifb or SDL2 for a graphical window display instead of terminal output
- **MUL/DIV instructions**: Implement multiplication and division operations (MUL, IMUL, DIV, IDIV)
- **Keyboard input**: Implement keyboard handling via:
  - Memory-mapped keyboard buffer (e.g., at address 0x60 for scan codes)
  - INT 16h BIOS keyboard services
  - Direct port I/O (IN/OUT instructions for ports 0x60, 0x64)
- **More CPU instructions**: Implement additional 8086 instructions as needed (especially IN/OUT for port I/O)
- **Interrupt handling**: Add support for BIOS and DOS interrupts (INT instruction)

## Reference

- [8086 User's Manual](https://codeberg.org/bolt/8086-Users-Manual.git)
- [OSDev](https://wiki.osdev.org/)
