# corrode86

An 8086 instruction decoder/disassembler and simulator written in Rust.

![Asteroids Demo](docs/asteroids_demo.gif)

## Features

- Disassembles 8086 binary files
- Simulates instruction execution with register state tracking
- Verifies disassembly matches original assembly source
- Memory-mapped video display (text mode 80x25) with terminal output
- Simple text-mode video with ANSI colors

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

The emulator includes a simple text-mode video display that maps memory addresses 2000-3999 to an 80x25 character display. The video output is rendered in the terminal using ANSI escape codes for colors:

- White text for letters (A-Z)
- Yellow for asterisks (*)
- Green for the ship (^)

Memory mapping: Address = Row * 80 + Column + 2000

## Implemented Instructions

- MOV (register/memory/immediate)
- ADD, SUB (arithmetic operations)
- CMP (comparison)
- JMP, JLE, JL, JNE (jumps)
- INC, DEC (increment/decrement)
- HLT (halt)

## TODO

- **Implement proper segmentation**: Add segment:offset addressing to access real VGA text mode memory at 0xB8000 (currently using simplified memory mapping at addresses 100-2099)
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
