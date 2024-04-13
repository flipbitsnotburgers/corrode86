#!/usr/bin/env python3

import subprocess
import sys
import difflib
from pathlib import Path

def shell_run(cmd, capture_output=False):
    """Run a shell command and optionally capture output"""
    if isinstance(cmd, str):
        cmd = cmd.split()
    
    if capture_output:
        result = subprocess.run(cmd, capture_output=True, text=True)
        return result.stdout.strip()
    else:
        subprocess.run(cmd, check=True)

def extract_asm_instructions(asm_file):
    """Extract instructions between ;<start> and ;<end> markers"""
    instructions = []
    in_code = False
    
    with open(asm_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line == ';<start>':
                in_code = True
            elif line == ';<end>':
                in_code = False
            elif in_code and not line.endswith(':') and line:
                instructions.append(line)
    
    return instructions

def process_asm_file(asm_file):
    """Process a single assembly file"""
    bin_file = Path(asm_file).with_suffix('.bin')
    
    print(f"Processing: {asm_file}")
    
    # Step 1: Assemble with nasm
    print("  1. Assembling with nasm...")
    shell_run(f"nasm -f bin {asm_file} -o {bin_file}")
    print(f"     Created: {bin_file}")
    
    # Step 2: Extract expected output
    print("  2. Extracting expected output...")
    expected = extract_asm_instructions(asm_file)
    
    # Step 3: Disassemble
    print("  3. Disassembling...")
    output = shell_run(f"cargo run --bin simx86 --quiet -- {bin_file} --instructions-only", 
                      capture_output=True)
    actual = [line.strip() for line in output.split('\n') if line.strip()]
    
    # Step 4: Compare
    print("  4. Verifying disassembly...")
    if expected == actual:
        print("     ✓ Disassembly matches original ASM")
    else:
        print("     ✗ Disassembly does not match original ASM")
        print("     Differences:")
        for line in difflib.unified_diff(expected, actual, lineterm=''):
            print(f"     {line}")
    
    # Step 5: Simulate
    print("  5. Simulating...")
    shell_run(f"cargo run --bin simx86 -- {bin_file} --execute")
    
    print()

def main():
    print("Building Rust project...")
    shell_run("cargo build --release")
    
    # Get files to process
    if len(sys.argv) > 1:
        # Process specific files
        asm_files = sys.argv[1:]
    else:
        # Process all .asm files in asm directory
        asm_files = list(Path('asm').glob('*.asm'))
    
    # Process each file
    for asm_file in asm_files:
        if Path(asm_file).exists():
            process_asm_file(asm_file)
        else:
            print(f"Warning: File not found: {asm_file}")


if __name__ == '__main__':
    main()