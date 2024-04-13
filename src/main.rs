use simx86::cpu::CPU;

use std::fs::File;
use std::io::{self, Read};

use std::panic::{self, AssertUnwindSafe};

fn read_file_to_bytes(filename: &str) -> io::Result<Vec<u8>> {
    let mut file = File::open(filename)?;
    let mut buffer = Vec::new();

    // Read the whole file
    file.read_to_end(&mut buffer)?;
    Ok(buffer)
}

fn main() {
    panic::set_hook(Box::new(|_| {}));

    // Check command line arguments
    let args: Vec<String> = std::env::args().collect();
    
    if args.len() > 1 {
        // Use file from command line
        let filename = &args[1];
        let execute_mode = args.get(2).map(|s| s == "--execute").unwrap_or(false);
        let instructions_only = args.get(2).map(|s| s == "--instructions-only").unwrap_or(false);
        
        process_file(filename, execute_mode, instructions_only);
    } else {
        // Show usage
        println!("Usage: {} <filename> [--execute|--instructions-only]", args[0]);
        println!();
        println!("Examples:");
        println!("  Disassemble:         {} data/calc_add.bin", args[0]);
        println!("  Execute:             {} data/calc_add.bin --execute", args[0]);
        println!("  Instructions only:   {} data/calc_add.bin --instructions-only", args[0]);
    }
}

fn process_file(file: &str, execute_mode: bool, instructions_only: bool) {
    if !instructions_only {
        if execute_mode {
            println!("Executing: {}", file);
        } else {
            println!("Disassembling: {}", file);
        }
    }

    let bytes = read_file_to_bytes(file).unwrap();

    let mut cpu = CPU::new();
    cpu.set_execute_mode(execute_mode);
    cpu.set_program_size(bytes.len() as u16);

    // Load the program into memory
    for (i, byte) in bytes.iter().enumerate() {
        cpu.write_memory_byte(i as u16, *byte);
    }

    // Execute the program
    loop {
        let result = panic::catch_unwind(AssertUnwindSafe(|| {
            cpu.execute_next_instruction();
        }));

        match result {
            Ok(_) => continue,
            Err(_) => {
                if instructions_only {
                    cpu.dump_instructions_only();
                } else {
                    cpu.dump_assembly();
                    if execute_mode {
                        println!("\nFinal register state:");
                        cpu.dump_registers();
                    }
                }
                break;
            }
        }
    }
}
