use simx86::cpu::CPU;
use simx86::video::VideoDisplay;

use std::fs::File;
use std::io::{self, Read};
use std::thread;
use std::time::Duration;

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
        let mut execute_mode = false;
        let mut instructions_only = false;
        let mut video_window = false;
        
        // Parse flags
        for arg in &args[2..] {
            match arg.as_str() {
                "--execute" => execute_mode = true,
                "--instructions-only" => instructions_only = true,
                "--video" => video_window = true,
                _ => {}
            }
        }
        
        process_file(filename, execute_mode, instructions_only, video_window);
    } else {
        // Show usage
        println!("Usage: {} <filename> [OPTIONS]", args[0]);
        println!();
        println!("Options:");
        println!("  --execute           Execute the program");
        println!("  --instructions-only Show only instructions (no addresses)");
        println!("  --video            Show video display window (with --execute)");
        println!();
        println!("Examples:");
        println!("  Disassemble:         {} data/calc_add.bin", args[0]);
        println!("  Execute:             {} data/calc_add.bin --execute", args[0]);
        println!("  Execute with video:  {} data/calc_add.bin --execute --video", args[0]);
    }
}

fn process_file(file: &str, execute_mode: bool, instructions_only: bool, video_window: bool) {
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
    
    // Disable instruction logging in video mode to prevent memory exhaustion
    if video_window && execute_mode {
        cpu.set_log_instructions(false);
    }

    // Load the program into memory
    for (i, byte) in bytes.iter().enumerate() {
        cpu.write_memory_byte(i as u16, *byte);
    }

    // Start video display if requested
    let mut video_display = if video_window && execute_mode {
        let mut display = VideoDisplay::new();
        display.start();
        Some(display)
    } else {
        None
    };

    // Execute the program
    loop {
        let result = panic::catch_unwind(AssertUnwindSafe(|| {
            cpu.execute_next_instruction();
        }));
        
        // Update video display if active
        if let Some(ref mut display) = video_display {
            if cpu.is_video_dirty() {
                display.update_from_text_buffer(cpu.get_video_buffer());
                cpu.clear_video_dirty();
            }
        }

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
                        if !video_window {
                            cpu.dump_video();
                        }
                    }
                }
                break;
            }
        }
    }
    
    // Keep window open if video display is active
    if let Some(ref display) = video_display {
        println!("\nPress Ctrl+C to exit...");
        while display.is_running() {
            thread::sleep(Duration::from_millis(100));
        }
    }
}
