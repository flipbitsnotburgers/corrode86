use std::io::{self, Write};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

pub struct VideoDisplay {
    text_buffer: Arc<Mutex<[[char; 80]; 25]>>,
    running: Arc<Mutex<bool>>,
}

impl VideoDisplay {
    pub fn new() -> Self {
        VideoDisplay {
            text_buffer: Arc::new(Mutex::new([[' '; 80]; 25])),
            running: Arc::new(Mutex::new(false)),
        }
    }

    pub fn start(&mut self) {
        let text_buffer = Arc::clone(&self.text_buffer);
        let running = Arc::clone(&self.running);

        *running.lock().unwrap() = true;

        thread::spawn(move || {
            // Clear screen and hide cursor
            print!("\x1B[2J\x1B[?25l");
            io::stdout().flush().unwrap();

            let mut last_buffer = [[' '; 80]; 25];
            let frame_duration = Duration::from_millis(50); // 20 FPS
            
            while *running.lock().unwrap() {
                let frame_start = Instant::now();
                
                // Get the current buffer
                let buffer = text_buffer.lock().unwrap().clone();
                
                // Only update if the buffer has changed
                if buffer != last_buffer {
                    // Move cursor to top-left
                    print!("\x1B[H");

                    // Draw the text buffer
                    for y in 0..25 {
                        for x in 0..80 {
                            let ch = buffer[y][x];
                            // Use ANSI colors for different characters
                            // https://gist.github.com/JBlond/2fea43a3049b38287e5e9cefc87b2124
                            match ch {
                                '*' => print!("\x1B[33m{}\x1B[0m", ch),       // Yellow
                                '^' => print!("\x1B[32m{}\x1B[0m", ch),       // Green
                                'A'..='Z' => print!("\x1B[37m{}\x1B[0m", ch), // White
                                _ => print!("{}", ch),
                            }
                        }
                        println!();
                    }

                    io::stdout().flush().unwrap();
                    last_buffer = buffer;
                }
                
                // Sleep for the remainder of the frame time
                let elapsed = frame_start.elapsed();
                if elapsed < frame_duration {
                    thread::sleep(frame_duration - elapsed);
                }
            }

            // Show cursor again
            print!("\x1B[?25h");
            io::stdout().flush().unwrap();
        });
    }

    pub fn update_from_text_buffer(&mut self, new_buffer: &[[char; 80]; 25]) {
        *self.text_buffer.lock().unwrap() = *new_buffer;
    }

    pub fn stop(&mut self) {
        *self.running.lock().unwrap() = false;
    }

    pub fn is_running(&self) -> bool {
        *self.running.lock().unwrap()
    }
}

impl Drop for VideoDisplay {
    fn drop(&mut self) {
        self.stop();
        thread::sleep(Duration::from_millis(100));
    }
}
