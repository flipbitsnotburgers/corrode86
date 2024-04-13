use std::{i16, u16};

#[allow(dead_code)]
#[derive(PartialEq, Eq, Debug, Copy, Clone)]
#[repr(u16)]
pub enum Register {
    // 16-bit registers
    AX,
    BX,
    CX,
    DX,
    SI,
    DI,
    BP,
    SP,
    IP,
    CS,
    DS,
    ES,
    SS,
    // 8-bit high and low parts of AX, BX, CX, DX
    AH,
    AL,
    BH,
    BL,
    CH,
    CL,
    DH,
    DL,
}

impl Register {
    pub fn as_str(&self) -> &'static str {
        match self {
            Register::AX => "AX",
            Register::BX => "BX",
            Register::CX => "CX",
            Register::DX => "DX",
            Register::SI => "SI",
            Register::DI => "DI",
            Register::BP => "BP",
            Register::SP => "SP",
            Register::IP => "IP",
            Register::CS => "CS",
            Register::DS => "DS",
            Register::ES => "ES",
            Register::SS => "SS",
            Register::AH => "AH",
            Register::AL => "AL",
            Register::BH => "BH",
            Register::BL => "BL",
            Register::CH => "CH",
            Register::CL => "CL",
            Register::DH => "DH",
            Register::DL => "DL",
        }
    }

    fn from_ireg(width: Width, ireg: u8) -> Register {
        if width == Width::Byte {
            match ireg {
                0x00 => Register::AL,
                0x01 => Register::CL,
                0x02 => Register::DL,
                0x03 => Register::BL,
                0x04 => Register::AH,
                0x05 => Register::CH,
                0x06 => Register::DH,
                0x07 => Register::BH,
                _ => panic!("Invalid ireg value"),
            }
        } else {
            match ireg {
                0x00 => Register::AX,
                0x01 => Register::CX,
                0x02 => Register::DX,
                0x03 => Register::BX,
                0x04 => Register::SP,
                0x05 => Register::BP,
                0x06 => Register::SI,
                0x07 => Register::DI,
                _ => panic!("Invalid ireg value"),
            }
        }
    }
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
enum Opcode {
    MovRegMemToFromReg,
    MovImmToRegMem,
    MovImmToReg,
    MovMemToAcc,
    MovAccToMem,
    MovRegMemToSegReg,
    MovSegRegToRegMem,
    AddRegMemWithRegToEither,
    AddImmToRegMem,
    AddImmToAcc,
    SubRegMemAndRegToEither,
    SubImmFromRegMem,
    SubImmFromAcc,
    JmpShort,
    CmpRegMemAndReg,
    CmpImmWithRegMem,
    CmpImmWithAcc,
    Jle,
    Unkown,
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
enum Direction {
    To,
    From,
}

#[derive(PartialEq, Eq, Copy, Clone, Debug)]
enum Width {
    Byte,
    Word,
}

impl From<u8> for Width {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Byte,
            1 => Self::Word,
            _ => panic!("Invalid Width"),
        }
    }
}

impl From<u8> for Opcode {
    fn from(value: u8) -> Self {
        match value {
            0x88..=0x8B => Opcode::MovRegMemToFromReg,
            0xB0..=0xBF => Opcode::MovImmToReg,
            0xC6 | 0xC7 => Opcode::MovImmToRegMem,
            0xA0..=0xA1 => Opcode::MovMemToAcc,
            0xA2..=0xA3 => Opcode::MovAccToMem,
            0x8C => Opcode::MovSegRegToRegMem,
            0x8E => Opcode::MovRegMemToSegReg,
            0x00..=0x03 => Opcode::AddRegMemWithRegToEither,
            0x04..=0x05 => Opcode::AddImmToAcc,
            0x80..=0x83 if (value & 0x38) == 0x00 => Opcode::AddImmToRegMem,
            0x28..=0x2B => Opcode::SubRegMemAndRegToEither,
            0x2C..=0x2D => Opcode::SubImmFromAcc,
            0x80..=0x83 if (value & 0x38) == 0x28 => Opcode::SubImmFromRegMem,
            0x38..=0x3B => Opcode::CmpRegMemAndReg,
            0x3C..=0x3D => Opcode::CmpImmWithAcc,
            0x80..=0x83 if (value & 0x38) == 0x38 => Opcode::CmpImmWithRegMem,
            0xEB => Opcode::JmpShort,
            0x7E => Opcode::Jle,
            _ => {
                println!("Unknown {:0x}", value);
                return Opcode::Unkown;
            }
        }
    }
}

#[derive(Debug)]
struct Instruction {
    opcode: Opcode,
    direction: Direction,
    width: Width,
}

#[derive(Debug, Copy, Clone)]
enum ArithmeticOp {
    Add,
    Sub,
}

#[allow(dead_code)]
pub struct CPU {
    registers: [u16; 9],
    flags: u16,
    memory: [u8; 65536],  // 64KB for 8086
    stack: [u16; 16],
    instruction_log: Vec<String>,
    execute_mode: bool,  // true for execution, false for disassembly only
    program_size: u16,   // Size of loaded program
}

// Flags register bits
const FLAG_CARRY: u16 = 0x0001;
const FLAG_PARITY: u16 = 0x0004;
const FLAG_AUX_CARRY: u16 = 0x0010;
const FLAG_ZERO: u16 = 0x0040;
const FLAG_SIGN: u16 = 0x0080;
const FLAG_OVERFLOW: u16 = 0x0800;

impl CPU {
    pub fn new() -> CPU {
        CPU {
            registers: [0; 9],
            memory: [0; 65536],
            stack: [0; 16],
            flags: 0,
            instruction_log: Vec::new(),
            execute_mode: false,
            program_size: 0,
        }
    }

    pub fn set_execute_mode(&mut self, mode: bool) {
        self.execute_mode = mode;
    }

    pub fn set_program_size(&mut self, size: u16) {
        self.program_size = size;
    }

    pub fn read_register(&self, register: Register) -> u16 {
        match register {
            Register::AH | Register::AL => {
                self.get_register_half(Register::AX, register == Register::AH)
            }
            Register::BH | Register::BL => {
                self.get_register_half(Register::BX, register == Register::BH)
            }
            Register::CH | Register::CL => {
                self.get_register_half(Register::CX, register == Register::CH)
            }
            Register::DH | Register::DL => {
                self.get_register_half(Register::DX, register == Register::DH)
            }
            _ => self.registers[register as usize],
        }
    }

    pub fn write_register(&mut self, register: Register, value: u16) {
        match register {
            Register::AH | Register::AL => {
                self.set_register_half(Register::AX, register == Register::AH, value)
            }
            Register::BH | Register::BL => {
                self.set_register_half(Register::BX, register == Register::BH, value)
            }
            Register::CH | Register::CL => {
                self.set_register_half(Register::CX, register == Register::CH, value)
            }
            Register::DH | Register::DL => {
                self.set_register_half(Register::DX, register == Register::DH, value)
            }
            _ => self.registers[register as usize] = value,
        }
    }

    pub fn fetch_instruction(&mut self) -> u8 {
        let address = self.registers[Register::IP as usize];
        if address >= self.program_size {
            panic!("End of program");
        }
        let instr = self.memory[address as usize];

        self.registers[Register::IP as usize] += 1;

        instr
    }

    fn decode_instruction(&mut self, instruction: u8) -> Instruction {
        let opcode = match instruction {
            0x80..=0x83 => {
                // For these opcodes, we need to peek at the ModRM byte
                let modrm = self.memory[self.registers[Register::IP as usize] as usize];
                let op_bits = (modrm >> 3) & 0x07;
                match op_bits {
                    0x00 => Opcode::AddImmToRegMem,
                    0x05 => Opcode::SubImmFromRegMem,
                    0x07 => Opcode::CmpImmWithRegMem,
                    _ => Opcode::Unkown,
                }
            }
            _ => Opcode::from(instruction),
        };
        
        Instruction {
            opcode,
            direction: if (instruction & 0b0000_0010) == 0 {
                Direction::To
            } else {
                Direction::From
            },
            width: Width::from(instruction & 0b0000_0001),
        }
    }

    fn ea_calc(&mut self, irm: u8) -> String {
        match irm {
            0b000 => format!("BX + SI"),
            0b001 => format!("BX + DI"),
            0b010 => format!("BP + SI"),
            0b011 => format!("BP + DI"),
            0b100 => format!("SI"),
            0b101 => format!("DI"),
            0b110 => format!("BP"),
            0b111 => format!("BX"),
            _ => panic!("invalid irm"),
        }
    }

    fn ea_calc_address(&self, irm: u8, displacement: i16) -> u16 {
        let base = match irm {
            0b000 => self.read_register(Register::BX).wrapping_add(self.read_register(Register::SI)),
            0b001 => self.read_register(Register::BX).wrapping_add(self.read_register(Register::DI)),
            0b010 => self.read_register(Register::BP).wrapping_add(self.read_register(Register::SI)),
            0b011 => self.read_register(Register::BP).wrapping_add(self.read_register(Register::DI)),
            0b100 => self.read_register(Register::SI),
            0b101 => self.read_register(Register::DI),
            0b110 => self.read_register(Register::BP),
            0b111 => self.read_register(Register::BX),
            _ => panic!("invalid irm"),
        };
        base.wrapping_add(displacement as u16)
    }

    fn read_memory_word(&self, address: u16) -> u16 {
        let low = self.memory[address as usize] as u16;
        let high = self.memory[address.wrapping_add(1) as usize] as u16;
        (high << 8) | low
    }

    fn write_memory_word(&mut self, address: u16, value: u16) {
        self.memory[address as usize] = (value & 0xFF) as u8;
        self.memory[address.wrapping_add(1) as usize] = ((value >> 8) & 0xFF) as u8;
    }

    fn set_flags_for_result(&mut self, result: u32, width: Width) {
        // Zero flag
        if width == Width::Byte {
            self.set_flag(FLAG_ZERO, (result & 0xFF) == 0);
        } else {
            self.set_flag(FLAG_ZERO, (result & 0xFFFF) == 0);
        }

        // Sign flag
        if width == Width::Byte {
            self.set_flag(FLAG_SIGN, (result & 0x80) != 0);
        } else {
            self.set_flag(FLAG_SIGN, (result & 0x8000) != 0);
        }

        // Parity flag (count of 1 bits in low byte)
        let mut parity = 0;
        let byte = (result & 0xFF) as u8;
        for i in 0..8 {
            if (byte >> i) & 1 == 1 {
                parity += 1;
            }
        }
        self.set_flag(FLAG_PARITY, parity % 2 == 0);
    }

    fn set_flag(&mut self, flag: u16, value: bool) {
        if value {
            self.flags |= flag;
        } else {
            self.flags &= !flag;
        }
    }

    fn execute_instruction(&mut self, instruction: u8) {
        let instr = self.decode_instruction(instruction);

        match instr.opcode {
            Opcode::MovRegMemToFromReg => {
                let byte = self.fetch_instruction();

                let imod = (byte & 0b1100_0000) >> 6;
                let ireg = (byte & 0b0011_1000) >> 3;
                let irm = byte & 0b0000_0111;

                match imod {
                    0b11 => {
                        let (source, destination) = match instr.direction {
                            Direction::To => (
                                Register::from_ireg(instr.width, ireg),
                                Register::from_ireg(instr.width, irm),
                            ),
                            Direction::From => (
                                Register::from_ireg(instr.width, irm),
                                Register::from_ireg(instr.width, ireg),
                            ),
                        };

                        let log_entry =
                            format!("MOV {}, {}", destination.as_str(), source.as_str());
                        self.instruction_log.push(log_entry);
                        
                        if self.execute_mode {
                            let value = self.read_register(source);
                            self.write_register(destination, value);
                        }
                    }
                    0b00 => {
                        let (addr_str, addr_val) = if irm != 0b110 {
                            let addr = self.ea_calc_address(irm, 0);
                            (format!("[{}]", self.ea_calc(irm)), addr)
                        } else {
                            let data = if instr.width == Width::Byte {
                                self.read_data_byte() as i16
                            } else {
                                self.read_data_long()
                            };
                            (format!("[{}]", data), data as u16)
                        };
                        
                        let reg = Register::from_ireg(instr.width, ireg);
                        let (source_str, dest_str) = match instr.direction {
                            Direction::To => (reg.as_str(), addr_str.as_str()),
                            Direction::From => (addr_str.as_str(), reg.as_str()),
                        };

                        let log_entry = format!("MOV {}, {}", dest_str, source_str);
                        self.instruction_log.push(log_entry);
                        
                        if self.execute_mode {
                            match instr.direction {
                                Direction::To => {
                                    let value = self.read_register(reg);
                                    if instr.width == Width::Byte {
                                        self.write_memory_byte(addr_val, value as u8);
                                    } else {
                                        self.write_memory_word(addr_val, value);
                                    }
                                }
                                Direction::From => {
                                    let value = if instr.width == Width::Byte {
                                        self.read_memory_byte(addr_val) as u16
                                    } else {
                                        self.read_memory_word(addr_val)
                                    };
                                    self.write_register(reg, value);
                                }
                            }
                        }
                    }

                    0b01 => {
                        let addr_base = self.ea_calc(irm);
                        let disp = self.read_data_byte();
                        let addr_val = self.ea_calc_address(irm, disp as i16);

                        let addr_str = if disp == 0 {
                            format!("[{}]", addr_base)
                        } else if disp < 0 {
                            format!("[{} {}]", addr_base, disp)
                        } else {
                            format!("[{} + {}]", addr_base, disp)
                        };
                        
                        let reg = Register::from_ireg(instr.width, ireg);
                        let (source_str, dest_str) = match instr.direction {
                            Direction::To => (reg.as_str(), addr_str.as_str()),
                            Direction::From => (addr_str.as_str(), reg.as_str()),
                        };

                        let log_entry = format!("MOV {}, {}", dest_str, source_str);
                        self.instruction_log.push(log_entry);
                        
                        if self.execute_mode {
                            match instr.direction {
                                Direction::To => {
                                    let value = self.read_register(reg);
                                    if instr.width == Width::Byte {
                                        self.write_memory_byte(addr_val, value as u8);
                                    } else {
                                        self.write_memory_word(addr_val, value);
                                    }
                                }
                                Direction::From => {
                                    let value = if instr.width == Width::Byte {
                                        self.read_memory_byte(addr_val) as u16
                                    } else {
                                        self.read_memory_word(addr_val)
                                    };
                                    self.write_register(reg, value);
                                }
                            }
                        }
                    }
                    0b10 => {
                        let addr_base = self.ea_calc(irm);
                        let disp = self.read_data_long();
                        let addr_val = self.ea_calc_address(irm, disp);
                        
                        let addr_str = if disp == 0 {
                            format!("[{}]", addr_base)
                        } else if disp < 0 {
                            format!("[{} {}]", addr_base, disp)
                        } else {
                            format!("[{} + {}]", addr_base, disp)
                        };

                        let reg = Register::from_ireg(instr.width, ireg);
                        let (source_str, dest_str) = match instr.direction {
                            Direction::To => (reg.as_str(), addr_str.as_str()),
                            Direction::From => (addr_str.as_str(), reg.as_str()),
                        };

                        let log_entry = format!("MOV {}, {}", dest_str, source_str);
                        self.instruction_log.push(log_entry);
                        
                        if self.execute_mode {
                            match instr.direction {
                                Direction::To => {
                                    let value = self.read_register(reg);
                                    if instr.width == Width::Byte {
                                        self.write_memory_byte(addr_val, value as u8);
                                    } else {
                                        self.write_memory_word(addr_val, value);
                                    }
                                }
                                Direction::From => {
                                    let value = if instr.width == Width::Byte {
                                        self.read_memory_byte(addr_val) as u16
                                    } else {
                                        self.read_memory_word(addr_val)
                                    };
                                    self.write_register(reg, value);
                                }
                            }
                        }
                    }
                    _ => panic!("bad imod"),
                }
            }

            Opcode::MovImmToReg => {
                let width = Width::from((instruction & 0b00001000) >> 3);
                let ireg = instruction & 0b00000111;
                let destination = Register::from_ireg(width, ireg);

                let source = match width {
                    Width::Byte => self.read_data_byte() as i16,
                    Width::Word => self.read_data_long(),
                };

                let log_entry = format!("MOV {}, {}", destination.as_str(), source);
                self.instruction_log.push(log_entry);
                
                if self.execute_mode {
                    self.write_register(destination, source as u16);
                }
            }

            Opcode::MovImmToRegMem => {
                let byte = self.fetch_instruction();
                let imod = (byte & 0b1100_0000) >> 6;
                let irm = byte & 0b0000_0111;

                match (instr.width, imod) {
                    (Width::Byte, 0b00) => {
                        let addr_str = self.ea_calc(irm);
                        let addr_val = self.ea_calc_address(irm, 0);
                        let data = self.read_data_byte();

                        let log_entry = format!("MOV [{}], byte {}", addr_str, data);
                        self.instruction_log.push(log_entry);
                        
                        if self.execute_mode {
                            self.write_memory_byte(addr_val, data as u8);
                        }
                    }
                    (Width::Word, _) => {
                        let addr_str = self.ea_calc(irm);
                        let disp = if imod != 0b00 { self.read_data_long() } else { 0 };
                        let addr_val = self.ea_calc_address(irm, disp);
                        let data = self.read_data_long();

                        let log_entry = if disp != 0 {
                            format!("MOV [{} + {}], word {}", addr_str, disp, data)
                        } else {
                            format!("MOV [{}], word {}", addr_str, data)
                        };
                        self.instruction_log.push(log_entry);
                        
                        if self.execute_mode {
                            self.write_memory_word(addr_val, data as u16);
                        }
                    }
                    _ => panic!("Unsupported MOV immediate to memory combination")
                }
            }

            Opcode::MovMemToAcc => {
                let (accumulator, addr) = if instr.width == Width::Byte {
                    (Register::AL, self.read_data_byte() as u16)
                } else {
                    (Register::AX, self.read_data_long() as u16)
                };

                let log_entry = format!("MOV {}, [{}]", accumulator.as_str(), addr);
                self.instruction_log.push(log_entry);
                
                if self.execute_mode {
                    let value = if instr.width == Width::Byte {
                        self.read_memory_byte(addr) as u16
                    } else {
                        self.read_memory_word(addr)
                    };
                    self.write_register(accumulator, value);
                }
            }

            Opcode::MovAccToMem => {
                let (accumulator, addr) = if instr.width == Width::Byte {
                    (Register::AL, self.read_data_byte() as u16)
                } else {
                    (Register::AX, self.read_data_long() as u16)
                };

                let log_entry = format!("MOV [{}], {}", addr, accumulator.as_str());
                self.instruction_log.push(log_entry);
                
                if self.execute_mode {
                    let value = self.read_register(accumulator);
                    if instr.width == Width::Byte {
                        self.write_memory_byte(addr, value as u8);
                    } else {
                        self.write_memory_word(addr, value);
                    }
                }
            }

            Opcode::JmpShort => {
                let offset = self.read_data_byte();
                let target = (self.read_register(Register::IP) as i16 + offset as i16) as u16;
                
                let log_entry = format!("JMP short {}", offset);
                self.instruction_log.push(log_entry);
                
                if self.execute_mode {
                    self.write_register(Register::IP, target);
                }
            }
            
            Opcode::AddRegMemWithRegToEither => {
                self.execute_arithmetic_reg_mem(instruction, instr, ArithmeticOp::Add);
            }
            
            Opcode::AddImmToAcc => {
                self.execute_arithmetic_imm_acc(instr, ArithmeticOp::Add);
            }
            
            Opcode::AddImmToRegMem => {
                self.execute_arithmetic_imm_reg_mem(instruction, instr, ArithmeticOp::Add);
            }
            
            Opcode::SubRegMemAndRegToEither => {
                self.execute_arithmetic_reg_mem(instruction, instr, ArithmeticOp::Sub);
            }
            
            Opcode::SubImmFromAcc => {
                self.execute_arithmetic_imm_acc(instr, ArithmeticOp::Sub);
            }
            
            Opcode::SubImmFromRegMem => {
                self.execute_arithmetic_imm_reg_mem(instruction, instr, ArithmeticOp::Sub);
            }
            
            Opcode::CmpRegMemAndReg => {
                self.execute_cmp_reg_mem(instruction, instr);
            }
            
            Opcode::CmpImmWithAcc => {
                self.execute_cmp_imm_acc(instr);
            }
            
            Opcode::CmpImmWithRegMem => {
                self.execute_cmp_imm_reg_mem(instruction, instr);
            }
            
            Opcode::Jle => {
                let offset = self.read_data_byte();
                let log_entry = format!("JLE {}", offset);
                self.instruction_log.push(log_entry);
                
                if self.execute_mode {
                    // Jump if ZF=1 OR SF≠OF
                    let zero_flag = (self.flags & FLAG_ZERO) != 0;
                    let sign_flag = (self.flags & FLAG_SIGN) != 0;
                    let overflow_flag = (self.flags & FLAG_OVERFLOW) != 0;
                    
                    if zero_flag || (sign_flag != overflow_flag) {
                        let target = (self.read_register(Register::IP) as i16 + offset as i16) as u16;
                        self.write_register(Register::IP, target);
                    }
                }
            }
            
            _ => {
                let log_entry = format!("{:#?}", instr.opcode);
                self.instruction_log.push(log_entry);
                panic!("Unknown instruction");
            }
        }
    }

    // Combine fetch, decode, and execute for simplicity
    pub fn execute_next_instruction(&mut self) {
        let instr = self.fetch_instruction();
        self.execute_instruction(instr);
    }

    // Function to read a byte from memory
    pub fn read_memory_byte(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn read_data_byte(&mut self) -> i8 {
        self.fetch_instruction() as i8
    }

    fn read_data_long(&mut self) -> i16 {
        let data0 = self.fetch_instruction();
        let data1 = self.fetch_instruction();

        let mut data = data1 as i16;
        data <<= 8;
        data |= 0x00ff & (data0 as i16);
        data
    }

    // Function to write a byte to memory
    pub fn write_memory_byte(&mut self, address: u16, value: u8) {
        self.memory[address as usize] = value;
    }

    fn get_register_half(&self, reg: Register, high: bool) -> u16 {
        let value = self.registers[reg as usize];
        if high {
            (value >> 8) as u16
        } else {
            (value & 0x00FF) as u16
        }
    }

    fn set_register_half(&mut self, reg: Register, high: bool, value: u16) {
        let register_index = reg as usize;
        if high {
            self.registers[register_index] =
                (self.registers[register_index] & 0x00FF) | (value << 8);
        } else {
            self.registers[register_index] =
                (self.registers[register_index] & 0xFF00) | (value & 0x00FF);
        }
    }

    pub fn dump_assembly(&self) {
        for (i, instr) in self.instruction_log.iter().enumerate() {
            println!("{:04X}: {}", i, instr);
        }
    }
    
    pub fn dump_instructions_only(&self) {
        for instr in self.instruction_log.iter() {
            println!("{}", instr);
        }
    }

    pub fn dump_registers(&self) {
        println!("AX: {:04X} (AH: {:02X}, AL: {:02X})", 
            self.read_register(Register::AX),
            self.read_register(Register::AH),
            self.read_register(Register::AL));
        println!("BX: {:04X} (BH: {:02X}, BL: {:02X})", 
            self.read_register(Register::BX),
            self.read_register(Register::BH),
            self.read_register(Register::BL));
        println!("CX: {:04X} (CH: {:02X}, CL: {:02X})", 
            self.read_register(Register::CX),
            self.read_register(Register::CH),
            self.read_register(Register::CL));
        println!("DX: {:04X} (DH: {:02X}, DL: {:02X})", 
            self.read_register(Register::DX),
            self.read_register(Register::DH),
            self.read_register(Register::DL));
        println!("SP: {:04X}, BP: {:04X}, SI: {:04X}, DI: {:04X}", 
            self.read_register(Register::SP),
            self.read_register(Register::BP),
            self.read_register(Register::SI),
            self.read_register(Register::DI));
        println!("IP: {:04X}", self.read_register(Register::IP));
        println!("Flags: {:04X} (C:{} P:{} A:{} Z:{} S:{} O:{})",
            self.flags,
            if self.flags & FLAG_CARRY != 0 { "1" } else { "0" },
            if self.flags & FLAG_PARITY != 0 { "1" } else { "0" },
            if self.flags & FLAG_AUX_CARRY != 0 { "1" } else { "0" },
            if self.flags & FLAG_ZERO != 0 { "1" } else { "0" },
            if self.flags & FLAG_SIGN != 0 { "1" } else { "0" },
            if self.flags & FLAG_OVERFLOW != 0 { "1" } else { "0" });
    }

    fn execute_arithmetic_reg_mem(&mut self, _instruction: u8, instr: Instruction, op: ArithmeticOp) {
        let byte = self.fetch_instruction();
        let imod = (byte & 0b1100_0000) >> 6;
        let ireg = (byte & 0b0011_1000) >> 3;
        let irm = byte & 0b0000_0111;

        let op_str = match op {
            ArithmeticOp::Add => "ADD",
            ArithmeticOp::Sub => "SUB",
        };

        match imod {
            0b11 => {
                // Register to register
                let (source, destination) = match instr.direction {
                    Direction::To => (
                        Register::from_ireg(instr.width, ireg),
                        Register::from_ireg(instr.width, irm),
                    ),
                    Direction::From => (
                        Register::from_ireg(instr.width, irm),
                        Register::from_ireg(instr.width, ireg),
                    ),
                };

                let log_entry = format!("{} {}, {}", op_str, destination.as_str(), source.as_str());
                self.instruction_log.push(log_entry);

                if self.execute_mode {
                    let src_val = self.read_register(source);
                    let dst_val = self.read_register(destination);
                    let (result, carry, overflow) = match op {
                        ArithmeticOp::Add => {
                            let res = dst_val.wrapping_add(src_val);
                            let carry = dst_val as u32 + src_val as u32 > 0xFFFF;
                            let overflow = ((dst_val ^ res) & (src_val ^ res) & 0x8000) != 0;
                            (res, carry, overflow)
                        }
                        ArithmeticOp::Sub => {
                            let res = dst_val.wrapping_sub(src_val);
                            let carry = dst_val < src_val;
                            let overflow = ((dst_val ^ src_val) & (dst_val ^ res) & 0x8000) != 0;
                            (res, carry, overflow)
                        }
                    };
                    self.write_register(destination, result);
                    self.set_flag(FLAG_CARRY, carry);
                    self.set_flag(FLAG_OVERFLOW, overflow);
                    self.set_flags_for_result(result as u32, instr.width);
                }
            }
            _ => {
                // Memory operations - similar pattern but with memory access
                let disp = match imod {
                    0b00 => if irm == 0b110 { self.read_data_long() } else { 0 },
                    0b01 => self.read_data_byte() as i16,
                    0b10 => self.read_data_long(),
                    _ => 0,
                };

                let addr_val = if irm == 0b110 && imod == 0b00 {
                    disp as u16
                } else {
                    self.ea_calc_address(irm, disp)
                };

                let addr_str = if irm == 0b110 && imod == 0b00 {
                    format!("[{}]", disp)
                } else {
                    let base = self.ea_calc(irm);
                    if disp == 0 {
                        format!("[{}]", base)
                    } else if disp < 0 {
                        format!("[{} {}]", base, disp)
                    } else {
                        format!("[{} + {}]", base, disp)
                    }
                };

                let reg = Register::from_ireg(instr.width, ireg);
                let (source_str, dest_str) = match instr.direction {
                    Direction::To => (reg.as_str(), addr_str.as_str()),
                    Direction::From => (addr_str.as_str(), reg.as_str()),
                };

                let log_entry = format!("{} {}, {}", op_str, dest_str, source_str);
                self.instruction_log.push(log_entry);

                if self.execute_mode {
                    match instr.direction {
                        Direction::To => {
                            // Reg to memory
                            let reg_val = self.read_register(reg);
                            let mem_val = if instr.width == Width::Byte {
                                self.read_memory_byte(addr_val) as u16
                            } else {
                                self.read_memory_word(addr_val)
                            };
                            
                            let (result, carry, overflow) = match op {
                                ArithmeticOp::Add => {
                                    let res = mem_val.wrapping_add(reg_val);
                                    let carry = mem_val as u32 + reg_val as u32 > 0xFFFF;
                                    let overflow = ((mem_val ^ res) & (reg_val ^ res) & 0x8000) != 0;
                                    (res, carry, overflow)
                                }
                                ArithmeticOp::Sub => {
                                    let res = mem_val.wrapping_sub(reg_val);
                                    let carry = mem_val < reg_val;
                                    let overflow = ((mem_val ^ reg_val) & (mem_val ^ res) & 0x8000) != 0;
                                    (res, carry, overflow)
                                }
                            };
                            
                            if instr.width == Width::Byte {
                                self.write_memory_byte(addr_val, result as u8);
                            } else {
                                self.write_memory_word(addr_val, result);
                            }
                            self.set_flag(FLAG_CARRY, carry);
                            self.set_flag(FLAG_OVERFLOW, overflow);
                            self.set_flags_for_result(result as u32, instr.width);
                        }
                        Direction::From => {
                            // Memory to reg
                            let reg_val = self.read_register(reg);
                            let mem_val = if instr.width == Width::Byte {
                                self.read_memory_byte(addr_val) as u16
                            } else {
                                self.read_memory_word(addr_val)
                            };
                            
                            let (result, carry, overflow) = match op {
                                ArithmeticOp::Add => {
                                    let res = reg_val.wrapping_add(mem_val);
                                    let carry = reg_val as u32 + mem_val as u32 > 0xFFFF;
                                    let overflow = ((reg_val ^ res) & (mem_val ^ res) & 0x8000) != 0;
                                    (res, carry, overflow)
                                }
                                ArithmeticOp::Sub => {
                                    let res = reg_val.wrapping_sub(mem_val);
                                    let carry = reg_val < mem_val;
                                    let overflow = ((reg_val ^ mem_val) & (reg_val ^ res) & 0x8000) != 0;
                                    (res, carry, overflow)
                                }
                            };
                            
                            self.write_register(reg, result);
                            self.set_flag(FLAG_CARRY, carry);
                            self.set_flag(FLAG_OVERFLOW, overflow);
                            self.set_flags_for_result(result as u32, instr.width);
                        }
                    }
                }
            }
        }
    }

    fn execute_arithmetic_imm_acc(&mut self, instr: Instruction, op: ArithmeticOp) {
        let op_str = match op {
            ArithmeticOp::Add => "ADD",
            ArithmeticOp::Sub => "SUB",
        };

        let (acc_reg, imm_val) = if instr.width == Width::Byte {
            (Register::AL, self.read_data_byte() as u16)
        } else {
            (Register::AX, self.read_data_long() as u16)
        };

        let log_entry = format!("{} {}, {}", op_str, acc_reg.as_str(), imm_val as i16);
        self.instruction_log.push(log_entry);

        if self.execute_mode {
            let acc_val = self.read_register(acc_reg);
            let (result, carry, overflow) = match op {
                ArithmeticOp::Add => {
                    let res = acc_val.wrapping_add(imm_val);
                    let carry = acc_val as u32 + imm_val as u32 > 0xFFFF;
                    let overflow = ((acc_val ^ res) & (imm_val ^ res) & 0x8000) != 0;
                    (res, carry, overflow)
                }
                ArithmeticOp::Sub => {
                    let res = acc_val.wrapping_sub(imm_val);
                    let carry = acc_val < imm_val;
                    let overflow = ((acc_val ^ imm_val) & (acc_val ^ res) & 0x8000) != 0;
                    (res, carry, overflow)
                }
            };
            
            self.write_register(acc_reg, result);
            self.set_flag(FLAG_CARRY, carry);
            self.set_flag(FLAG_OVERFLOW, overflow);
            self.set_flags_for_result(result as u32, instr.width);
        }
    }

    fn execute_arithmetic_imm_reg_mem(&mut self, instruction: u8, instr: Instruction, op: ArithmeticOp) {
        let byte = self.fetch_instruction();
        let imod = (byte & 0b1100_0000) >> 6;
        let irm = byte & 0b0000_0111;

        let op_str = match op {
            ArithmeticOp::Add => "ADD",
            ArithmeticOp::Sub => "SUB",
        };

        // Check if it's sign-extended
        let sign_extend = (instruction & 0x02) != 0;

        if imod == 0b11 {
            // Immediate to register
            let reg = Register::from_ireg(instr.width, irm);
            let imm_val = if sign_extend && instr.width == Width::Word {
                self.read_data_byte() as i8 as i16 as u16
            } else if instr.width == Width::Byte {
                self.read_data_byte() as u16
            } else {
                self.read_data_long() as u16
            };

            let log_entry = format!("{} {}, {}", op_str, reg.as_str(), imm_val as i16);
            self.instruction_log.push(log_entry);

            if self.execute_mode {
                let reg_val = self.read_register(reg);
                let (result, carry, overflow) = match op {
                    ArithmeticOp::Add => {
                        let res = reg_val.wrapping_add(imm_val);
                        let carry = reg_val as u32 + imm_val as u32 > 0xFFFF;
                        let overflow = ((reg_val ^ res) & (imm_val ^ res) & 0x8000) != 0;
                        (res, carry, overflow)
                    }
                    ArithmeticOp::Sub => {
                        let res = reg_val.wrapping_sub(imm_val);
                        let carry = reg_val < imm_val;
                        let overflow = ((reg_val ^ imm_val) & (reg_val ^ res) & 0x8000) != 0;
                        (res, carry, overflow)
                    }
                };
                
                self.write_register(reg, result);
                self.set_flag(FLAG_CARRY, carry);
                self.set_flag(FLAG_OVERFLOW, overflow);
                self.set_flags_for_result(result as u32, instr.width);
            }
        } else {
            // Immediate to memory
            let disp = match imod {
                0b00 => if irm == 0b110 { self.read_data_long() } else { 0 },
                0b01 => self.read_data_byte() as i16,
                0b10 => self.read_data_long(),
                _ => 0,
            };

            let addr_val = if irm == 0b110 && imod == 0b00 {
                disp as u16
            } else {
                self.ea_calc_address(irm, disp)
            };

            let imm_val = if sign_extend && instr.width == Width::Word {
                self.read_data_byte() as i8 as i16 as u16
            } else if instr.width == Width::Byte {
                self.read_data_byte() as u16
            } else {
                self.read_data_long() as u16
            };

            let addr_str = if irm == 0b110 && imod == 0b00 {
                format!("[{}]", disp)
            } else {
                let base = self.ea_calc(irm);
                if disp == 0 {
                    format!("[{}]", base)
                } else if disp < 0 {
                    format!("[{} {}]", base, disp)
                } else {
                    format!("[{} + {}]", base, disp)
                }
            };

            let size_prefix = if instr.width == Width::Byte { "byte " } else { "word " };
            let log_entry = format!("{} {}{}, {}", op_str, size_prefix, addr_str, imm_val as i16);
            self.instruction_log.push(log_entry);

            if self.execute_mode {
                let mem_val = if instr.width == Width::Byte {
                    self.read_memory_byte(addr_val) as u16
                } else {
                    self.read_memory_word(addr_val)
                };
                
                let (result, carry, overflow) = match op {
                    ArithmeticOp::Add => {
                        let res = mem_val.wrapping_add(imm_val);
                        let carry = mem_val as u32 + imm_val as u32 > 0xFFFF;
                        let overflow = ((mem_val ^ res) & (imm_val ^ res) & 0x8000) != 0;
                        (res, carry, overflow)
                    }
                    ArithmeticOp::Sub => {
                        let res = mem_val.wrapping_sub(imm_val);
                        let carry = mem_val < imm_val;
                        let overflow = ((mem_val ^ imm_val) & (mem_val ^ res) & 0x8000) != 0;
                        (res, carry, overflow)
                    }
                };
                
                if instr.width == Width::Byte {
                    self.write_memory_byte(addr_val, result as u8);
                } else {
                    self.write_memory_word(addr_val, result);
                }
                self.set_flag(FLAG_CARRY, carry);
                self.set_flag(FLAG_OVERFLOW, overflow);
                self.set_flags_for_result(result as u32, instr.width);
            }
        }
    }

    fn execute_cmp_reg_mem(&mut self, _instruction: u8, instr: Instruction) {
        let byte = self.fetch_instruction();
        let imod = (byte & 0b1100_0000) >> 6;
        let ireg = (byte & 0b0011_1000) >> 3;
        let irm = byte & 0b0000_0111;

        match imod {
            0b11 => {
                // Register with register
                let reg1 = Register::from_ireg(instr.width, irm);
                let reg2 = Register::from_ireg(instr.width, ireg);

                let log_entry = format!("CMP {}, {}", reg1.as_str(), reg2.as_str());
                self.instruction_log.push(log_entry);

                if self.execute_mode {
                    let val1 = self.read_register(reg1);
                    let val2 = self.read_register(reg2);
                    let result = val1.wrapping_sub(val2);
                    let carry = val1 < val2;
                    let overflow = ((val1 ^ val2) & (val1 ^ result) & 0x8000) != 0;
                    
                    self.set_flag(FLAG_CARRY, carry);
                    self.set_flag(FLAG_OVERFLOW, overflow);
                    self.set_flags_for_result(result as u32, instr.width);
                }
            }
            _ => {
                // Memory operations - similar to arithmetic but without storing result
                panic!("CMP memory operations not yet implemented");
            }
        }
    }

    fn execute_cmp_imm_acc(&mut self, instr: Instruction) {
        let (acc_reg, imm_val) = if instr.width == Width::Byte {
            (Register::AL, self.read_data_byte() as u16)
        } else {
            (Register::AX, self.read_data_long() as u16)
        };

        let log_entry = format!("CMP {}, {}", acc_reg.as_str(), imm_val as i16);
        self.instruction_log.push(log_entry);

        if self.execute_mode {
            let acc_val = self.read_register(acc_reg);
            let result = acc_val.wrapping_sub(imm_val);
            let carry = acc_val < imm_val;
            let overflow = ((acc_val ^ imm_val) & (acc_val ^ result) & 0x8000) != 0;
            
            self.set_flag(FLAG_CARRY, carry);
            self.set_flag(FLAG_OVERFLOW, overflow);
            self.set_flags_for_result(result as u32, instr.width);
        }
    }

    fn execute_cmp_imm_reg_mem(&mut self, instruction: u8, instr: Instruction) {
        let byte = self.fetch_instruction();
        let imod = (byte & 0b1100_0000) >> 6;
        let irm = byte & 0b0000_0111;

        // Check if it's sign-extended
        let sign_extend = (instruction & 0x02) != 0;

        if imod == 0b11 {
            // Immediate with register
            let reg = Register::from_ireg(instr.width, irm);
            let imm_val = if sign_extend && instr.width == Width::Word {
                self.read_data_byte() as i8 as i16 as u16
            } else if instr.width == Width::Byte {
                self.read_data_byte() as u16
            } else {
                self.read_data_long() as u16
            };

            let log_entry = format!("CMP {}, {}", reg.as_str(), imm_val as i16);
            self.instruction_log.push(log_entry);

            if self.execute_mode {
                let reg_val = self.read_register(reg);
                let result = reg_val.wrapping_sub(imm_val);
                let carry = reg_val < imm_val;
                let overflow = ((reg_val ^ imm_val) & (reg_val ^ result) & 0x8000) != 0;
                
                self.set_flag(FLAG_CARRY, carry);
                self.set_flag(FLAG_OVERFLOW, overflow);
                self.set_flags_for_result(result as u32, instr.width);
            }
        } else {
            panic!("CMP immediate with memory not yet implemented");
        }
    }
}
