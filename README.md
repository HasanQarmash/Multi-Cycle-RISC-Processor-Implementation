# 🚀 Multi-Cycle RISC Processor Implementation

[![University](https://img.shields.io/badge/University-Birzeit%20University-green)](https://www.birzeit.edu)
[![Course](https://img.shields.io/badge/Course-ENC54370%20Computer%20Architecture-blue)](https://github.com)
[![Language](https://img.shields.io/badge/Language-Verilog-red)](https://verilog.com)
[![License](https://img.shields.io/badge/License-Academic-yellow)](https://github.com)

> **A complete implementation of a multi-cycle RISC processor with 5 execution stages supporting R-type, I-type, J-type, and S-type instructions with comprehensive verification and testing.**

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture Features](#architecture-features)
- [Instruction Set Architecture (ISA)](#instruction-set-architecture-isa)
- [Pipeline Stages](#execution-stages)
- [Implementation Details](#implementation-details)
- [Verification & Testing](#verification--testing)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Results & Performance](#results--performance)
- [Team & Contributions](#team--contributions)
- [Academic Information](#academic-information)

## 🎯 Overview

This project implements a **16-bit RISC processor** with a **multi-cycle architecture** featuring 5 distinct execution stages designed to execute a comprehensive instruction set including arithmetic, logic, memory, branch, and jump operations. The processor features separate instruction and data memories, an 8-register register file, and supports both signed and unsigned operations.

### Key Highlights

- ✅ **Multi-Cycle Design**: 5 execution stages with state-machine control
- ✅ **21 Instructions**: Complete ISA supporting R, I, J, and S type instructions
- ✅ **16-bit Architecture**: Word-aligned with byte-addressable memory
- ✅ **8 General-Purpose Registers**: R0-R7 with R0 hardwired to zero
- ✅ **Sequential Execution**: Each instruction executes through multiple clock cycles
- ✅ **Memory Hierarchy**: Separate instruction and data memories
- ✅ **Comprehensive Testing**: Multiple testbenches and verification environments

## 🏗️ Architecture Features

<img width="1394" height="685" alt="image" src="https://github.com/user-attachments/assets/7403ce70-0496-4945-946b-22c593d61e6b" />

### Processor Specifications

| Feature              | Specification                                  |
| -------------------- | ---------------------------------------------- |
| **Architecture**     | 16-bit RISC Multi-Cycle                        |
| **Execution Stages** | 5 (Fetch, Decode, Execute, Memory, Write-back) |
| **Instruction Size** | 16 bits                                        |
| **Word Size**        | 16 bits                                        |
| **Register File**    | 8 × 16-bit registers (R0-R7)                   |
| **Memory**           | Separate instruction and data memories         |
| **Addressing**       | Byte-addressable with word alignment           |
| **Endianness**       | Little-endian                                  |

### Core Components

1. **Control Unit** - Multi-cycle state machine managing execution flow
2. **ALU** - 16-bit arithmetic and logic unit with flag generation
3. **Register File** - 8 × 16-bit registers with dual read ports
4. **Instruction Memory** - 128 × 16-bit instruction storage
5. **Data Memory** - 256 × 16-bit data storage with byte access
6. **PC Module** - Program counter with jump/branch target calculation

## 📚 Instruction Set Architecture (ISA)

The processor supports **21 instructions** across 4 instruction types:

### R-Type Instructions (Register-to-Register)

```
Format: [Opcode(4)] [Rd(3)] [Rs1(3)] [Rs2(3)] [Unused(3)]
```

| Instruction | Opcode | Description                   |
| ----------- | ------ | ----------------------------- |
| `AND`       | 0000   | Reg(Rd) = Reg(Rs1) & Reg(Rs2) |
| `ADD`       | 0001   | Reg(Rd) = Reg(Rs1) + Reg(Rs2) |
| `SUB`       | 0010   | Reg(Rd) = Reg(Rs1) - Reg(Rs2) |

### I-Type Instructions (Immediate Operations)

```
Format: [Opcode(4)] [m(1)] [Rd(3)] [Rs1(3)] [Immediate(5)]
```

| Instruction | Opcode | Description                    |
| ----------- | ------ | ------------------------------ |
| `ADDI`      | 0011   | Reg(Rd) = Reg(Rs1) + Immediate |
| `ANDI`      | 0100   | Reg(Rd) = Reg(Rs1) & Immediate |
| `LW`        | 0101   | Reg(Rd) = Mem[Reg(Rs1) + Imm]  |
| `LBu`       | 0110   | Load byte unsigned             |
| `LBs`       | 0110   | Load byte signed               |
| `SW`        | 0111   | Mem[Reg(Rs1) + Imm] = Reg(Rd)  |

### Branch Instructions

| Instruction | Opcode | Description            |
| ----------- | ------ | ---------------------- |
| `BGT/BGTZ`  | 1000   | Branch if greater than |
| `BLT/BLTZ`  | 1001   | Branch if less than    |
| `BEQ/BEQZ`  | 1010   | Branch if equal        |
| `BNE/BNEZ`  | 1011   | Branch if not equal    |

### J-Type Instructions (Jump Operations)

```
Format: [Opcode(4)] [Jump Offset(12)]
```

| Instruction | Opcode | Description                          |
| ----------- | ------ | ------------------------------------ |
| `JMP`       | 1100   | PC = {PC[15:12], Immediate}          |
| `CALL`      | 1101   | Call subroutine, save return address |
| `RET`       | 1110   | Return from subroutine               |

### S-Type Instructions (Store)

```
Format: [Opcode(4)] [Rs(3)] [Immediate(9)]
```

| Instruction | Opcode | Description         |
| ----------- | ------ | ------------------- |
| `SV`        | 1111   | Mem[Rs] = Immediate |

## ⚙️ Execution Stages

The processor implements a multi-cycle architecture with 5 distinct execution stages:

### 1. 🔍 Instruction Fetch (IF)

- Fetches instruction from instruction memory using current PC
- Updates program counter based on control signals
- Handles branch target calculation and jump addresses
- **Duration**: 1 clock cycle

### 2. 🔧 Instruction Decode (ID)

- Decodes instruction fields (opcode, registers, immediate values)
- Reads source registers from register file
- Generates control signals for subsequent stages
- Determines instruction type and execution path
- **Duration**: 1 clock cycle

### 3. ⚡ Execute (EX)

- Performs arithmetic/logic operations in ALU
- Calculates effective addresses for memory operations
- Evaluates branch conditions and sets status flags
- Executes register-to-register operations
- **Duration**: 1 clock cycle

### 4. 💾 Memory Access (MEM)

- Accesses data memory for load/store instructions
- Supports word and byte operations (signed/unsigned)
- Writes data to memory for store operations
- Bypassed for non-memory instructions
- **Duration**: 1 clock cycle (when needed)

### 5. ✏️ Write Back (WB)

- Writes results back to register file
- Selects appropriate data source (ALU result or memory data)
- Updates destination registers
- Completes instruction execution
- **Duration**: 1 clock cycle

### Multi-Cycle Execution Flow

Unlike pipelined processors, each instruction completes all stages before the next instruction begins:

```
Instruction 1: IF → ID → EX → MEM → WB
Instruction 2:           IF → ID → EX → MEM → WB
Instruction 3:                     IF → ID → EX → MEM → WB
```

This approach provides:

- **Simplified Control**: No pipeline hazards to manage
- **Clear State Transitions**: Each stage completes before moving to next
- **Easier Debugging**: Sequential execution is easier to trace
- **Educational Value**: Clear understanding of processor operation

## 🛠️ Implementation Details

### Control Unit

The control unit implements a finite state machine managing the multi-cycle execution stages:

```verilog
// State definitions for multi-cycle execution
`define STG_FTCH 3'b000  // Fetch stage
`define STG_DCDE 3'b001  // Decode stage
`define STG_EXEC 3'b010  // Execute stage
`define STG_MEM  3'b011  // Memory stage
`define STG_WRB  3'b100  // Write-back stage
`define STG_INIT 3'b101  // Initial state
```

**Multi-Cycle Control Flow:**

- Each instruction progresses sequentially through all required stages
- State machine ensures proper timing and control signal generation
- Different instruction types may skip certain stages (e.g., R-type skips MEM)
- Control signals are generated based on current stage and instruction type

### ALU Operations

The ALU supports multiple operations with comprehensive flag generation:

- **Arithmetic**: Addition, Subtraction with overflow detection
- **Logic**: Bitwise AND operations
- **Comparison**: All branch condition evaluations
- **Flag Generation**: Zero, Negative, Overflow flags

### Memory System

- **Instruction Memory**: 128 × 16-bit, read-only
- **Data Memory**: 256 × 16-bit, supports word/byte access
- **Addressing**: Byte-addressable with automatic alignment

## 🧪 Verification & Testing

### Comprehensive Test Suite

The project includes extensive verification through multiple testbenches:

#### Individual Component Tests

- **Control Unit Testbench**: Verifies state transitions and control signal generation
- **ALU Testbench**: Tests all arithmetic/logic operations and flag generation
- **Register File Testbench**: Validates read/write operations and register access
- **Data Memory Testbench**: Tests memory operations including byte access
- **Instruction Memory Testbench**: Verifies instruction fetch operations

#### Integration Tests

- **Processor Testbench**: Complete system-level testing
- **Instruction Sequence Tests**: Validates program execution flow
- **Multi-Cycle Verification**: Ensures correct stage transitions and timing

#### Test Coverage

- ✅ All 21 instructions tested individually
- ✅ Multi-cycle execution scenarios
- ✅ Branch and jump instruction accuracy
- ✅ Memory access patterns
- ✅ Edge cases and error conditions

### Sample Test Programs

```verilog
// Example: Addition Loop Program
instruction_memory[1] = {ADDI, 1'b0, R1, R2, 5'b00101};  // Load immediate
instruction_memory[2] = {ADD, R1, R0, R2, 3'b001};       // Add operation
instruction_memory[3] = {SW, 1'b1, R1, R2, 5'b00000};    // Store result

// Example: Branch Test Program
instruction_memory[4] = {BEQ, 1'b0, R1, R1, 5'b00011};   // Branch if equal
instruction_memory[5] = {BGTZ, 1'b1, R6, R0, 5'b00010};  // Branch if > 0
```

## 📁 Project Structure

```
Our_arc/
├── src/
│   ├── Our_arc.v              # Main processor implementation
│   ├── ControlUnitWaveForm.*  # Simulation waveforms
│   └── wave.asdb              # Additional waveform data
├── compile/                   # Compilation artifacts
├── log/                       # Simulation logs
├── Simulink/                  # MATLAB/Simulink models
│   └── dataMemoryTestBench.m
├── test_results/              # Verification results
│   ├── stage1_test_results.txt
│   └── stage2_test_results.txt
└── README.md                  # This file
```

## 🚀 Getting Started

### Prerequisites

- ModelSim/QuestaSim or similar Verilog simulator
- Verilog synthesis tools (optional)
- Text editor or Verilog IDE

### Compilation & Simulation

1. **Clone the repository**

   ```bash
   git clone https://github.com/HasanQarmash/Multi-Cycle-RISC-Processor-Implementation.git
   cd Multi-Cycle-RISC-Processor-Implementation
   ```

2. **Compile the design**

   ```bash
   vlog src/Our_arc.v
   ```

3. **Run simulation**

   ```bash
   vsim -do "run -all" riscProcessor
   ```

4. **View waveforms**
   ```bash
   vsim -view controlUnitTestBench.vcd
   ```

### Running Tests

Execute individual component tests:

```bash
# ALU test
vsim ALUTestBench

# Control Unit test
vsim controlUnitTestBench

# Data Memory test
vsim dataMemoryTestBench

# Register File test
vsim registerFileTestBench
```

## 📊 Results & Performance

### Verification Results

- ✅ **100% Instruction Coverage**: All 21 instructions verified
- ✅ **Multi-Cycle Integrity**: All 5 stages functioning correctly
- ✅ **Memory Operations**: Word and byte access validated
- ✅ **Branch Operations**: Conditional branches working accurately
- ✅ **Flag Generation**: Zero, negative, overflow flags correct

### Performance Metrics

- **Clock Frequency**: Optimized for educational simulation
- **CPI**: Variable cycles per instruction (3-5 cycles depending on instruction type)
- **Execution Model**: Sequential multi-cycle execution
- **Memory Latency**: Single-cycle access for both instruction and data
- **Register Access**: Dual-port read, single-port write

### Multi-Cycle Performance Characteristics

| Instruction Type            | Cycles Required | Stages Used             |
| --------------------------- | --------------- | ----------------------- |
| **R-Type** (ADD, SUB, AND)  | 4 cycles        | IF → ID → EX → WB       |
| **I-Type ALU** (ADDI, ANDI) | 4 cycles        | IF → ID → EX → WB       |
| **Load** (LW, LBs, LBu)     | 5 cycles        | IF → ID → EX → MEM → WB |
| **Store** (SW, SV)          | 4 cycles        | IF → ID → EX → MEM      |
| **Branch** (BEQ, BNE, etc.) | 3 cycles        | IF → ID → EX            |
| **Jump** (JMP, CALL, RET)   | 3 cycles        | IF → ID → EX            |

### Simulation Results

The processor successfully executes complex instruction sequences including:

- Arithmetic operations with immediate values
- Memory load/store operations (word and byte)
- Conditional branch instructions
- Subroutine calls and returns
- Mixed instruction type programs

## 👥 Team & Contributions

### Project Team

- **Hasan Qarmash** (ID: 1210611)
  - Lead Developer & System Architect
  - Control Unit & Multi-Cycle Implementation
  - Integration & System Testing

### Supervision

- **Dr. Aziz Qaroush** - Course Instructor
- **Dr. Ayman Hroub** - Course Instructor

### Development Timeline

- **Project Start**: June 2024
- **Completion**: June 22, 2024, 6:00 PM Saturday
- **Duration**: Intensive development cycle

## 🎓 Academic Information

### Course Details

- **Course**: ENC54370 - Computer Architecture
- **Institution**: Birzeit University
- **Department**: Electrical and Computer Engineering
- **Semester**: Second Semester, 2023/2024
- **Project**: #2 - RISC Processor Design

### Learning Objectives

This project demonstrates:

- Deep understanding of computer architecture principles
- Practical implementation of multi-cycle processor design
- Verification and testing methodologies
- Hardware description language (Verilog) proficiency
- System integration and debugging skills

## 📄 License

This project is developed for academic purposes at Birzeit University. The code is available for educational use and reference.

## 🤝 Contributing

This is an academic project, but suggestions and improvements are welcome! Feel free to:

- Report issues or bugs
- Suggest optimizations
- Share educational insights
- Propose additional test cases

## 📞 Contact

**Hasan Qarmash**  
📧 [Email : qarmash.hasan@gmail.com]  
🎓 Birzeit University - Electrical and Computer Engineering

---

⭐ **Star this repository if you found it helpful for learning computer architecture!**

_Last Updated: June 2024_
