

`timescale 1ns / 1ps

module Inst_MEM(
    input  [31:0] address,   // PC value
    output reg [31:0] inst
    );
    reg [31:0] mem [0:255]; // 256 instructions of 32-bit each
    integer i;
    initial begin
        // fill with NOPs (ADDI x0, x0, 0) so reads never return X/Z
        for (i = 0; i < 256; i = i + 1)
            mem[i] = 32'b000000000000_00000_000_00000_0010011; // ADDI x0,x0,0 -> NOP

            mem[0] = 32'h00000537;  // LUI x10, 0x0 -> x10 = 0x00000000 (base addr)
            mem[1] = 32'h02000593;  // ADDI x11, x0, 32 -> x11 = 0x00000020 (end addr)
            mem[2] = 32'h00B5000B;  // CUSTOM FFT x0, x10, x11  (start FFT)
            mem[3] = 32'h00000013;  // NOP (addi x0, x0, 0)
            mem[4] = 32'h00000013;  // NOP
            mem[5] = 32'h00000013;  // NOP

            // ========================================================================
            // Extended Superscalar Demo - Pure ALU Operations
            // Demonstrates parallel execution across multiple cycles
            // ========================================================================
            mem[6]  = 32'h00100093; // addi x1, x0, 1
            mem[7]  = 32'h00200113; // addi x2, x0, 2
            // Cycle 5 (PC=32)
            mem[8]  = 32'h00300193; // addi x3, x0, 3
            mem[9]  = 32'h00400213; // addi x4, x0, 4
            // Cycle 6 (PC=40)
            mem[10] = 32'h00500293; // addi x5, x0, 5
            mem[11] = 32'h00600313; // addi x6, x0, 6

            mem[12]  = 32'h00100093; // addi x1, x0, 1
            mem[13]  = 32'h00200113; // addi x2, x0, 2
            // Cycle 5 (PC=32)
            mem[14]  = 32'h00300193; // addi x3, x0, 3
            mem[15]  = 32'h00400213; // addi x4, x0, 4
            // Cycle 6 (PC=40)
            mem[16] = 32'h00500293; // addi x5, x0, 5
            mem[17] = 32'h00600313; // addi x6, x0, 6

            // --- Round 1 Increment (Starts at PC=48) ---
            //mem[12] = 32'b000000000001_00001_000_00001_0010011; // addi x1, x1, 1  (x1=2)
            //mem[13] = 32'b000000000001_00010_000_00010_0010011; // addi x2, x2, 1  (x2=3)
            //// Cycle 8 (PC=56)
            //
            //mem[14] = 32'b000000000001_00100_000_00100_0010011; // addi x4, x4, 1  (x4=5)
            // Cycle 9 (PC=64)
            //mem[15] = 32'b000000000001_00101_000_00101_0010011; // addi x5, x5, 1  (x5=6)
            //mem[16] = 32'b000000000001_00110_000_00110_0010011; // addi x6, x6, 1  (x6=7)
            //mem[17] = 32'b000000000001_00011_000_00011_0010011; // addi x3, x3, 1  (x3=4)

            //x0: 0 (Always zero)
            //x1: 4 (Initialized to 1, then incremented 3 times)
            //x2: 5 (Initialized to 2, then incremented 3 times)
            //x3: 6 (Initialized to 3, then incremented 3 times)
            //x4: 7 (Initialized to 4, then incremented 3 times)
            //x5: 8 (Initialized to 5, then incremented 3 times)
            //x6: 9 (Initialized to 6, then incremented 3 times)
        
    end

    always @(*) begin
        inst = mem[address[9:2]]; 
        if (address >= 0 && address < 64)
            $display("IMEM_READ: addr=%0d word_idx=%0d inst=%h", address, address[9:2], mem[address[9:2]]);
    end
endmodule


/*
RISC-V (RV32I) Instruction Format Reference

Here is a reference for the 6 basic instruction formats in the RISC-V 32-bit Integer instruction set.

R-type (Register-Register)

Purpose: Used for register-to-register arithmetic/logic operations.

Examples: add, sub, sll, slt, xor, or, and

Format:

| 31     -     25 | 24  -  20 | 19  -  15 | 14  -  12 | 11   -   7 | 6    -    0 |
|------------------|-----------|-----------|-----------|------------|-------------|
|      funct7      |    rs2    |    rs1    |   funct3  |     rd     |   opcode    |


Fields:

opcode: 0110011 (OP)

rd: Destination register

funct3: Defines the specific operation (e.g., 000 for add/sub)

rs1: Source register 1

rs2: Source register 2

funct7: A secondary operation definition (e.g., 0000000 for add, 0100000 for sub)

I-type (Immediate)

Purpose: Used for operations with a 12-bit immediate, including loads.

Examples: addi, slti, xori, lw, jalr

Format:

| 31     -     20 | 19  -  15 | 14  -  12 | 11   -   7 | 6    -    0 |
|------------------|-----------|-----------|------------|-------------|
|     imm[11:0]    |    rs1    |   funct3  |     rd     |   opcode    |


Fields:

opcode: Varies (e.g., 0010011 for OP-IMM, 0000011 for LOAD, 1100111 for JALR)

rd: Destination register

funct3: Defines the specific operation

rs1: Source register 1

imm[11:0]: A 12-bit sign-extended immediate value.

S-type (Store)

Purpose: Used for store instructions (e.g., sw, sh, sb).

Format:

| 31     -     25 | 24  -  20 | 19  -  15 | 14  -  12 | 11   -   7 | 6    -    0 |
|------------------|-----------|-----------|-----------|------------|-------------|
|     imm[11:5]    |    rs2    |    rs1    |   funct3  |  imm[4:0]  |   opcode    |


Fields:

opcode: 0100011 (STORE)

imm[4:0]: Lower 5 bits of the immediate

funct3: Defines store size (e.g., 010 for sw)

rs1: Base address register

rs2: Source register (data to be stored)

imm[11:5]: Upper 7 bits of the immediate

Immediate: The 12-bit immediate is re-assembled from {imm[11:5], imm[4:0]} and sign-extended.

B-type (Branch)

Purpose: Used for conditional branch instructions (e.g., beq, bne).

Format:

| 31           | 30     -     25 | 24  -  20 | 19  -  15 | 14  -  12 | 11   -   8 | 7            | 6    -    0 |
|--------------|------------------|-----------|-----------|-----------|------------|--------------|-------------|
|   imm[12]    |     imm[10:5]    |    rs2    |    rs1    |   funct3  |  imm[4:1]  |   imm[11]    |   opcode    |


Fields:

opcode: 1100011 (BRANCH)

imm[11]: Bit 11 of the immediate

imm[4:1]: Bits 4:1 of the immediate

funct3: Defines branch type (e.g., 000 for beq, 001 for bne)

rs1: Source register 1 (for comparison)

rs2: Source register 2 (for comparison)

imm[10:5]: Bits 10:5 of the immediate

imm[12]: Bit 12 of the immediate

Immediate: The 13-bit immediate (for a 12-bit offset) is re-assembled from {imm[12], imm[11], imm[10:5], imm[4:1], 1'b0} and 
sign-extended. The target address is PC + imm.

U-type (Upper Immediate)

Purpose: Used to load a 20-bit immediate into the upper 20 bits of a register.

Examples: lui (Load Upper Immediate), auipc (Add Upper Immediate to PC)

Format:

| 31     -     12 | 11   -   7 | 6    -    0 |
|------------------|------------|-------------|
|     imm[31:12]   |     rd     |   opcode    |


Fields:

opcode: 0110111 (LUI) or 0010111 (AUIPC)

rd: Destination register

imm[31:12]: A 20-bit immediate value.

Value:

For lui, rd is loaded with {imm[31:12], 12'b0}.

For auipc, rd is loaded with PC + {imm[31:12], 12'b0}.

J-type (Jump)

Purpose: Used for unconditional jumps (e.g., jal).

Format:

| 31           | 30     -     21 | 20           | 19     -     12 | 11   -   7 | 6    -    0 |
|--------------|------------------|--------------|------------------|------------|-------------|
|   imm[20]    |     imm[10:1]    |   imm[11]    |     imm[19:12]   |     rd     |   opcode    |


Fields:

opcode: 1101111 (JAL)

rd: Destination register (to store link address, PC+4)

imm[19:12]: Bits 19:12 of the immediate

imm[11]: Bit 11 of the immediate

imm[10:1]: Bits 10:1 of the immediate

imm[20]: Bit 20 of the immediate

Immediate: The 21-bit immediate (for a 20-bit offset) is re-assembled from {imm[20], imm[19:12], imm[11], imm[10:1], 1'b0} 
and sign-extended. 

The target address is PC + imm.


*/

/*

  //-------------- Test: Load/Store verification ---------------
        // Format: 32'b [funct7][rs2][rs1][funct3][rd][opcode]

        // ADDI x1, x0, 5
        mem[0] = 32'b000000000101_00000_000_00001_0010011;
        // ADDI x2, x0, 3
        mem[1] = 32'b000000000011_00000_000_00010_0010011;
        // ADD x3, x1, x2  => x3 = 8
        mem[2] = 32'b0000000_00010_00001_000_00011_0110011;
        // ADDI x10, x0, 16 (base address)
        mem[3] = 32'b0000000010000_00000_000_01010_0010011;  // imm=16, rd=10
        // SW x3, 0(x10) => store x3 into address 16
        // funct7=0000000, rs2=00011, rs1=01010, funct3=010, imm[11:5]=0000000, imm[4:0]=00000
        mem[4] = 32'b0000000_00011_01010_010_00000_0100011;
        // LW x4, 0(x10) => load from address 16 into x4
        mem[5] = 32'b000000000000_01010_010_00100_0000011;
        // ADD x5, x3, x4 => x5 = x3 + x4 (should be 16)
        mem[6] = 32'b0000000_00100_00011_000_00101_0110011;
        // (Optional NOP or halt placeholder)
        mem[7] = 32'b0000000_00000_00000_000_00000_0000000;


    // ---------- Branch & Jump Test Program ----------

    // 0: ADDI x1, x0, 5
    mem[0] = 32'b000000000101_00000_000_00001_0010011;
    // 4: ADDI x2, x0, 5
    mem[1] = 32'b000000000101_00000_000_00010_0010011;
    // 8: BEQ x1, x2, +8  (branch to mem[4] i.e. PC+8 -> index 2)
    mem[2] = 32'b0000000_00010_00001_000_00100_1100011;
    // 12: ADDI x3, x0, 111   (skipped if BEQ taken)
    mem[3] = 32'b000001101111_00000_000_00011_0010011;
    // 16: ADDI x3, x0, 222   (executed, target of BEQ)
    mem[4] = 32'b000011011110_00000_000_00011_0010011;
    // 20: BNE x1, x2, +8   (not taken)
    mem[5] = 32'b0000000_00010_00001_001_00100_1100011;
    // 24: ADDI x4, x0, 333   (executed)
    mem[6] = 32'b000101001101_00000_000_00100_0010011;
    // 28: JAL x5, +8 (unconditional jump to mem[9])
    mem[7] = 32'b00000001000000000000001011101111;  // CORRECT for jal x5, +8
    // 32: ADDI x5, x0, 444   (skipped)
    mem[8] = 32'b000110111100_00000_000_00101_0010011;
    // 36: ADDI x5, x0, 555   (executed after jump)
    mem[9] = 32'b001000101011_00000_000_00101_0010011;


         //         STORE x4, 4(x10)
        //mem[3] = 32'b0000000_00100_01010_010_00100_0100011;
        /*
        // ADDI x1, x0, 5
    mem[0] = 32'b000000000101_00000_000_00001_0010011;
    // ADD x2, x1, x1   --> RAW hazard (reads x1 written by inst1)
    mem[1] = 32'b0000000_00001_00001_000_00010_0110011;

    // WAW: Both instructions write to x1
    //mem[2] = 32'b000000000101_00000_000_00001_0010011; // ADDI x1, x0, 5
    //mem[3] = 32'b000000000111_00000_000_00001_0010011; // ADDI x1, x0, 7
    //mem[4] = 32'b000000000101_00000_000_00001_0010011; // ADDI x1, x0, 5
    //mem[5] = 32'b0000000_00001_00001_000_00010_0110011;// ADD x2, x1, x1   --> RAW hazard (reads x1 written by inst1)
    // LOADUSE: LW followed immediately by ADD using loaded value
    //mem[4] = 32'b000000000000_00000_010_00001_0000011; // LW x1, 0(x0)
    //mem[5] = 32'b0000000_00001_00001_000_00010_0110011; // ADD x2, x1, x1
    //mem[6] = 32'b0000000_00001_00010_000_00011_0110011; // ADD x3, x1, x2
    //mem[7] = 32'b0000000_00011_00100_000_00101_0110011; // ADD x5, x3, x4
    //mem[0] = 32'b000000000101_00000_000_00001_0010011; // ADDI x1, x0, 5
    //mem[1] = 32'b000000000011_00000_000_00010_0010011; // ADDI x2, x0, 3   (independent -> dual-issue)
    //mem[2] = 32'b0000000_00010_00001_000_00011_0110011; // ADD x3, x1, x2
    //mem[3] = 32'b0000001_00010_00001_000_00100_0110011; // MUL x4, x1, x2   (MAC)
    //mem[0] = 32'b000000000000_00000_000_01010_0010011; // ADDI x10, x0, 0      ; base address
    //mem[1] = 32'b000000000000_01010_010_00001_0000011; // LW x1, 0(x10)
    //mem[2] = 32'b000000000100_01010_010_00010_0000011; // LW x2, 4(x10)
    //mem[3] = 32'b0000000_00001_00010_000_00011_0110011; // ADD x3, x1, x2

     // ADDI x1, x0, 5
        mem[0] = 32'b000000000101_00000_000_00001_0010011;
        // ADDI x2, x0, 3
        mem[1] = 32'b000000000011_00000_000_00010_0010011;
        // ADD x3, x0, 8  => x3 = 8
        mem[2] = 32'b0000000_00010_00001_000_00011_0110011;
        // ADDI x10, x0, 8 (base address)
        mem[3] = 32'b00000000_01000_00000_000_01010_0010011;  // imm=8, rd=10
        // SW x3, 0(x10) => store x3 into address 8
        // funct7=0000000, rs2=00011, rs1=01010, funct3=010, imm[11:5]=0000000, imm[4:0]=00000
        mem[4] = 32'b0000000_00011_01010_010_00000_0100011;
        // LW x4, 0(x10) => load from address 8 into x4
        mem[5] = 32'b000000000000_01010_010_00100_0000011;
        // ADD x5, x4, x0  => x5 = x4
        mem[7] = 32'b0000000_00000_00100_000_00101_0110011;
    
        //test 1 Simple Branch (BEQ taken)
        // Test if BEQ works when condition is true
        mem[0] = 32'b000000000000_00000_000_00001_0010011; // addi x1, x0, 0 (x1=0)
        mem[1] = 32'b000000000101_00000_000_00010_0010011; // addi x2, x0, 5 (x2=5)
        mem[2] = 32'b0_000001_00000_00001_000_0000_0_1100011; // beq x1, x0, +8 (should branch to mem[4])
        mem[3] = 32'b000000001010_00010_000_00010_0010011; // addi x2, x2, 10 (SKIP - x2 should stay 5)
        mem[4] = 32'b000000000001_00010_000_00010_0010011; // addi x2, x2, 1 (x2=6)
        mem[5] = 32'b000000000000_00000_000_00000_0010011; // nop (end)

        // Expected: x1=0, x2=6 (proves branch taken, skipped mem[3])

        //test 2 Simple Branch (BEQ not taken)
        mem[0] = 32'b000000000101_00000_000_00001_0010011; // addi x1, x0, 5 (x1=5)
        mem[1] = 32'b000000000010_00000_000_00010_0010011; // addi x2, x0, 2 (x2=2)
        mem[2] = 32'b0_000001_00000_00001_000_0000_0_1100011; // beq x1, x0, +8 (should NOT branch)
        mem[3] = 32'b000000000011_00010_000_00010_0010011; // addi x2, x2, 3 (x2=5)
        mem[4] = 32'b000000000001_00010_000_00010_0010011; // addi x2, x2, 1 (SHOULD NOT REACH)
        // Expected: x1=5, x2=5 (proves branch not taken, executed mem[3])
        
        // Expected: x1=6, x3=8 (proves JAL jumped correctly and saved return address)
        
        //Test 4 JAL + JALR (call and return)
        mem[0] = 32'b000000000101_00000_000_00001_0010011; // addi x1, x0, 5
        mem[1] = 32'b0_0000001000_0_00000000_00011_1101111; // jal x3, +16 to mem[5], x3=8
        mem[2] = 32'b000000000001_00001_000_00001_0010011; // addi x1, x1, 1 (return here, x1=6)
        mem[3] = 32'b000000000000_00000_000_00000_0010011; // nop (end)
        mem[4] = 32'b000000000000_00000_000_00000_0010011; // nop
        mem[5] = 32'b000000000010_00001_000_00001_0010011; // addi x1, x1, 2 (x1=7)
        mem[6] = 32'b000000000000_00011_000_00000_1100111; // jalr x0, 0(x3) - return to mem[2]

        // Expected: x1=8 (5+2+1), x3=8 (proves call/return works)


        //test 5 Simple backward jump

        mem[0] = 32'b000000000000_00000_000_00001_0010011; // addi x1, x0, 0
        mem[1] = 32'b000000000001_00001_000_00001_0010011; // LOOP: addi x1, x1, 1
        mem[2] = 32'b000000000101_00000_000_00010_0010011; // addi x2, x0, 5
        mem[3] = 32'b1111111_00010_00001_001_1000_1_1100011; // bne x1, x2, -8 (back to mem[1] if x1≠5)
        mem[4] = 32'b000000000000_00000_000_00000_0010011; // nop (end)

        // Expected: x1=5, x2=5 (proves loop executed 5 times)

        mem[0] = 32'b00000000_01000_00000_000_01010_0010011; // addi x10, x0, 8 (base addr)
        mem[1] = 32'b000000000101_00000_000_00001_0010011;   // addi x1, x0, 5
        mem[2] = 32'b0000000_00001_01010_010_00000_0100011;  // sw x1, 0(x10) - store 5 to addr 8
        mem[3] = 32'b000000000000_01010_010_00010_0000011;   // lw x2, 0(x10) - load from addr 8
        mem[5] = 32'b0000000_00000_00010_000_00011_0110011;  // add x3, x2, x0 (use loaded value)
        mem[6] = 32'b000000000001_00011_000_00011_0010011;   // addi x3, x3, 1

         // ========================================================================
        // RISC-V Branch + Subroutine Test Program (CORRECTED)
        // ========================================================================

        // addr 0x00 - SETUP
        // addr 0x00
        mem[0] = 32'b000000000101_00000_000_00001_0010011; // addi x1, x0, 5
        // addr 0x04  
        mem[1] = 32'b000000000000_00000_000_00010_0010011; // addi x2, x0, 0
        // addr 0x08 - LOOP
        mem[2] = 32'b0_000011_00000_00001_000_1100_0_1100011; // beq x1, x0, +28 (to DONE at 0x24)
        // addr 0x0C
        mem[3] = 32'b0_0000001000_0_00000000_00011_1101111; // jal x3, +16 (to SUBROUTINE at 0x1C)
        // addr 0x10
        mem[4] = 32'b111111111111_00001_000_00001_0010011; // addi x1, x1, -1
        // addr 0x14
        mem[5] = 32'b11111111010_1_11111111_00000_1101111; // jal x0, -12 (back to LOOP at 0x08)
        // addr 0x18
        mem[6] = 32'b000000000000_00000_000_00000_0010011; // nop
        // addr 0x1C - SUBROUTINE
        mem[7] = 32'b000000000001_00010_000_00010_0010011; // addi x2, x2, 1
        // addr 0x20
        mem[8] = 32'b000000000000_00011_000_00000_1100111; // jalr x0, 0(x3)
        // addr 0x24 - DONE
        mem[9] = 32'b000000000000_00000_000_00000_0010011; // nops

         // Test that load-use hazard is properly detected and stalled
        mem[0] = 32'b00000000_01000_00000_000_01010_0010011; // addi x10, x0, 8 (base addr)
        mem[1] = 32'b000000000101_00000_000_00001_0010011;   // addi x1, x0, 5
        mem[2] = 32'b0000000_00001_01010_010_00000_0100011;  // sw x1, 0(x10) - store 5 to addr 8
        mem[3] = 32'b000000000000_01010_010_00010_0000011;   // lw x2, 0(x10) - load from addr 8
        mem[4] = 32'b0000000_00000_00010_000_00011_0110011;  // add x3, x2, x0 (use loaded value)
        mem[5] = 32'b000000000001_00011_000_00011_0010011;   // addi x3, x3, 1


          // ========================================================================
        // RISC-V Branch + Subroutine Test Program (CORRECTED)
        // ========================================================================

        // addr 0x00 - SETUP
        // addr 0x00
        mem[0] = 32'b000000000101_00000_000_00001_0010011; // addi x1, x0, 5
        // addr 0x04  
        mem[1] = 32'b000000000000_00000_000_00010_0010011; // addi x2, x0, 0
        // addr 0x08 - LOOP
        mem[2] = 32'b0_000011_00000_00001_000_1100_0_1100011; // beq x1, x0, +28 (to DONE at 0x24)
        // addr 0x0C
        mem[3] = 32'b0_0000001000_0_00000000_00011_1101111; // jal x3, +16 (to SUBROUTINE at 0x1C)
        // addr 0x10
        mem[4] = 32'b111111111111_00001_000_00001_0010011; // addi x1, x1, -1
        // addr 0x14
        mem[5] = 32'b11111111010_1_11111111_00000_1101111; // jal x0, -12 (back to LOOP at 0x08)
        // addr 0x18
        mem[6] = 32'b000000000000_00000_000_00000_0010011; // nop
        // addr 0x1C - SUBROUTINE
        mem[7] = 32'b000000000001_00010_000_00010_0010011; // addi x2, x2, 1
        // addr 0x20
        mem[8] = 32'b000000000000_00011_000_00000_1100111; // jalr x0, 0(x3)
        // addr 0x24 - DONE
        mem[9] = 32'b000000000000_00000_000_00000_0010011; // nops


        // ========================================================================
            // Extended Superscalar Demo - Pure ALU Operations
            // Demonstrates parallel execution across multiple cycles
            // ========================================================================

            // Initialize registers
            mem[0] = 32'b000000000001_00000_000_00001_0010011;   // addi x1, x0, 1
            mem[1] = 32'b000000000010_00000_000_00010_0010011;   // addi x2, x0, 2
            mem[2] = 32'b000000000011_00000_000_00011_0010011;   // addi x3, x0, 3
            mem[3] = 32'b000000000100_00000_000_00100_0010011;   // addi x4, x0, 4
            mem[4] = 32'b000000000101_00000_000_00101_0010011;   // addi x5, x0, 5
            mem[5] = 32'b000000000110_00000_000_00110_0010011;   // addi x6, x0, 6

            // Independent ADD operations (no dependencies between pairs)
            // These should execute in parallel (2 per cycle)
            mem[6] = 32'b0000000_00010_00001_000_00111_0110011;  // add x7, x1, x2  (1+2=3)
            mem[7] = 32'b0000000_00100_00011_000_01000_0110011;  // add x8, x3, x4  (3+4=7)

            mem[8] = 32'b0000000_00110_00101_000_01001_0110011;  // add x9, x5, x6  (5+6=11)
            mem[9] = 32'b0000000_00010_00001_000_01010_0110011;  // add x10, x1, x2 (1+2=3)

            mem[10] = 32'b0000000_00100_00011_000_01011_0110011; // add x11, x3, x4 (3+4=7)
            mem[11] = 32'b0000000_00110_00101_000_01100_0110011; // add x12, x5, x6 (5+6=11)

            mem[12] = 32'b0000000_00010_00001_000_01101_0110011; // add x13, x1, x2 (1+2=3)
            mem[13] = 32'b0000000_00100_00011_000_01110_0110011; // add x14, x3, x4 (3+4=7)

            mem[14] = 32'b0000000_00110_00101_000_01111_0110011; // add x15, x5, x6 (5+6=11)
            mem[15] = 32'b0000000_00010_00001_000_10000_0110011; // add x16, x1, x2 (1+2=3)

            mem[16] = 32'b0000000_00100_00011_000_10001_0110011; // add x17, x3, x4 (3+4=7)
            mem[17] = 32'b0000000_00110_00101_000_10010_0110011; // add x18, x5, x6 (5+6=11)

            mem[18] = 32'b0000000_00010_00001_000_10011_0110011; // add x19, x1, x2 (1+2=3)
            mem[19] = 32'b0000000_00100_00011_000_10100_0110011; // add x20, x3, x4 (3+4=7)

            mem[20] = 32'b0000000_00110_00101_000_10101_0110011; // add x21, x5, x6 (5+6=11)
            mem[21] = 32'b0000000_00010_00001_000_10110_0110011; // add x22, x1, x2 (1+2=3)

            mem[22] = 32'b0000000_00100_00011_000_10111_0110011; // add x23, x3, x4 (3+4=7)
            mem[23] = 32'b0000000_00110_00101_000_11000_0110011; // add x24, x5, x6 (5+6=11)

            mem[24] = 32'b0000000_00010_00001_000_11001_0110011; // add x25, x1, x2 (1+2=3)
            mem[25] = 32'b0000000_00100_00011_000_11010_0110011; // add x26, x3, x4 (3+4=7)

            mem[26] = 32'b0000000_00110_00101_000_11011_0110011; // add x27, x5, x6 (5+6=11)
            mem[27] = 32'b0000000_00010_00001_000_11100_0110011; // add x28, x1, x2 (1+2=3)

            mem[28] = 32'b0000000_00100_00011_000_11101_0110011; // add x29, x3, x4 (3+4=7)
            mem[29] = 32'b0000000_00110_00101_000_11110_0110011; // add x30, x5, x6 (5+6=11)

            mem[30] = 32'b0000000_00010_00001_000_11111_0110011; // add x31, x1, x2 (1+2=3)
            mem[31] = 32'b000000000000_00000_000_00000_0010011;  // nop (end)

            // Expected output:
            // x1=1, x2=2, x3=3, x4=4, x5=5, x6=6
            // x7=3, x8=7, x9=11, x10=3, x11=7, x12=11, x13=3, x14=7, x15=11, x16=3
            // x17=7, x18=11, x19=3, x20=7, x21=11, x22=3, x23=7, x24=11, x25=3, x26=7
            // x27=11, x28=3, x29=7, x30=11, x31=3

            // Debug output should show:
            // [ADDI | ADDI] issue2=1  <- Parallel
            // [ADDI | ADDI] issue2=1  <- Parallel
            // [ADDI | ADDI] issue2=1  <- Parallel
            // [ADD | ADD] issue2=1    <- Parallel
            // [ADD | ADD] issue2=1    <- Parallel
            // ... many more parallel ADD pairs ...

       */

