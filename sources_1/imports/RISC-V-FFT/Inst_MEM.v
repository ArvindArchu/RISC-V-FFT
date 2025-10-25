

`timescale 1ns / 1ps

module Inst_MEM(
    input  [31:0] address,   // PC value
    output reg [31:0] inst
    );
    reg [31:0] mem [0:255]; // 256 instructions of 32-bit each
    integer i;
    initial begin
        /*
            wire [6:0] opcode1 = inst1_reg[6:0];
            wire [4:0] rd1 = inst1_reg[11:7];
            wire [2:0] funct3_1 = inst1_reg[14:12];
            wire [4:0] rs1_1 = inst1_reg[19:15];
            wire [4:0] rs2_1 = inst1_reg[24:20];
            wire [6:0] funct7_1 = inst1_reg[31:25];


            btype :
            imm[12]: 1

            imm[10:5]: 111111

            rs2 (x2): 00010

            rs1 (x1): 00001

            funct3 (bne): 001

            imm[4:1]: 1000

            imm[11]: 1

            opcode (branch): 1100011
        */
        
        // fill with NOPs (ADDI x0, x0, 0) so reads never return X/Z
        for (i = 0; i < 256; i = i + 1)
            mem[i] = 32'b000000000000_00000_000_00000_0010011; // ADDI x0,x0,0 -> NOP

       
        



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
        mem[9] = 32'b000000000000_00000_000_00000_0010011; // nop

       
        
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
    */
     /*
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
         
        */

