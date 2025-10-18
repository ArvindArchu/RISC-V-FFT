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

        /*
        // ---------- Dual-issue test program ----------
        // Cycle 0: ADDI x1, x0, 5         ; inst1
        mem[0] = 32'b000000000101_00000_000_00001_0010011;
        //         ADDI x2, x0, 3         ; inst2 (independent)
        mem[1] = 32'b000000000011_00000_000_00010_0010011;

        // Cycle 1: ADD x3, x1, x2
        mem[2] = 32'b0000000_00010_00001_000_00011_0110011;
        //         MUL x4, x1, x2
        mem[3] = 32'b0000001_00010_00001_000_00100_0110011;

        // Cycle 2: ADDI x10, x0, 16
        mem[4] = 32'b0000000100000_00000_000_01010_0010011;
        //         ADDI x11, x0, 7
        mem[5] = 32'b000000000111_00000_000_01011_0010011;

        // Cycle 3: STORE x3, 0(x10)
        mem[6] = 32'b0000000_00011_01010_010_00000_0100011;
        //         STORE x4, 4(x10)
        mem[7] = 32'b0000000_00100_01010_010_00100_0100011;

        // Cycle 4: LOAD x12, 0(x10)
        mem[8] = 32'b000000000000_01010_010_01100_0000011;
        //         LOAD x13, 4(x10)
        mem[9] = 32'b000000000100_01010_010_01101_0000011;

        // Cycle 5: ADD x14, x12, x13
        mem[10] = 32'b0000000_01101_01100_000_01110_0110011;
        
        // ADDI x1, x0, 5
    mem[0] = 32'b000000000101_00000_000_00001_0010011;
    // ADD x2, x1, x1   --> RAW hazard (reads x1 written by inst1)
    mem[1] = 32'b0000000_00001_00001_000_00010_0110011;

        */
    // WAW: Both instructions write to x1
    mem[2] = 32'b000000000101_00000_000_00001_0010011; // ADDI x1, x0, 5
    mem[3] = 32'b000000000111_00000_000_00001_0010011; // ADDI x1, x0, 7
    // LOADUSE: LW followed immediately by ADD using loaded value
    mem[4] = 32'b000000000000_00000_010_00001_0000011; // LW x1, 0(x0)
    mem[5] = 32'b0000000_00001_00001_000_00010_0110011; // ADD x2, x1, x1
    mem[6] = 32'b0000000_00001_00010_000_00011_0110011; // ADD x3, x1, x2
    mem[7] = 32'b0000000_00011_00100_000_00101_0110011; // ADD x5, x3, x4



        // rest remain NOPs
    end

    always @(*) begin
        inst = mem[address[9:2]]; 
        // drop lower 2 bits (word aligned)
    end
endmodule
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
        */  