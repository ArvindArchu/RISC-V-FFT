`timescale 1ns / 1ps

module Inst_MEM(
    input  [31:0] address,   // PC value
    output reg [31:0] inst
    );
    reg [31:0] mem [0:255]; // 256 instructions of 32-bit each
    initial begin
        // Example RISC-V encoded instructions (pseudo-binary, fix with assembler)
        // ADDI x1, x0, 5
        mem[0] = 32'b000000000101_00000_000_00001_0010011; 
        // ADDI x2, x0, 3
        mem[1] = 32'b000000000011_00000_000_00010_0010011; 
        // ADD x3, x1, x2
        mem[2] = 32'b0000000_00010_00001_000_00011_0110011; 
        mem[3] = 32'b000000000010_00000_000_00100_0010011; // ADDI x4, x0, 2
        mem[4] = 32'b000000000011_00000_000_00101_0010011; // ADDI x5, x0, 3
        mem[5] = 32'b0000001_00101_00100_000_00110_0110011; // MUL x6, x4, x5
    end

    always @(*) begin
        inst = mem[address[9:2]]; 
        // drop lower 2 bits (word aligned)
    end



endmodule
