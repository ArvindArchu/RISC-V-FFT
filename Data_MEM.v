`timescale 1ns / 1ps

module Data_MEM(
    input clk,
    input write_en,                  // write enable
    input [31:0] address,            // memory address
    input [31:0] write_DAT,          // data to write
    output reg [31:0] read_DAT       // data read
);

    reg [31:0] mem [0:255];  // 256 words of memory

    // Read (combinational or synchronous)
    always @(*) begin
        read_DAT = mem[address[7:0]];  // only lower 8 bits as index
    end

    // Write
    always @(posedge clk) begin
        if (write_en)
            mem[address[7:0]] <= write_DAT;
    end

endmodule


