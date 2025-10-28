`timescale 1ns / 1ps

module Data_MEM(
    input           clk,
    input           reset,
    input           write_en,
    input  [31:0]   address,
    input  [31:0]   write_DAT,
    output [31:0]   read_DAT
);

    reg [31:0] mem [0:255];
    integer i;
    
    // Convert byte address to word index (divide by 4)
    wire [7:0] word_index = address[9:2];
    
    // Synchronous write
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 256; i = i + 1)
                mem[i] <= 32'd0;
                
            // FFT test data - 8 samples at word indices 0-7
            // FORMAT: {Im[15:0], Re[15:0]}
            //mem[0] <= {16'd0, 16'd32767};     // word 0 (byte addr 0):  Re=32767, Im=0
            //mem[1] <= {16'd0, 16'd23169};     // word 1 (byte addr 4):  Re=23169, Im=0
            //mem[2] <= {16'd0, 16'd0};         // word 2 (byte addr 8):  Re=0, Im=0
            //mem[3] <= {16'd0, -16'd23169};    // word 3 (byte addr 12): Re=-23169, Im=0
            //mem[4] <= {16'd0, -16'd32767};    // word 4 (byte addr 16): Re=-32767, Im=0
            //mem[5] <= {16'd0, -16'd23169};    // word 5 (byte addr 20): Re=-23169, Im=0
            //mem[6] <= {16'd0, 16'd0};         // word 6 (byte addr 24): Re=0, Im=0
            //mem[7] <= {16'd0, 16'd23169};     // word 7 (byte addr 28): Re=23169, Im=0
        end else if (write_en) begin
            mem[word_index] <= write_DAT;
            $display("DMEM_WRITE  t=%0t byte_addr=%0d word_idx=%0d data=%h", 
                     $time, address, word_index, write_DAT);
        end
    end
    
    // Combinational read — instant response
    assign read_DAT = mem[word_index];
    
    // Debug monitor (use $monitor instead of always @(*) to reduce spam)
    always @(posedge clk) begin
        if (!reset) begin
            $monitor("DMEM_READ   t=%0t byte_addr=%0d word_idx=%0d read_DAT=%h mem[%0d]=%h",
                     $time, address, word_index, mem[word_index], word_index, mem[word_index]);
        end
    end

endmodule

/*
`timescale 1ns / 1ps

module Data_MEM(
    input           clk,
    input           reset,       // synchronous active-high reset
    input           write_en,    // write enable (sampled on posedge)
    input  [31:0]   address,     // memory address (word addressed)
    input  [31:0]   write_DAT,   // data to write (sampled on posedge)
    output reg [31:0] read_DAT   // registered read output (valid 1 cycle after address)
);

    reg [31:0] mem [0:255]; // 256 words
    integer i;

    // Registered read address (sampled on posedge); read_DAT updated from mem at same posedge
    reg [7:0] read_addr_reg;

    // Synchronous reset and writes
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 256; i = i + 1)
                mem[i] <= 32'd0;
            read_addr_reg <= 8'd0;
            read_DAT      <= 32'd0;
        end else begin
            // sample address into register (this makes read synchronous)
            read_addr_reg <= address[7:0];

            // synchronous write (happens at posedge)
            if (write_en) begin
                mem[address[7:0]] <= write_DAT;
                $display("DMEM_WRITE  t=%0t addr=%0d data=%h write_en=%b", $time, address, write_DAT, write_en);
            end

            // update read output from the memory location selected by the previously sampled address
            // (this gives stable read_DAT one cycle after address was applied)
            read_DAT <= mem[read_addr_reg[7:0]];
            $display("DMEM_STATE  t=%0t input_addr=%0d sampled_idx=%0d read_DAT=%h mem_at_sampled=%h",
            $time, address, read_addr_reg, read_DAT, mem[read_addr_reg]);
        end
    end

endmodule





/*

`timescale 1ns / 1ps

module Data_MEM(
    input clk,
    input reset,
    input write_en,                  // write enable
    input [31:0] address,            // memory address
    input [31:0] write_DAT,          // data to write
    output reg [31:0] read_DAT       // data read
);

    reg [31:0] mem [0:255];  // 256 words of memory
    integer i;
    // Read (combinational or synchronous)
    always @(*) begin
        if (reset) begin
            for (i = 0; i < 255; i = i + 1)
                mem[i] <= 32'd0;
        end
        read_DAT = mem[address[7:0]];  // only lower 8 bits as index
    end

    // Write
    always @(posedge clk) begin
        if (write_en)
            mem[address[7:0]] <= write_DAT;
    end

endmodule

*/
