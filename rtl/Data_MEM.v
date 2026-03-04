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
    // always @(posedge clk) begin
    //     if (!reset) begin
    //         $monitor("DMEM_READ   t=%0t byte_addr=%0d word_idx=%0d read_DAT=%h mem[%0d]=%h",
    //                  $time, address, word_index, mem[word_index], word_index, mem[word_index]);
    //     end
    // end

endmodule

