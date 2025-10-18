
`timescale 1ns / 1ps
module register_file_dual(
    input clk,
    input reset,
    input we1, we2,              // write enables for inst1 & inst2
    input [4:0] rs1_1, rs2_1,    // instruction 1 sources
    input [4:0] rs1_2, rs2_2,    // instruction 2 sources
    input [4:0] rd1, rd2,        // instruction 1 and 2 destinations
    input [31:0] wdata1, wdata2, // write-back data
    output [31:0] rdata1_1, rdata2_1, // instruction 1 reads
    output [31:0] rdata1_2, rdata2_2  // instruction 2 reads
);
    reg [31:0] regs [0:31];

    // Writes (simple priority: inst1 first)
    always @(posedge clk) begin
        if (we1 && rd1 != 0) regs[rd1] <= wdata1;
        if (we2 && rd2 != 0) regs[rd2] <= wdata2;
    end
    // Reads (combinational)
    assign rdata1_1 = (rs1_1 != 0) ? regs[rs1_1] : 32'b0;
    assign rdata2_1 = (rs2_1 != 0) ? regs[rs2_1] : 32'b0;
    assign rdata1_2 = (rs1_2 != 0) ? regs[rs1_2] : 32'b0;
    assign rdata2_2 = (rs2_2 != 0) ? regs[rs2_2] : 32'b0;
endmodule

/*
module register_file(
    input reset,
    input clk,
    input we,                    // write enable
    input [4:0] rs1, rs2, rd,    // register addresses
    input [31:0] wdata,          // data to write
    output [31:0] rdata1, rdata2 // data read
);

    reg [31:0] regs[31:0];
    integer i;
    // write on clock edge
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                regs[i] <= 32'b0;
            end
        end
        else if (we && rd != 0) begin
            $display("WRITE: x%0d <= %h", rd, wdata);
            regs[rd] <= wdata;
        end
    end

    // read combinationally
    assign rdata1 = regs[rs1];
    assign rdata2 = regs[rs2];

endmodule
*/