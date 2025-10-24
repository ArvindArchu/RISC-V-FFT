
`timescale 1ns / 1ps
module register_file_dual(
    input         clk,
    input         reset,    // active-high synchronous reset (clears registers to 0)
    input         we1,      // write enable port 1
    input         we2,      // write enable port 2
    input  [4:0]  rs1_1,
    input  [4:0]  rs2_1,
    input  [4:0]  rs1_2,
    input  [4:0]  rs2_2,
    input  [4:0]  rd1,
    input  [4:0]  rd2,
    input  [31:0] wdata1,
    input  [31:0] wdata2,
    output [31:0] rdata1_1,
    output [31:0] rdata2_1,
    output [31:0] rdata1_2,
    output [31:0] rdata2_2
);

    // 32 registers, reg[0] hardwired to zero when read; we still store 0 in reg[0] on reset.
    reg [31:0] regs [0:31];

    // read ports - combinational reads (reflect current regs)
    //assign rdata1_1 = (rs1_1 == 5'd0) ? 32'd0 : regs[rs1_1];
    //assign rdata2_1 = (rs2_1 == 5'd0) ? 32'd0 : regs[rs2_1];
    //assign rdata1_2 = (rs1_2 == 5'd0) ? 32'd0 : regs[rs1_2];
    //assign rdata2_2 = (rs2_2 == 5'd0) ? 32'd0 : regs[rs2_2];
    // Read ports - combinational reads with write-forwarding (write-before-read semantics)
    // Priority rules:
    //  - If rs == 0 -> return 0
    //  - If we1 && rd1 == rs -> forward wdata1 (port1 write has highest priority)
    //  - else if we2 && rd2 == rs -> forward wdata2
    //  - else return regs[rs]
    assign rdata1_1 = (rs1_1 == 5'd0) ? 32'd0 :
                      (we1 && (rd1 == rs1_1)) ? wdata1 :
                      (we2 && (rd2 == rs1_1)) ? wdata2 :
                      regs[rs1_1];

    assign rdata2_1 = (rs2_1 == 5'd0) ? 32'd0 :
                      (we1 && (rd1 == rs2_1)) ? wdata1 :
                      (we2 && (rd2 == rs2_1)) ? wdata2 :
                      regs[rs2_1];

    assign rdata1_2 = (rs1_2 == 5'd0) ? 32'd0 :
                      (we1 && (rd1 == rs1_2)) ? wdata1 :
                      (we2 && (rd2 == rs1_2)) ? wdata2 :
                      regs[rs1_2];

    assign rdata2_2 = (rs2_2 == 5'd0) ? 32'd0 :
                      (we1 && (rd1 == rs2_2)) ? wdata1 :
                      (we2 && (rd2 == rs2_2)) ? wdata2 :
                      regs[rs2_2];

    integer i;
    // synchronous reset + writes
    always @(posedge clk) begin
        if (reset) begin
            // Initialize registers to zero on reset
            for (i = 0; i < 32; i = i + 1)
                regs[i] <= 32'd0;
        end else begin
            // Write port arbitration: give port1 (we1/rd1) priority when both write same rd
            // Only write if rd != 0 (x0 is always zero by architecture)
            if (we1 && (rd1 != 5'd0)) begin
                regs[rd1] <= wdata1;
                $display("REG_WRITE1: t=%0t rd=%0d data=%h", $time, rd1, wdata1);
            end
            // If we2 writes same rd as we1 this cycle, we skip we2 (priority to port1)
            if (we2 && (rd2 != 5'd0) && ~(we1 && (rd1 == rd2))) begin
                regs[rd2] <= wdata2;
                $display("REG_WRITE2: t=%0t rd=%0d data=%h", $time, rd2, wdata2);
            end
        end
    end
    //always @(posedge clk) begin
    //    if (we1)
    //        $display("REGFILE_WRITE1: t=%0t rd=%0d data=%h", $time, rd1, wdata1);
    //    if (we2)
    //        $display("REGFILE_WRITE2: t=%0t rd=%0d data=%h", $time, rd2, wdata2);
    //end

endmodule

/*
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

*/