`timescale 1ns / 1ps


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

