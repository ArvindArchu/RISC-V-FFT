`timescale 1ns / 1ps
module pc(
    input           clk,
    input           reset,     // active-high reset
    input  [31:0]   pc_next,
    output [31:0]   pc
);
    reg [31:0] pc_reg;
    assign pc = pc_reg;

    always @(posedge clk or posedge reset) begin
        if (reset)
            pc_reg <= 32'd0;
        else
            pc_reg <= pc_next;
    end
endmodule
