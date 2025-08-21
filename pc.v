`timescale 1ns / 1ps

module pc(
    input clk,
    input reset,
    output reg [31:0] pc
    );

always @(posedge clk or posedge reset) begin
    if (reset)
        pc <= 32'b0;
    else
        pc <= pc + 4;  // increment by 4 each inst is 4 bytes long
end
endmodule
