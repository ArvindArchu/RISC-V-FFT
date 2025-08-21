`timescale 1ns / 1ps

module MAC(
    input [31:0] A,
    input [31:0] B,
    input [4:0] ctrl,
    output reg [31:0] Y
    );

wire signed [31:0] As = A;
wire signed [31:0] Bs = B;
wire [63:0] mulU = A * B;
wire signed [63:0] mulS = As * Bs;
wire signed [63:0] mulHSU = As * B;

always @(*) begin
    case (ctrl)
        4'd10: Y = mulU[31:0];   // MUL
        4'd11: Y = mulS[63:32];  // MULH (signed * signed high 32)
        4'd12: Y = mulU[63:32];  // MULHU (unsigned * unsigned high 32)
        4'd13: Y = mulHSU[63:32];// MULHSU (signed * unsigned high 32)
        default: Y = 32'b0;
    endcase
end
endmodule
