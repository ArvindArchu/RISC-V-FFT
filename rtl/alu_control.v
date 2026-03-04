`timescale 1ns / 1ps

module alu_control(
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,
    output reg [4:0] ctrl
    );
always @(*) begin
    case (opcode)
        7'b0110011: begin // R-type
            case ({funct7, funct3})
                10'b0000000_000: ctrl = 5'd0;   // ADD
                10'b0100000_000: ctrl = 5'd1;   // SUB
                10'b0000000_111: ctrl = 5'd2;   // AND
                10'b0000000_110: ctrl = 5'd3;   // OR
                10'b0000000_100: ctrl = 5'd4;   // XOR
                10'b0000000_001: ctrl = 5'd5;   // SLL
                10'b0000000_101: ctrl = 5'd6;   // SRL
                10'b0100000_101: ctrl = 5'd7;   // SRA
                10'b0000000_010: ctrl = 5'd8;   // SLT
                10'b0000000_011: ctrl = 5'd9;   // SLTU

                // M-extension
                10'b0000001_000: ctrl = 5'd10;  // MUL
                10'b0000001_001: ctrl = 5'd11;  // MULH
                10'b0000001_010: ctrl = 5'd13;  // MULHSU
                10'b0000001_011: ctrl = 5'd12;  // MULHU

                default: ctrl = 5'd0;
            endcase
        end
        default: ctrl = 5'd0;
    endcase
end
endmodule
