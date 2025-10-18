`timescale 1ns / 1ps

module ALU(
    input [31:0] A,
    input [31:0] B,
    input [4:0] ctrl,
    output reg [31:0] Y
    );

    //7 bit opcode - specifies instruction type
    //3 bit funct3 - selects operation withing opcode family
    //7 bit funct7 - further differentiates certain operations

always @(*) begin
    case(ctrl)
        4'd0: Y = A + B; // ADD 
        4'd1: Y = A - B; // SUB 
        4'd2: Y = A & B; // AND 
        4'd3: Y = A | B; // OR 
        4'd4: Y = A ^ B; // XOR 
        4'd5: Y = A << B[4:0]; // SLL essentially multiples by 2^B
        4'd6: Y = A >> B[4:0]; // SRL (logical) fills zero on left
        4'd7: Y = $signed(A) >>> B[4:0]; // SRA (arith) preserves sign 
        4'd8: Y = ($signed(A) < $signed(B)) ? 32'd1 : 32'd0; // SLT comparitive operator
        4'd9: Y = (A < B) ? 32'd1 : 32'd0; // SLTU unsigned comparitive operator                
        default: Y = 32'd0; 
    endcase
end
endmodule
