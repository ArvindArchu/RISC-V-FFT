`timescale 1ns / 1ps
`include "opcodes.vh"


module control_unit(
    input [6:0] opcode,
    output      RegWrite,
    output      MemRead,  
    output      MemWrite,
    output      MemToReg,
    output      ALUSrc,
    output      Branch
    );

    
    reg regwrite_r, memread_r, memwrite_r, memtoreg_r, alusrc_r, branch_r;  
    always @(*) begin
        // defaults (NOP)
        regwrite_r = 1'b0;
        memread_r  = 1'b0;
        memwrite_r = 1'b0;
        memtoreg_r = 1'b0;
        alusrc_r   = 1'b0;
        branch_r   = 1'b0;

        case (opcode)
            `OP_RTYPE: begin
                regwrite_r = 1'b1; // write ALU/MAC result to rd
                alusrc_r   = 1'b0; // B = rs2
            end
            `OP_ITYPE: begin
                regwrite_r = 1'b1; // write ALU result to rd
                alusrc_r   = 1'b1; // B = I-immediate
            end
            `OP_LOAD: begin
                regwrite_r = 1'b1; // write memory data to rd
                memread_r  = 1'b1;
                memtoreg_r = 1'b1; // select MEM data for WB
                alusrc_r   = 1'b1; // address = rs1 + imm_i
            end
            `OP_STORE: begin
                memwrite_r = 1'b1; // write rs2 to memory
                alusrc_r   = 1'b1; // address = rs1 + imm_s
            end
            // BRANCH (BEQ, BNE)
            `OP_BRANCH: begin
                branch_r   = 1'b1;
                alusrc_r   = 1'b0;
            end

            // JAL (unconditional PC-relative jump)
            `OP_JAL: begin
                regwrite_r = 1'b1; // write return address to rd
                branch_r   = 1'b1; // unconditional branch
            end

            // JALR (register + immediate jump)
            `OP_JALR: begin
                regwrite_r = 1'b1;
                branch_r   = 1'b1; // unconditional
                alusrc_r   = 1'b1; // use rs1 + imm_i
            end
            default: begin
                // keep defaults
            end
        endcase
    end

    assign RegWrite = regwrite_r;
    assign MemRead  = memread_r;
    assign MemWrite = memwrite_r;
    assign MemToReg = memtoreg_r;
    assign ALUSrc   = alusrc_r;
    assign Branch   = branch_r;
endmodule
