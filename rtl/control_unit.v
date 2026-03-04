`timescale 1ns / 1ps
`include "opcodes.vh" // Make sure this includes `define OP_CUSTOM0 7'b0001011

module control_unit(
    input  wire [6:0] opcode,
    output wire       RegWrite,
    output wire       MemRead,
    output wire       MemWrite,
    output wire       MemToReg,
    output wire       ALUSrc,
    output wire       Branch,
    output wire       FFTStart // New output signal for FFT
    );


    reg regwrite_r, memread_r, memwrite_r, memtoreg_r, alusrc_r, branch_r;
    reg fftstart_r; // Register for the new signal

    always @(*) begin
        // defaults (NOP or unknown instruction)
        regwrite_r = 1'b0;
        memread_r  = 1'b0;
        memwrite_r = 1'b0;
        memtoreg_r = 1'b0;
        alusrc_r   = 1'b0;
        branch_r   = 1'b0;
        fftstart_r = 1'b0; // Default FFTStart to 0

        case (opcode)
            `OP_RTYPE: begin
                regwrite_r = 1'b1; // write ALU result to rd
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
            `OP_BRANCH: begin
                branch_r   = 1'b1; // Indicates potential branch (taken calculated later)
                alusrc_r   = 1'b0; // Uses rs1, rs2 for comparison
            end
            `OP_JAL: begin
                regwrite_r = 1'b1; // write return address to rd
                branch_r   = 1'b1; // Treat as unconditional branch for PC logic
            end
            `OP_JALR: begin
                regwrite_r = 1'b1; // write return address to rd
                branch_r   = 1'b1; // Treat as unconditional branch for PC logic
                alusrc_r   = 1'b1; // use rs1 + imm_i for target address calculation
            end
             `OP_LUI: begin // Added LUI
                 regwrite_r = 1'b1;
                 // ALUSrc might depend on ALU implementation (pass imm_u or let ALU handle) - Assume ALU handles
             end
             `OP_AUIPC: begin // Added AUIPC
                 regwrite_r = 1'b1;
                 alusrc_r = 1'b1; // ALU needs PC + imm_u
             end
            `OP_CUSTOM0: begin // === ADDED FFT CASE ===
                fftstart_r = 1'b1; // Signal the FFT interface to start
                // All other standard control signals remain 0
                regwrite_r = 1'b0;
                memread_r  = 1'b0;
                memwrite_r = 1'b0;
                memtoreg_r = 1'b0;
                alusrc_r   = 1'b0; // ALU not used directly by FFT instr
                branch_r   = 1'b0;
            end
            default: begin
                // keep defaults for unknown opcodes
            end
        endcase
    end

    assign RegWrite = regwrite_r;
    assign MemRead  = memread_r;
    assign MemWrite = memwrite_r;
    assign MemToReg = memtoreg_r;
    assign ALUSrc   = alusrc_r;
    assign Branch   = branch_r;
    assign FFTStart = fftstart_r; // Assign the new output

endmodule