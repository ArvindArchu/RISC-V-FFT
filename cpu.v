`timescale 1ns / 1ps
`include "opcodes.vh"


module cpu(
    input clk,
    input reset
    );

    wire[31:0] pc;
    wire[31:0] inst;

    pc u_pc(
        .clk(clk),
        .reset(reset),
        .pc(pc)
    );
    Inst_MEM u_Inst_MEM(
        .address(pc),
        .inst(inst)
    );

    //------------Instruction decode--------------
    wire [6:0] opcode = inst[6:0];
    wire [4:0] rd = inst[11:7];
    wire [2:0] funct3 = inst[14:12];
    wire [4:0] rs1 = inst[19:15];
    wire [4:0] rs2 = inst[24:20];
    wire [6:0] funct7 = inst[31:25];
    

    //Control signals
    wire RegWrite, MemRead, MemWrite, MemToReg, ALUSrc, Branch;
    control_unit u_ctrl (
        .opcode  (opcode),
        .RegWrite(RegWrite),
        .MemRead (MemRead),
        .MemWrite(MemWrite),
        .MemToReg(MemToReg),
        .ALUSrc  (ALUSrc),
        .Branch  (Branch)   // not used yet
    );

    // Register file (32x32): must expose rs1/rs2 read, rd write
    wire [31:0] rs1_data, rs2_data;
    wire [31:0] wb_data = (MemToReg) ? mem_rdata : ex_Y;
    wire [31:0] wdata = wb_data;
    register_file u_regfile (
        .reset  (reset),
        .clk    (clk),
        .we     (RegWrite),
        .rs1    (rs1),
        .rs2    (rs2),
        .rd     (rd),
        .wdata  (wdata),
        .rdata1 (rs1_data),
        .rdata2 (rs2_data)
    );
    // -------------- Immediate Gen --------------
    // I-type: imm[31:0] = sign-extend inst[31:20]
    wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
    // S-type: imm[31:0] = sign-extend {inst[31:25], inst[11:7]}
    wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};
    // (B/J/U types can be added later)

    // -------------- EX (ALU/MAC) --------------
    //wire [31:0] alu_B = (ALUSrc) ? ((opcode == 7'b0100011) ? imm_s : imm_i) : rs2_data;
    wire [31:0] alu_B = (ALUSrc) ? ((opcode == `OP_STORE) ? imm_s : imm_i) : rs2_data;


    // ALU control (you already have this module):
    // maps (opcode, funct3, funct7) -> 5-bit alu_ctrl
    wire [4:0] alu_ctrl;
    alu_control u_alu_ctrl (  // if your file is named alu_control.v, use that module name
        .opcode (opcode),
        .funct3 (funct3),
        .funct7 (funct7),
        .ctrl   (alu_ctrl)
    );

    // Execute in ALU and MAC
    wire [31:0] alu_Y, mac_Y, ex_Y;
    ALU u_alu (
        .A    (rs1_data),
        .B    (alu_B),
        .ctrl (alu_ctrl),
        .Y    (alu_Y)
    );

    MAC u_mac (
        .A    (rs1_data),
        .B    (alu_B),
        .ctrl (alu_ctrl),
        .Y    (mac_Y)
    );

    // Select ALU vs MAC by ctrl range (10..13 => MAC)
    assign ex_Y = (alu_ctrl >= 5'd10 && alu_ctrl <= 5'd13) ? mac_Y : alu_Y;

    // -------------- MEM --------------
    // Address comes from ALU result (rs1 + imm)
    wire [31:0] mem_rdata;
    Data_MEM u_dmem (
        .clk       (clk),
        .write_en  (MemWrite),
        .address   (ex_Y),
        .write_DAT (rs2_data),
        .read_DAT  (mem_rdata)
    );

    // -------------- WB --------------
    //wire [31:0] wb_data = (MemToReg) ? mem_rdata : ex_Y;

    // Connect WB to regfile
    // (We can’t loop a wire back into u_regfile’s port inline,
    //  so expose a wire and connect)
    //wire [31:0] wdata = wb_data;
    // Some tools require explicit continuous assignment:
    // assign wdata = wb_data;

    // If your reg_file ports require wdata at instantiation time,
    // change u_regfile above to .wdata(wdata)
    always @(posedge clk) begin
    $display("PC=%0d instr=%h rd=%d rs1=%d rs2=%d RegWrite=%b ALUSrc=%b", 
             uut.u_pc.pc, uut.u_Inst_MEM.inst, rd, rs1, rs2, RegWrite, ALUSrc);
    $display("rs1_data=%h rs2_data=%h alu_B=%h ex_Y=%h", rs1_data, rs2_data, alu_B, ex_Y);
    end
endmodule
