`timescale 1ns / 1ps
`include "opcodes.vh"

module cpu(
    input clk,
    input reset
    );

    wire[31:0] pc;
    wire[31:0] inst1, inst2;
    wire [31:0] rs1_data1, rs2_data1, alu_B1;
    wire [31:0] rs1_data2, rs2_data2, alu_B2;
    wire [31:0] alu_Y1, mac_Y1, ex1_Y;
    wire [31:0] alu_Y2, mac_Y2, ex2_Y;
    //Control signals
    wire RegWrite1, MemRead1, MemWrite1, MemToReg1, ALUSrc1, Branch1;
    wire RegWrite2, MemRead2, MemWrite2, MemToReg2, ALUSrc2, Branch2;

    // -------------------------
    // Instruction decode
    // -------------------------
    // Instruction 1
    wire [6:0] opcode1 = inst1[6:0];
    wire [4:0] rd1 = inst1[11:7];
    wire [2:0] funct3_1 = inst1[14:12];
    wire [4:0] rs1_1 = inst1[19:15];
    wire [4:0] rs2_1 = inst1[24:20];
    wire [6:0] funct7_1 = inst1[31:25];

    // Instruction 2
    wire [6:0] opcode2 = inst2[6:0];
    wire [4:0] rd2 = inst2[11:7];
    wire [2:0] funct3_2 = inst2[14:12];
    wire [4:0] rs1_2 = inst2[19:15];
    wire [4:0] rs2_2 = inst2[24:20];
    wire [6:0] funct7_2 = inst2[31:25];

    // -------------- Immediate Gen --------------
    wire [31:0] imm_i1 = {{20{inst1[31]}}, inst1[31:20]};
    wire [31:0] imm_s1 = {{20{inst1[31]}}, inst1[31:25], inst1[11:7]};
    wire [31:0] imm_b1 = {{19{inst1[31]}}, inst1[31], inst1[7], inst1[30:25], inst1[11:8], 1'b0};
    wire [31:0] imm_j1 = {{11{inst1[31]}}, inst1[31], inst1[19:12], inst1[20], inst1[30:21], 1'b0};
    wire [31:0] signext_imm_j1 = {{11{imm_j1[20]}}, imm_j1};
    wire [31:0] jal_target1 = pc + signext_imm_j1;
    wire [31:0] jalr_target1 = (rs1_data1 + imm_i1) & ~32'd1;
    wire [31:0] branch_target1 = pc + imm_b1;

    wire [31:0] imm_i2 = {{20{inst2[31]}}, inst2[31:20]};
    wire [31:0] imm_s2 = {{20{inst2[31]}}, inst2[31:25], inst2[11:7]};
    wire [31:0] imm_b2 = {{19{inst2[31]}}, inst2[31], inst2[7], inst2[30:25], inst2[11:8], 1'b0};

    // -------------------------
    // Functional-unit usage signals & structural hazards
    // -------------------------
    // Determine whether instruction uses ALU-like ops (arithmetic/logic/branches/jalr)
    // We treat R-type, I-type ALU ops, branch, JALR/JAL as ALU users (JAL uses PC math but doesn't use ALU result)
    // Adjust the opcode checks to match your opcodes.vh definitions.
    wire uses_mem1 = (opcode1 == `OP_LOAD) || (opcode1 == `OP_STORE);
    wire uses_mem2 = (opcode2 == `OP_LOAD) || (opcode2 == `OP_STORE);

    // M-extension (MAC) detected from alu_ctrl range (10..13 used earlier)
    wire uses_mac1 = (alu_ctrl1 >= 5'd10 && alu_ctrl1 <= 5'd13);
    wire uses_mac2 = (alu_ctrl2 >= 5'd10 && alu_ctrl2 <= 5'd13);

    // ALU usage: ops that use ALU (R-type, I-type arithmetic/logical, branch target/jalr calc)
    // Adjust/add opcodes if your ISA defines more ALU-using opcodes
    wire uses_alu1 = (opcode1 == `OP_RTYPE) || (opcode1 == `OP_ITYPE) || (opcode1 == `OP_BRANCH) || (opcode1 == `OP_JALR) || (opcode1 == `OP_JAL);
    wire uses_alu2 = (opcode2 == `OP_RTYPE) || (opcode2 == `OP_ITYPE    ) || (opcode2 == `OP_BRANCH) || (opcode2 == `OP_JALR) || (opcode2 == `OP_JAL);

    // If an instruction maps to MAC (M-extension), treat it as MAC user, not ALU user:
    assign uses_alu1 = uses_alu1 & ~uses_mac1;
    assign uses_alu2 = uses_alu2 & ~uses_mac2;

    // Availability of parallel functional units in your microarchitecture:
    // - You currently have two ALU+MAC instances in the design, so ALU/MAC parallelism exists.
    // - Data memory is single-ported (we used a single Data_MEM instance). Adjust if you add a second memory port.
    localparam dual_alu_available = 1'b1;
    localparam dual_mac_available = 1'b1;
    localparam dual_mem_available = 1'b0;

    // Structural hazard detection:
    // if both instructions require the same single-ported resource and only one instance exists => structural hazard.
    wire structural_alu_hazard = (uses_alu1 && uses_alu2 && !dual_alu_available);
    wire structural_mac_hazard = (uses_mac1 && uses_mac2 && !dual_mac_available);
    wire structural_mem_hazard = (uses_mem1 && uses_mem2 && !dual_mem_available);

    wire structural_hazard = structural_alu_hazard | structural_mac_hazard | structural_mem_hazard;

    // -------------------------
    // RAW / WAW / Load-use hazard checks
    // -------------------------
    // RAW: inst2 reads a reg that inst1 will write (and inst1 actually writes)
    wire raw_hazard_rs1 = (rd1 != 5'd0) && ((rd1 == rs1_2));
    wire raw_hazard_rs2 = (rd1 != 5'd0) && ((rd1 == rs2_2));
    wire raw_hazard = (RegWrite1) && (raw_hazard_rs1 || raw_hazard_rs2);

    // WAW: both write the same destination register
    wire waw_hazard = (rd1 != 5'd0) && (rd1 == rd2) && RegWrite1 && RegWrite2;

    // Load-use: inst1 is a load and inst2 reads the loaded register — cannot forward in a single-cycle memory model
    wire load_use_hazard = (opcode1 == `OP_LOAD) && ((rd1 == rs1_2) || (rd1 == rs2_2));

    // -------------------------
    // Final hazard and issue logic
    // -------------------------
    // If any hazard exists, do not issue inst2 this cycle.
    wire any_hazard = raw_hazard | waw_hazard | load_use_hazard | structural_hazard;

    // issue_inst2 should be 1 only if no hazard and other checks (like reset) allow it.
    assign issue_inst2 = (~any_hazard) && /* you can keep reset guard if needed */ 1'b1;

    //Control signals per instruction
    control_unit u_ctrl1 (
        .opcode  (opcode1),
        .RegWrite(RegWrite1),
        .MemRead (MemRead1),
        .MemWrite(MemWrite1),
        .MemToReg(MemToReg1),
        .ALUSrc  (ALUSrc1),
        .Branch  (Branch1)   // not used yet
    );

    control_unit u_ctrl2 (
        .opcode  (opcode2),
        .RegWrite(RegWrite2),
        .MemRead (MemRead2),
        .MemWrite(MemWrite2),
        .MemToReg(MemToReg2),
        .ALUSrc  (ALUSrc2),
        .Branch  (Branch2)   // not used yet
    );

    // -------------- Register File --------------
    wire write2_blocked = issue_inst2 && (rd1 == rd2);

    register_file_dual u_regfile (
        .clk(clk),
        .reset(reset),
        .we1(RegWrite1),
        .we2(RegWrite2 & ~write2_blocked),
        .rs1_1(rs1_1),
        .rs2_1(rs2_1),
        .rs1_2(rs1_2),
        .rs2_2(rs2_2),
        .rd1(rd1),
        .rd2(rd2),
        .wdata1(ex1_Y),
        .wdata2(ex2_Y),
        .rdata1_1(rs1_data1),
        .rdata2_1(rs2_data1),
        .rdata1_2(rs1_data2),
        .rdata2_2(rs2_data2)
    );

    // -------------- ALU Control --------------
    wire [4:0] alu_ctrl1;
    alu_control u_alu_ctrl1 (
        .opcode(opcode1),
        .funct3(funct3_1),
        .funct7(funct7_1),
        .ctrl(alu_ctrl1)
    );

    wire [4:0] alu_ctrl2;
    alu_control u_alu_ctrl2 (
        .opcode(opcode2),
        .funct3(funct3_2),
        .funct7(funct7_2),
        .ctrl(alu_ctrl2)
    );

    // ---------- Simple EX->EX forwarding for inst2 ----------
    wire [31:0] rs1_val2_forwarded = (RegWrite1 && (rd1 != 5'd0) && (rd1 == rs1_2)) ? ex1_Y : rs1_data2;
    wire [31:0] rs2_val2_forwarded = (RegWrite1 && (rd1 != 5'd0) && (rd1 == rs2_2)) ? ex1_Y : rs2_data2;


    // -------------- ALU/MAC EX --------------
    assign alu_B1 = (ALUSrc1) ? ((opcode1 == `OP_STORE) ? imm_s1 : imm_i1) : rs2_data1;
    assign alu_B2 = (ALUSrc2) ? ((opcode2 == `OP_STORE) ? imm_s2 : imm_i2) : rs2_val2_forwarded;

    ALU u_alu1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(alu_Y1));
    MAC u_mac1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(mac_Y1));
    assign ex1_Y = (alu_ctrl1 >= 10 && alu_ctrl1 <= 13) ? mac_Y1 : alu_Y1;

    ALU u_alu2 (.A(rs1_val2_forwarded), .B(alu_B2), .ctrl(alu_ctrl2), .Y(alu_Y2));
    MAC u_mac2 (.A(rs1_val2_forwarded), .B(alu_B2), .ctrl(alu_ctrl2), .Y(mac_Y2));
    assign ex2_Y = (alu_ctrl2 >= 10 && alu_ctrl2 <= 13) ? mac_Y2 : alu_Y2;

    // -------------- Branch logic --------------
    wire branch_taken1;
    assign branch_taken1 = (opcode1 == `OP_BRANCH) && (
        (funct3_1 == 3'b000 && rs1_data1 == rs2_data1) || // BEQ
        (funct3_1 == 3'b001 && rs1_data1 != rs2_data1)    // BNE
    );

    // -------------------------
    // PC logic
    // -------------------------
    reg [31:0] pc_next;
    always @(*) begin
        // Default
        pc_next = pc + 4;

        if (reset)
            pc_next = 32'b0;
        else if (branch_taken1)
            pc_next = branch_target1;
        else if (opcode1 == `OP_JAL)
            pc_next = jal_target1;
        else if (opcode1 == `OP_JALR)
            pc_next = jalr_target1;
        else if (issue_inst2)
            pc_next = pc + 8;
    end

    pc u_pc(
        .clk(clk),
        .reset(reset),
        .pc_next(pc_next),
        .pc(pc)
    );

    // -------------------------
    // Instruction Memory
    // -------------------------
    Inst_MEM u_Inst_MEM1(.address(pc),    .inst(inst1));
    Inst_MEM u_Inst_MEM2(.address(pc+4), .inst(inst2));

    // -------------------------
    // Data Memory
    // -------------------------
    wire [31:0] mem_rdata;
    Data_MEM u_dmem (
        .clk       (clk),
        .write_en  (MemWrite1),
        .address   (ex1_Y),
        .write_DAT (rs2_data1),
        .read_DAT  (mem_rdata)
    );

    // -------------------------
    // Debug prints
    // -------------------------
    // small debug wires (place before the always @(posedge clk) debug print)
    wire [31:0] dbg_pc_next = pc_next;
    wire dbg_issue2 = issue_inst2;
    wire [6:0] dbg_op1 = opcode1;
    wire [6:0] dbg_op2 = opcode2;

    always @(posedge clk) begin
        //$display("PC=%0d | rd1=%0d rs1_2=%0d rs2_2=%0d forward_rs1=%b forward_rs2=%b ex1=%h ex2=%h",
        // pc, rd1, rs1_2, rs2_2, (rd1==rs1_2), (rd1==rs2_2), ex1_Y, ex2_Y);
        $display("T=%0t PC=%0d issue2=%b RAW=%b WAW=%b LOADUSE=%b STRUCT=%b",
           $time, pc, issue_inst2, raw_hazard, waw_hazard, load_use_hazard, structural_hazard);

    end

endmodule


