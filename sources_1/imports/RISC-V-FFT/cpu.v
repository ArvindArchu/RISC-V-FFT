`timescale 1ns / 1ps
`include "opcodes.vh"

module cpu(
    input clk,
    input reset
);

    // =========================================================================
    // PC and Fetch Stage
    // =========================================================================
    reg [31:0] pc_reg;
    wire [31:0] pc = pc_reg;
    
    wire [31:0] inst1_f, inst2_f;
    Inst_MEM u_Inst_MEM1(.address(pc), .inst(inst1_f));
    Inst_MEM u_Inst_MEM2(.address((pc+4)), .inst(inst2_f));
    
    reg [31:0] inst1_reg, inst2_reg;
    reg inst2_valid;  // Track if inst2_reg holds a valid stalled instruction
    
    // =========================================================================
    // Decode Stage
    // =========================================================================
    wire [6:0] opcode1 = inst1_reg[6:0];
    wire [4:0] rd1 = inst1_reg[11:7];
    wire [2:0] funct3_1 = inst1_reg[14:12];
    wire [4:0] rs1_1 = inst1_reg[19:15];
    wire [4:0] rs2_1 = inst1_reg[24:20];
    wire [6:0] funct7_1 = inst1_reg[31:25];

    wire [6:0] opcode2 = inst2_reg[6:0];
    wire [4:0] rd2 = inst2_reg[11:7];
    wire [2:0] funct3_2 = inst2_reg[14:12];
    wire [4:0] rs1_2 = inst2_reg[19:15];
    wire [4:0] rs2_2 = inst2_reg[24:20];
    wire [6:0] funct7_2 = inst2_reg[31:25];

    // Immediate generation
    wire [31:0] imm_i1 = {{20{inst1_reg[31]}}, inst1_reg[31:20]};
    wire [31:0] imm_s1 = {{20{inst1_reg[31]}}, inst1_reg[31:25], inst1_reg[11:7]};
    wire [31:0] imm_b1 = {{19{inst1_reg[31]}}, inst1_reg[31], inst1_reg[7], inst1_reg[30:25], inst1_reg[11:8], 1'b0};
    wire [31:0] imm_j1 = {{11{inst1_reg[31]}}, inst1_reg[31], inst1_reg[19:12], inst1_reg[20], inst1_reg[30:21], 1'b0};
    
    wire [31:0] imm_i2 = {{20{inst2_reg[31]}}, inst2_reg[31:20]};
    wire [31:0] imm_s2 = {{20{inst2_reg[31]}}, inst2_reg[31:25], inst2_reg[11:7]};
    wire [31:0] imm_b2 = {{19{inst2_reg[31]}}, inst2_reg[31], inst2_reg[7], inst2_reg[30:25], inst2_reg[11:8], 1'b0};

    // Branch/Jump targets
    wire [31:0] jal_target1 = pc + imm_j1;
    wire [31:0] branch_target1 = pc + imm_b1;
    
    // =========================================================================
    // Control Signals
    // =========================================================================
    wire RegWrite1, MemRead1, MemWrite1, MemToReg1, ALUSrc1, Branch1;
    wire RegWrite2, MemRead2, MemWrite2, MemToReg2, ALUSrc2, Branch2;

    control_unit u_ctrl1 (
        .opcode(opcode1),
        .RegWrite(RegWrite1),
        .MemRead(MemRead1),
        .MemWrite(MemWrite1),
        .MemToReg(MemToReg1),
        .ALUSrc(ALUSrc1),
        .Branch(Branch1)
    );

    control_unit u_ctrl2 (
        .opcode(opcode2),
        .RegWrite(RegWrite2),
        .MemRead(MemRead2),
        .MemWrite(MemWrite2),
        .MemToReg(MemToReg2),
        .ALUSrc(ALUSrc2),
        .Branch(Branch2)
    );

    wire [4:0] alu_ctrl1, alu_ctrl2;
    alu_control u_alu_ctrl1 (
        .opcode(opcode1),
        .funct3(funct3_1),
        .funct7(funct7_1),
        .ctrl(alu_ctrl1)
    );

    alu_control u_alu_ctrl2 (
        .opcode(opcode2),
        .funct3(funct3_2),
        .funct7(funct7_2),
        .ctrl(alu_ctrl2)
    );

    // =========================================================================
    // Register File
    // =========================================================================
    wire [31:0] rs1_data1, rs2_data1, rs1_data2, rs2_data2;
    
    register_file_dual u_regfile (
        .clk(clk),
        .reset(reset),
        .we1(wb_we1),
        .we2(wb_we2),
        .rs1_1(rs1_1),
        .rs2_1(rs2_1),
        .rs1_2(rs1_2),
        .rs2_2(rs2_2),
        .rd1(wb_rd1),
        .rd2(wb_rd2),
        .wdata1(wb_wdata1),
        .wdata2(wb_wdata2),
        .rdata1_1(rs1_data1),
        .rdata2_1(rs2_data1),
        .rdata1_2(rs1_data2),
        .rdata2_2(rs2_data2)
    );

    // =========================================================================
    // Hazard Detection
    // =========================================================================
    wire is_mem1 = (opcode1 == `OP_LOAD) || (opcode1 == `OP_STORE);
    wire is_mem2 = (opcode2 == `OP_LOAD) || (opcode2 == `OP_STORE);
    
    // RAW hazard: inst2 reads a register that inst1 writes
    wire raw_rs1 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs1_2);
    wire raw_rs2 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs2_2);
    wire raw_hazard = raw_rs1 || raw_rs2;
    
    // Load-use hazard: inst1 is a load and inst2 needs its result
    // We can't forward load data in the same cycle, so must stall
    wire load_use_hazard = (opcode1 == `OP_LOAD) && raw_hazard;
    
    // WAW hazard: both write to same destination
    wire waw_hazard = (rd1 != 5'd0) && (rd1 == rd2) && RegWrite1 && RegWrite2;
    
    // CRITICAL: Memory operations execute alone (no superscalar)
    wire mem_blocks_superscalar = is_mem1 || is_mem2;
    
    // Issue logic: inst2 can only issue if no hazards and no memory ops
    // RAW hazard without load is OK (we have forwarding)
    // But load-use hazard requires stall
    wire issue_inst2 = ~load_use_hazard && ~waw_hazard && ~mem_blocks_superscalar;

    // =========================================================================
    // Execute Stage
    // =========================================================================
    // EX-EX forwarding: if inst1 writes a register that inst2 reads, forward it
    wire forward_rs1_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs1_2) && !MemToReg1;
    wire forward_rs2_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs2_2) && !MemToReg1;
    
    wire [31:0] rs1_data2_fwd = forward_rs1_2 ? ex1_Y : rs1_data2;
    wire [31:0] rs2_data2_fwd = forward_rs2_2 ? ex1_Y : rs2_data2;
    
    wire [31:0] alu_B1 = ALUSrc1 ? ((opcode1 == `OP_STORE) ? imm_s1 : imm_i1) : rs2_data1;
    wire [31:0] alu_B2 = ALUSrc2 ? ((opcode2 == `OP_STORE) ? imm_s2 : imm_i2) : rs2_data2_fwd;
    
    wire [31:0] alu_Y1, alu_Y2, mac_Y1, mac_Y2;
    
    ALU u_alu1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(alu_Y1));
    MAC u_mac1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(mac_Y1));
    
    ALU u_alu2 (.A(rs1_data2_fwd), .B(alu_B2), .ctrl(alu_ctrl2), .Y(alu_Y2));
    MAC u_mac2 (.A(rs1_data2_fwd), .B(alu_B2), .ctrl(alu_ctrl2), .Y(mac_Y2));
    
    wire uses_mac1 = (alu_ctrl1 >= 5'd10 && alu_ctrl1 <= 5'd13);
    wire uses_mac2 = (alu_ctrl2 >= 5'd10 && alu_ctrl2 <= 5'd13);
    
    wire [31:0] ex1_Y = uses_mac1 ? mac_Y1 : alu_Y1;
    wire [31:0] ex2_Y = uses_mac2 ? mac_Y2 : alu_Y2;
    
    // JALR target calculation
    wire [31:0] jalr_target1 = (rs1_data1 + imm_i1) & ~32'd1;

    // =========================================================================
    // Branch Logic
    // =========================================================================
    wire branch_taken1 = (opcode1 == `OP_BRANCH) && (
        (funct3_1 == 3'b000 && rs1_data1 == rs2_data1) ||  // BEQ
        (funct3_1 == 3'b001 && rs1_data1 != rs2_data1)     // BNE
    );

    // =========================================================================
    // Memory Stage (Single-Cycle, No Pipelining)
    // =========================================================================
    wire [31:0] mem_rdata;
    
    // Memory always uses inst1 (since memory ops execute alone)
    wire [31:0] mem_addr = ex1_Y;
    wire [31:0] mem_wdata = rs2_data1;
    wire mem_we = MemWrite1 && is_mem1;
    
    Data_MEM u_dmem (
        .clk(clk),
        .reset(reset),
        .write_en(mem_we),
        .address(mem_addr),
        .write_DAT(mem_wdata),
        .read_DAT(mem_rdata)
    );

    // =========================================================================
    // Writeback Stage
    // =========================================================================
    reg wb_we1, wb_we2;
    reg [4:0] wb_rd1, wb_rd2;
    reg [31:0] wb_wdata1, wb_wdata2;
    
    // Writeback data selection
    wire [31:0] wb_data1 = MemToReg1 ? mem_rdata : ex1_Y;
    wire [31:0] wb_data2 = ex2_Y;  // inst2 never accesses memory
    
    always @(posedge clk) begin
        if (reset) begin
            wb_we1 <= 1'b0;
            wb_we2 <= 1'b0;
            wb_rd1 <= 5'd0;
            wb_rd2 <= 5'd0;
            wb_wdata1 <= 32'd0;
            wb_wdata2 <= 32'd0;
        end else begin
            // Inst1 writeback (always valid after first cycle)
            wb_we1 <= RegWrite1 && (rd1 != 5'd0) && inst2_valid;
            wb_rd1 <= rd1;
            wb_wdata1 <= wb_data1;
            
            // Inst2 writeback (only if issued and valid)
            wb_we2 <= issue_inst2 && RegWrite2 && (rd2 != 5'd0) && inst2_valid;
            wb_rd2 <= rd2;
            wb_wdata2 <= wb_data2;
        end
    end

    // =========================================================================
    // PC Update and Instruction Fetch Logic
    // =========================================================================
    reg [31:0] pc_next;
    
    always @(*) begin
        if (reset) begin
            pc_next = 32'd0;
        end else if (!inst2_valid) begin
            // Initial fetch cycle - don't advance PC yet
            pc_next = pc;
        end else if (opcode1 == `OP_JAL) begin
            pc_next = jal_target1;
        end else if (opcode1 == `OP_JALR) begin
            pc_next = jalr_target1;
        end else if (branch_taken1) begin
            pc_next = branch_target1;
        end else begin
            // Normal increment
            if (issue_inst2) begin
                pc_next = pc + 8;  // Both instructions executed
            end else begin
                pc_next = pc + 4;  // Only inst1 executed
            end
        end
    end
    
    always @(posedge clk) begin
        if (reset) begin
            pc_reg <= 32'd0;
            inst1_reg <= 32'd0;
            inst2_reg <= 32'd0;
            inst2_valid <= 1'b0;
        end else begin
            pc_reg <= pc_next;
            
            // Fetch logic with stall handling
            if (!inst2_valid) begin
                // Initial fetch after reset - load both instructions
                inst1_reg <= inst1_f;
                inst2_reg <= inst2_f;
                inst2_valid <= 1'b1;
            end else if (issue_inst2) begin
                // Both instructions issued - fetch new pair
                inst1_reg <= inst1_f;
                inst2_reg <= inst2_f;
                inst2_valid <= 1'b1;
            end else begin
                // Only inst1 issued - inst2 was stalled
                // Promote inst2 to inst1, fetch new inst2
                inst1_reg <= inst2_reg;
                inst2_reg <= inst2_f;
                inst2_valid <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Debug Output
    // =========================================================================
    always @(posedge clk) begin
        if (!reset) begin
            $display("t=%0t PC=%0d (byte_addr) inst1_fetch_addr=%0d inst2_fetch_addr=%0d", 
                     $time, pc, pc, pc+4);
            $display("  op1=%b op2=%b issue2=%b inst2_valid=%b", 
                     opcode1, opcode2, issue_inst2, inst2_valid);
            $display("  inst1_reg=%h (rd=%0d) inst2_reg=%h (rd=%0d)", 
                     inst1_reg, rd1, inst2_reg, rd2);
            
            if (MemWrite1) begin
                $display("  STORE: addr=%0d data=%h", mem_addr, mem_wdata);
            end
            
            if (MemRead1) begin
                $display("  LOAD: addr=%0d data=%h rd=%0d", mem_addr, mem_rdata, rd1);
            end
            
            if (wb_we1) begin
                $display("  WB1: rd=%0d data=%h", wb_rd1, wb_wdata1);
            end
            
            if (wb_we2) begin
                $display("  WB2: rd=%0d data=%h", wb_rd2, wb_wdata2);
            end
            $display("");
        end
    end

endmodule




/*
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

    // --- fetch latch signals ---
    wire [31:0] inst1_f, inst2_f;   // raw outputs from Inst_MEM read
    reg  [31:0] inst1_reg, inst2_reg; // latched fetched instructions
    reg         pair_valid;         // true when inst1_reg/inst2_reg are valid

 
    // -------------------------
    // PC logic
    // -------------------------
    reg fetch_stall_flag_reg; // registered flag, updates at posedge (0 = normal, 1 = second-cycle advancement)
    reg [31:0] pc_next;

    always @(*) begin
    // default: hold (safe)
        pc_next = pc;

        if (reset) begin
            pc_next = 32'd0;
        end else if (branch_taken1_prev) begin
            pc_next = branch_target1;
        end else if (opcode1 == `OP_JAL) begin
            pc_next = jal_target1;
        end else if (opcode1 == `OP_JALR) begin
            pc_next = jalr_target1;
        end else begin
            // Normal progression driven by registered fetch state (no combinational races)
            if (issue_inst2_prev && !(mem_access1 || mem_access2)) begin
                pc_next = pc + 8;    // both executed this cycle
            end else begin
                // we are in a 1-cycle stall for inst2: either hold pc (first cycle) or advance by +4 (second)
                if (fetch_stall_flag_reg)
                    pc_next = pc + 4;
                else
                    pc_next = pc;     // hold during initial stall cycle
            end
        end
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
    //Inst_MEM u_Inst_MEM1(.address(pc),    .inst(inst1_f));
    //Inst_MEM u_Inst_MEM2(.address(pc+4), .inst(inst2_f));


    // --- FETCH LATCH: hold fetched instructions for decode ---
    
    // small FSM to handle hazard stalls without losing inst2.
    // states are implicit via fetch_stall_flag:
    //  - if pair_valid==0: fill pair from inst?_f
    //  - else if pair_valid==1 and issue_inst2==1: both can issue => accept new pair
    //  - else if pair_valid==1 and issue_inst2==0:
    //      - first time: set fetch_stall_flag=1 and let inst1 execute (hold pair)
    //      - next time (fetch_stall_flag==1): shift inst2->inst1, fetch new inst2 from inst2_f, clear fetch_stall_flag.
    //
    // This gives a single-cycle stall to let inst1 complete, then promotes inst2 so progress happens.
    //reg fetch_stall_flag; // 0 = not in the middle of stall, 1 = we previously stalled and must shift now
        // Synchronous PC + fetch FSM (ordered to avoid races)
    reg [31:0] pc_reg;
    wire [31:0] pc = pc_reg;

    // Raw instruction memory outputs (combinational)
    wire [31:0] inst1_f, inst2_f;
    Inst_MEM u_Inst_MEM1(.address(pc_reg),    .inst(inst1_f));
    Inst_MEM u_Inst_MEM2(.address(pc_reg+4), .inst(inst2_f));

    // fetch FSM registers
    reg        fetch_stall_flag_reg; // indicates we are in the second cycle of a stall
    reg        pair_valid_reg;

    // compute pc_next from CURRENT registered state (inst?_reg etc).
    // Do this as local variables at start of clock cycle (snapshot).
    reg [31:0] pc_next_local;
    reg        next_fetch_stall;
    reg [31:0] next_inst1_reg;
    reg [31:0] next_inst2_reg;
    reg        next_pair_valid;

    always @(*) begin
        // default to current values (safe)
        pc_next_local = pc_reg;
        next_inst1_reg = inst1_reg;
        next_inst2_reg = inst2_reg;
        next_pair_valid = pair_valid_reg;
        next_fetch_stall = fetch_stall_flag_reg;

        // --- Decide pc_next using the *current* inst?_reg and derived signals ---
        // The signals below (opcode1, branch_taken1, issue_inst2, jal_target1, etc)
        // are wires derived from inst1_reg/inst2_reg and other current state.
        if (reset) begin
            pc_next_local = 32'd0;
        end else if (opcode1 == `OP_JAL) begin
            pc_next_local = jal_target1;
        end else if (opcode1 == `OP_JALR) begin
            pc_next_local = jalr_target1;
        end else if (branch_taken1) begin
            pc_next_local = branch_target1;
        end else begin
            if (issue_inst2) begin
                pc_next_local = pc_reg + 8;
            end else begin
                if (fetch_stall_flag_reg)
                    pc_next_local = pc_reg + 4;
                else
                    pc_next_local = pc_reg; // hold
            end
        end

        // --- Decide fetch latch next-state (still based on current pair_valid & issue_inst2) ---
        if (!pair_valid_reg) begin
            next_inst1_reg    = inst1_f;
            next_inst2_reg    = inst2_f;
            next_pair_valid   = 1'b1;
            next_fetch_stall  = 1'b0;
        end else begin
            if (issue_inst2) begin
                // both issued -> accept new fetched pair
                next_inst1_reg   = inst1_f;
                next_inst2_reg   = inst2_f;
                next_pair_valid  = 1'b1;
                next_fetch_stall = 1'b0;
            end else begin
                // stall case
                if (!fetch_stall_flag_reg) begin
                    // first stall cycle: hold
                    next_inst1_reg   = inst1_reg;
                    next_inst2_reg   = inst2_reg;
                    next_pair_valid  = 1'b1;
                    next_fetch_stall = 1'b1;
                end else begin
                    // second cycle: promote inst2 -> inst1, fetch new inst2
                    next_inst1_reg   = inst2_reg;
                    next_inst2_reg   = inst2_f;
                    next_pair_valid  = 1'b1;
                    next_fetch_stall = 1'b0;
                end
            end
        end
    end

    // Apply the clocked updates in one place (non-blocking), using the precomputed values.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_reg <= 32'd0;
            inst1_reg <= 32'd0;
            inst2_reg <= 32'd0;
            pair_valid_reg <= 1'b0;
            fetch_stall_flag_reg <= 1'b0;
        end else begin
            pc_reg <= pc_next_local;
            inst1_reg <= next_inst1_reg;
            inst2_reg <= next_inst2_reg;
            pair_valid_reg <= next_pair_valid;
            fetch_stall_flag_reg <= next_fetch_stall;
        end
    end

    // expose small helpers for debug
    wire fetch_stall_flag = fetch_stall_flag_reg;
    wire pair_valid = pair_valid_reg;
    

// These are the ones used for decode:
assign inst1 = inst1_reg;
assign inst2 = inst2_reg;

    // -------------------------
    // Instruction decode
    // -------------------------
    // Instruction 1
    wire [6:0] opcode1 = inst1_reg[6:0];
    wire [4:0] rd1 = inst1_reg[11:7];
    wire [2:0] funct3_1 = inst1_reg[14:12];
    wire [4:0] rs1_1 = inst1_reg[19:15];
    wire [4:0] rs2_1 = inst1_reg[24:20];
    wire [6:0] funct7_1 = inst1_reg[31:25];

    // Instruction 2
    wire [6:0] opcode2 = inst2_reg[6:0];
    wire [4:0] rd2 = inst2_reg[11:7];
    wire [2:0] funct3_2 = inst2_reg[14:12];
    wire [4:0] rs1_2 = inst2_reg[19:15];
    wire [4:0] rs2_2 = inst2_reg[24:20];
    wire [6:0] funct7_2 = inst2_reg[31:25];

    // -------------- Immediate Gen --------------
    wire [31:0] imm_i1 = {{20{inst1_reg[31]}}, inst1_reg[31:20]};
    wire [31:0] imm_s1 = {{20{inst1_reg[31]}}, inst1_reg[31:25], inst1_reg[11:7]};
    wire [31:0] imm_b1 = {{19{inst1_reg[31]}}, inst1_reg[31], inst1_reg[7], inst1_reg[30:25], inst1_reg[11:8], 1'b0};
    wire [31:0] imm_j1 = {{11{inst1_reg[31]}}, inst1_reg[31], inst1_reg[19:12], inst1_reg[20], inst1_reg[30:21], 1'b0};
    wire [31:0] signext_imm_j1 = {{11{imm_j1[20]}}, imm_j1};
    wire [31:0] jal_target1 = pc + signext_imm_j1;
    wire [31:0] jalr_target1 = (rs1_data1 + imm_i1) & ~32'd1;
    wire [31:0] branch_target1 = pc + imm_b1;

    wire [31:0] imm_i2 = {{20{inst2_reg[31]}}, inst2_reg[31:20]};
    wire [31:0] imm_s2 = {{20{inst2_reg[31]}}, inst2_reg[31:25], inst2_reg[11:7]};
    wire [31:0] imm_b2 = {{19{inst2_reg[31]}}, inst2_reg[31], inst2_reg[7], inst2_reg[30:25], inst2_reg[11:8], 1'b0};

     //----------------------------------------------------------------------------------------------------------------------------
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
    assign issue_inst2 = (~any_hazard) && (~mem_access1) && (~mem_access2); //wire that enables superscalar execution 
    //currently disabled for load/store operations since there is a bug with writeback
    //assign issue_inst2 = 1'b0; // enable this to disable superscalar execution

    // --- Sampled control snapshot for next-cycle PC logic ---
    reg issue_inst2_prev;
    reg branch_taken1_prev;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            issue_inst2_prev   <= 1'b0;
            branch_taken1_prev <= 1'b0;
        end else begin
            issue_inst2_prev   <= issue_inst2;
            branch_taken1_prev <= branch_taken1;
        end
    end


    //----------------------------------------------------------------------------------------------------------------------------
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
    

    // Writeback / issue gating

    // Which instructions actually issue this cycle?
    // inst1 is always issued (in this simple in-order design) when not in reset.
    // inst2 is only issued when issue_inst2 is true (and not in reset).
    wire inst1_issued = ~reset;               // or use a more precise valid-for-issue if you have one
    wire inst2_issued = (~reset) && issue_inst2;

    // Choose writeback data (load uses mem_rdata)
    wire [31:0] wb_data1 = (MemToReg1) ? mem_rdata : ex1_Y;
    wire [31:0] wb_data2 = (MemToReg2) ? mem_rdata : ex2_Y;

    
    // If both instructions write to the same destination in the same cycle, block instr2 write
    // (WAW resolution: priority to inst1)
    wire write2_blocked_waw = (rd1 != 5'd0) && (rd1 == rd2) && RegWrite1 && RegWrite2 && inst1_issued && inst2_issued;
  

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
    // Memory access selection (existing logic)
    // -------------------------
    wire mem_access1 = MemRead1 | MemWrite1;
    wire mem_access2 = MemRead2 | MemWrite2;
    wire choose_inst1_for_mem = mem_access1; // priority to inst1 when both request (structural hazard should prevent this)

    // forwarded store data for memory writes (keeps your forwarding)
    wire [31:0] rs2_val1_forwarded = (RegWrite1 && (rd1 != 5'd0) && (rd1 == rs2_1)) ? ex1_Y : rs2_data1;
    wire [31:0] rs2_val2_forwarded = (RegWrite1 && (rd1 != 5'd0) && (rd1 == rs2_2)) ? ex1_Y : rs2_data2;
    wire [31:0] rs1_val1_forwarded = (RegWrite2 && (rd2 != 5'd0) && (rd2 == rs1_1)) ? ex2_Y : rs1_data1;

    wire [31:0] dmem_addr  = choose_inst1_for_mem ? ex1_Y : ex2_Y;
    wire [31:0] dmem_wdata = choose_inst1_for_mem ? rs2_val1_forwarded : rs2_val2_forwarded;
    wire        dmem_we    = choose_inst1_for_mem ? MemWrite1 : MemWrite2;

    // instantiate Data_MEM (keep your instrumented DMEM for debugging or the final DMEM module)
    //Data_MEM u_dmem (
    //    .clk       (clk),
    //    .reset     (reset),
    //    .write_en  (dmem_we),
    //    .address   (dmem_addr),
    //    .write_DAT (dmem_wdata),
    //    .read_DAT  (mem_rdata)
    //);
    // -------------------------
    // MEM -> WB response pipeline (1-cycle)
    // - When the load is selected (mem_access and MemRead), capture mem_rdata and the destination rd.
    // - On next clock we will use mem_resp_* to drive register writes for the load.
    // This avoids races where mem_rdata becomes visible *after* the register file sampled it.
    // -------------------------

    // --- mem read pending latch ---
    reg        mem_resp_valid;
    reg [4:0]  mem_resp_rd;
    reg [31:0] mem_resp_data;

    reg        load_req;         // set when a load is issued (cycle N)
    reg        load_pending;     // stage1 (cycle N+1)
    reg        load_pending2;    // stage2 (cycle N+2)
    reg  [4:0] load_rd_stage1;
    reg  [4:0] load_rd_stage2;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            load_req       <= 1'b0;
            load_pending   <= 1'b0;
            load_pending2  <= 1'b0;
            load_rd_stage1 <= 5'd0;
            load_rd_stage2 <= 5'd0;
        end else begin
            // Stage the load request on issue (cycle N)
            if (choose_inst1_for_mem && MemRead1) begin
                load_req       <= 1'b1;
                load_rd_stage1 <= rd1;
            end else if ((!choose_inst1_for_mem) && MemRead2) begin
                load_req       <= 1'b1;
                load_rd_stage1 <= rd2;
            end else begin
                load_req <= 1'b0;
            end

            // Progress pipeline: stage1 <- load_req, stage2 <- stage1
            load_pending  <= load_req;
            load_pending2 <= load_pending;

            // pipeline rd as well
            load_rd_stage2 <= load_rd_stage1;
        end
    end

    // Now capture mem_rdata when load_pending2==1 (cycle N+2),
    // because Data_MEM's read_DAT is valid one cycle after the address was applied.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_resp_valid <= 1'b0;
            mem_resp_rd    <= 5'd0;
            mem_resp_data  <= 32'd0;
        end else begin
            if (load_pending2) begin
                mem_resp_valid <= 1'b1;
                mem_resp_rd    <= load_rd_stage2;
                mem_resp_data  <= mem_rdata; // mem_rdata now stable
            end else begin
                mem_resp_valid <= 1'b0;
                mem_resp_rd    <= 5'd0;
                mem_resp_data  <= 32'd0;
            end
        end
    end

    // -------------------------
    // WB selection and register-file interface
    // - For ALU / MAC / non-load writes, we keep your original reg_we and wb_data immediate write behavior.
    // - For loads, we suppress the immediate write and instead use mem_resp_* the following cycle.
    // -------------------------

    // original immediate reg write enables from control + issue gating (inst issued earlier)
    wire reg_we1_immediate = RegWrite1 & inst1_issued; // stays as before for non-loads
    wire reg_we2_immediate = RegWrite2 & inst2_issued;

    // If instruction is a load, do not perform immediate write (we'll handle via mem_resp)
    wire reg_we1_final = reg_we1_immediate & ~MemToReg1;
    wire reg_we2_final = reg_we2_immediate & ~MemToReg2;

    // Prepare immediate WB data for non-loads (ALU/MAC)
    wire [31:0] wb_data1_imm = ex1_Y;
    wire [31:0] wb_data2_imm = ex2_Y;

    // Now create the "writeback stage" registers that drive the register file's two write ports.
    // We'll use these to write either immediate ALU results (same-cycle) or load responses (from mem_resp next cycle).
    reg        wb_we1_reg;
    reg        wb_we2_reg;
    reg  [4:0] wb_rd1_reg;
    reg  [4:0] wb_rd2_reg;
    reg [31:0] wb_wdata1_reg;
    reg [31:0] wb_wdata2_reg;

    // On each clock, build the WB outputs. Priority: if mem_resp_valid, it will use one write port
    // (we choose port1 for mem_resp if available; port arbitration is simple here).
    always @(posedge clk) begin
        if (reset) begin
            wb_we1_reg   <= 1'b0;
            wb_we2_reg   <= 1'b0;
            wb_rd1_reg   <= 5'd0;
            wb_rd2_reg   <= 5'd0;
            wb_wdata1_reg<= 32'd0;
            wb_wdata2_reg<= 32'd0;
        end else begin
            // Default: clear
            wb_we1_reg   <= 1'b0;
            wb_we2_reg   <= 1'b0;
            wb_rd1_reg   <= 5'd0;
            wb_rd2_reg   <= 5'd0;
            wb_wdata1_reg<= 32'd0;
            wb_wdata2_reg<= 32'd0;

            // 1) If we have a memory response from previous cycle, commit it now.
            if (mem_resp_valid) begin
                // Write the mem response into port1 (try to prefer port1)
                // If mem_resp_rd == 0 (x0) it will be ignored inside regfile
                wb_we1_reg    <= 1'b1;
                wb_rd1_reg    <= mem_resp_rd;
                wb_wdata1_reg <= mem_resp_data;

                // Also, allow an immediate ALU result to use port2 this same cycle if present
                if (reg_we1_final) begin
                    // ALU result for inst1 — but port1 already used by mem_resp, try using port2.
                    wb_we2_reg    <= reg_we1_final;
                    wb_rd2_reg    <= rd1;
                    wb_wdata2_reg <= wb_data1_imm;
                end else if (reg_we2_final) begin
                    // inst2 ALU result can use port2
                    wb_we2_reg    <= reg_we2_final;
                    wb_rd2_reg    <= rd2;
                    wb_wdata2_reg <= wb_data2_imm;
                end
            end else begin
                // No memory response pending — commit immediate ALU/MAC writes for inst1 and inst2 (if any)
                // Port1 <- inst1, Port2 <- inst2
                wb_we1_reg    <= reg_we1_final;
                wb_rd1_reg    <= rd1;
                wb_wdata1_reg <= wb_data1_imm;

                wb_we2_reg    <= reg_we2_final;
                wb_rd2_reg    <= rd2;
                wb_wdata2_reg <= wb_data2_imm;
            end
        end
    end

    register_file_dual u_regfile (
        .clk(clk),
        .reset(reset),
        .we1(wb_we1_reg),
        .we2(wb_we2_reg),
        .rs1_1(rs1_1),
        .rs2_1(rs2_1),
        .rs1_2(rs1_2),
        .rs2_2(rs2_2),
        .rd1(wb_rd1_reg),
        .rd2(wb_rd2_reg),
        .wdata1(wb_wdata1_reg),
        .wdata2(wb_wdata2_reg),
        .rdata1_1(rs1_data1),
        .rdata2_1(rs2_data1),
        .rdata1_2(rs1_data2),
        .rdata2_2(rs2_data2)
    );

    

    // -------------------------
    // Debug prints
    // -------------------------
    wire [31:0] dbg_pc_next = pc_next;
    wire dbg_issue2 = issue_inst2;
    wire [6:0] dbg_op1 = opcode1;
    wire [6:0] dbg_op2 = opcode2;

    always @(posedge clk) begin
        // Whenever a memory access is issued, print detailed context
        if (choose_inst1_for_mem && (MemRead1 || MemWrite1)) begin
            $display("CPU_MEM1_ISSUE t=%0t pc=%0d op=%b MemRead1=%b MemWrite1=%b rd1=%0d rs1_1=%0d rs2_1=%0d imm_i1=%0d imm_s1=%0d ex1_Y=%0d dmem_idx=%0d dmem_wdata=%h",
                    $time, pc_reg, opcode1, MemRead1, MemWrite1, rd1, rs1_1, rs2_1, imm_i1, imm_s1, ex1_Y, ex1_Y[9:2], rs2_val1_forwarded);
        end
        if ((!choose_inst1_for_mem) && (MemRead2 || MemWrite2)) begin
            $display("CPU_MEM2_ISSUE t=%0t pc=%0d op=%b MemRead2=%b MemWrite2=%b rd2=%0d rs1_2=%0d rs2_2=%0d imm_i2=%0d imm_s2=%0d ex2_Y=%0d dmem_idx=%0d dmem_wdata=%h",
                    $time, pc_reg, opcode2, MemRead2, MemWrite2, rd2, rs1_2, rs2_2, imm_i2, imm_s2, ex2_Y, ex2_Y[9:2], rs2_val2_forwarded);
        end

    end 

endmodule

/*
    reg        mem_resp_valid;
    reg  [4:0] mem_resp_rd;
    reg [31:0] mem_resp_data;

    // --- Added: capture load address for delayed read ---
    reg [31:0] pending_load_addr;
    reg [4:0]  pending_load_rd;
    reg        pending_load_active;

    // --- Modified: delayed memory response (data captured one cycle after address) ---
    always @(posedge clk) begin
    if (reset) begin
        pending_load_addr   <= 32'd0;
        pending_load_rd     <= 5'd0;
        pending_load_active <= 1'b0;
    end else begin
        // default: clear flag unless new load issued
        pending_load_active <= 1'b0;

        // when a load is issued (choose_inst1_for_mem ensures single port)
        if (choose_inst1_for_mem && MemRead1) begin
            pending_load_addr   <= dmem_addr;
            pending_load_rd     <= rd1;
            pending_load_active <= 1'b1;
        end else if ((!choose_inst1_for_mem) && MemRead2) begin
            pending_load_addr   <= dmem_addr;
            pending_load_rd     <= rd2;
            pending_load_active <= 1'b1;
        end
    end
end

    // Capture response on cycle where memory was accessed for read (choose_inst1_for_mem selects which one)
    // We capture the current mem_rdata (combinational) into a register at posedge.
    // --- Modified: delayed memory response (data captured one cycle after address) ---
    always @(posedge clk) begin
        if (reset) begin
            mem_resp_valid <= 1'b0;
            mem_resp_rd    <= 5'd0;
            mem_resp_data  <= 32'd0;
        end else begin
            // Default clear
            mem_resp_valid <= 1'b0; 
            mem_resp_rd    <= 5'd0;
            mem_resp_data  <= 32'd0;

            // One cycle after the load was issued, capture the stable data
            if (pending_load_active) begin
                mem_resp_valid <= 1'b1;
                mem_resp_rd    <= pending_load_rd;
                mem_resp_data  <= u_dmem.mem[pending_load_addr[7:0]]; 
                // ^ direct access to internal array (works in sim)
                // or mem_rdata if you make Data_MEM synchronous
            end
        end
    end







   // -------------------------
    // Simulation-only data memory (simple, reliable)
    // -------------------------
    reg [31:0] sim_mem [0:255]; // 256 words
    integer sm_i;
    wire [7:0] sim_mem_index = dmem_addr[9:2]; // use byte-indexing for now (matches your working case)

    // combinational read from sim_mem (instant)
    wire [31:0] sim_mem_rdata;
    assign sim_mem_rdata = sim_mem[sim_mem_index];

    // synchronous write into sim_mem (updates at posedge)
    always @(posedge clk) begin
        if (reset) begin
            for (sm_i = 0; sm_i < 256; sm_i = sm_i + 1) sim_mem[sm_i] <= 32'd0;
        end else begin
            if (dmem_we) begin
                sim_mem[sim_mem_index] <= dmem_wdata;
                $display("SIM_MEM_WRITE t=%0t idx=%0d addr=%0d data=%h", $time, sim_mem_index, dmem_addr, dmem_wdata);
            end
        end
    end
    wire [31:0] mem_rdata;
    assign mem_rdata = sim_mem[dmem_addr[7:0]];

*/