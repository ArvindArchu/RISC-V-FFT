// fft works , basic sueprscalar works but breaks even for multiple add instructions, brancha and jump completely broken
`timescale 1ns / 1ps
`include "opcodes.vh"

module cpu #(
    parameter CPU_DATA_W = 32,  // CPU data width
    parameter ADDR_W = 32       // Address width
) (
    input wire clk,
    input wire reset
);

    // =========================================================================
    // PC Stage, Fetch Stage, Decode Stage
    // =========================================================================
    reg [ADDR_W-1:0] pc_reg;
    reg [ADDR_W-1:0] pc_next;
    wire [ADDR_W-1:0] pc = pc_reg;
    wire [ADDR_W-1:0] pc_plus_4 = pc + 4;

    wire [CPU_DATA_W-1:0] inst1_f, inst2_f;
    Inst_MEM u_Inst_MEM1(.address(pc), .inst(inst1_f));
    Inst_MEM u_Inst_MEM2(.address(pc_plus_4), .inst(inst2_f));

    reg [ADDR_W-1:0] fd_pc;
    reg [CPU_DATA_W-1:0] fd_inst1, fd_inst2;
    reg fd_valid;

    wire [6:0] opcode1 = fd_inst1[6:0];
    wire [4:0] rd1 = fd_inst1[11:7];
    wire [2:0] funct3_1 = fd_inst1[14:12];
    wire [4:0] rs1_1 = fd_inst1[19:15];
    wire [4:0] rs2_1 = fd_inst1[24:20];
    wire [6:0] funct7_1 = fd_inst1[31:25];

    wire [6:0] opcode2 = fd_inst2[6:0];
    wire [4:0] rd2 = fd_inst2[11:7];
    wire [2:0] funct3_2 = fd_inst2[14:12];
    wire [4:0] rs1_2 = fd_inst2[19:15];
    wire [4:0] rs2_2 = fd_inst2[24:20];
    wire [6:0] funct7_2 = fd_inst2[31:25];

    // Immediate generation
    wire [CPU_DATA_W-1:0] imm_i1 = {{20{fd_inst1[31]}}, fd_inst1[31:20]};
    wire [CPU_DATA_W-1:0] imm_i2 = {{20{fd_inst2[31]}}, fd_inst2[31:20]};
    wire [CPU_DATA_W-1:0] imm_s1 = {{20{fd_inst1[31]}}, fd_inst1[31:25], fd_inst1[11:7]};
    wire [CPU_DATA_W-1:0] imm_s2 = {{20{fd_inst2[31]}}, fd_inst2[31:25], fd_inst2[11:7]};
    wire [CPU_DATA_W-1:0] imm_b1 = {{19{fd_inst1[31]}}, fd_inst1[31], fd_inst1[7], fd_inst1[30:25], fd_inst1[11:8], 1'b0};
    wire [CPU_DATA_W-1:0] imm_b2 = {{19{fd_inst2[31]}}, fd_inst2[31], fd_inst2[7], fd_inst2[30:25], fd_inst2[11:8], 1'b0};
    wire [CPU_DATA_W-1:0] imm_j1 = {{11{fd_inst1[31]}}, fd_inst1[31], fd_inst1[19:12], fd_inst1[20], fd_inst1[30:21], 1'b0};
    wire [CPU_DATA_W-1:0] imm_j2 = {{11{fd_inst2[31]}}, fd_inst2[31], fd_inst2[19:12], fd_inst2[20], fd_inst2[30:21], 1'b0};

    // Branch/Jump targets
    wire [ADDR_W-1:0] pc_inst1 = fd_pc;
    wire [ADDR_W-1:0] pc_inst2 = fd_pc + 4;
    wire [ADDR_W-1:0] jal_target1 = pc_inst1 + imm_j1;
    wire [ADDR_W-1:0] jal_target2 = pc_inst2 + imm_j2;
    wire [ADDR_W-1:0] branch_target1 = pc_inst1 + imm_b1;

    // =========================================================================
    // Control Signal Generation
    // =========================================================================
    wire RegWrite1, MemRead1, MemWrite1, MemToReg1, ALUSrc1, Branch1;
    wire RegWrite2, MemRead2, MemWrite2, MemToReg2, ALUSrc2, Branch2;

    //assign ALUSrc1   = (opcode1 == `OP_LOAD) || (opcode1 == `OP_STORE) || (opcode1 == `OP_ITYPE) || (opcode1 == `OP_JALR);
    //assign MemToReg1 = (opcode1 == `OP_LOAD);
    //assign RegWrite1 = (opcode1 == `OP_RTYPE) || (opcode1 == `OP_ITYPE) || (opcode1 == `OP_LOAD) ||
    //                   (opcode1 == `OP_JAL) || (opcode1 == `OP_JALR) || (opcode1 == `OP_LUI) || (opcode1 == `OP_AUIPC);
    //assign MemRead1  = (opcode1 == `OP_LOAD);
    //assign MemWrite1 = (opcode1 == `OP_STORE);
    //assign Branch1   = (opcode1 == `OP_BRANCH);

    //assign ALUSrc2   = (opcode2 == `OP_LOAD) || (opcode2 == `OP_STORE) || (opcode2 == `OP_ITYPE) || (opcode2 == `OP_JALR);
    //assign MemToReg2 = (opcode2 == `OP_LOAD);
    //assign RegWrite2 = (opcode2 == `OP_RTYPE) || (opcode2 == `OP_ITYPE) || (opcode2 == `OP_LOAD) ||
    //                   (opcode2 == `OP_JAL) || (opcode2 == `OP_JALR) || (opcode2 == `OP_LUI) || (opcode2 == `OP_AUIPC);
    //assign MemRead2  = (opcode2 == `OP_LOAD);
    //assign MemWrite2 = (opcode2 == `OP_STORE);
    //assign Branch2   = (opcode2 == `OP_BRANCH);

        control_unit u_ctrl1 (
        .opcode(opcode1),
        .RegWrite(RegWrite1),
        .MemRead(MemRead1),
        .MemWrite(MemWrite1),
        .MemToReg(MemToReg1),
        .ALUSrc(ALUSrc1),
        .Branch(Branch1),
        .FFTStart(FFTStart1)
    );

    control_unit u_ctrl2 (
        .opcode(opcode2),
        .RegWrite(RegWrite2),
        .MemRead(MemRead2),
        .MemWrite(MemWrite2),
        .MemToReg(MemToReg2),
        .ALUSrc(ALUSrc2),
        .Branch(Branch2),
        .FFTStart(FFTStart2)
    );

    wire [4:0] alu_ctrl1, alu_ctrl2;
    alu_control u_alu_ctrl1 (.opcode(opcode1), .funct3(funct3_1), .funct7(funct7_1), .ctrl(alu_ctrl1));
    alu_control u_alu_ctrl2 (.opcode(opcode2), .funct3(funct3_2), .funct7(funct7_2), .ctrl(alu_ctrl2));

    // =========================================================================
    // Register File
    // =========================================================================
    wire [CPU_DATA_W-1:0] rs1_data1, rs2_data1, rs1_data2, rs2_data2;
    reg wb_we1_reg, wb_we2_reg;
    reg [4:0] wb_rd1_reg, wb_rd2_reg;
    reg [CPU_DATA_W-1:0] wb_wdata1_reg, wb_wdata2_reg;

    register_file_dual u_regfile (
        .clk(clk), .reset(reset), 
        .we1(wb_we1_reg), .we2(wb_we2_reg),
        .rs1_1(rs1_1), .rs2_1(rs2_1), .rs1_2(rs1_2), .rs2_2(rs2_2),
        .rd1(wb_rd1_reg), .rd2(wb_rd2_reg), 
        .wdata1(wb_wdata1_reg), .wdata2(wb_wdata2_reg),
        .rdata1_1(rs1_data1), .rdata2_1(rs2_data1), 
        .rdata1_2(rs1_data2), .rdata2_2(rs2_data2)
    );

    // =========================================================================
    // Execute Stage
    // =========================================================================
    wire [CPU_DATA_W-1:0] alu_Y1, alu_Y2;
    wire [CPU_DATA_W-1:0] ex1_Y = alu_Y1;
    wire [CPU_DATA_W-1:0] ex2_Y = alu_Y2;

    wire forward_rs1_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs1_2) && !MemToReg1;
    wire forward_rs2_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs2_2) && !MemToReg1;
    
    wire [CPU_DATA_W-1:0] rs1_data2_fwd = forward_rs1_2 ? ex1_Y : rs1_data2;
    wire [CPU_DATA_W-1:0] rs2_data2_fwd = forward_rs2_2 ? ex1_Y : rs2_data2;

    wire [CPU_DATA_W-1:0] alu_B1 = ALUSrc1 ? ((opcode1 == `OP_STORE) ? imm_s1 : imm_i1) : rs2_data1;
    wire [CPU_DATA_W-1:0] alu_B2 = ALUSrc2 ? ((opcode2 == `OP_STORE) ? imm_s2 : imm_i2) : rs2_data2_fwd;

    ALU u_alu1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(alu_Y1));
    ALU u_alu2 (.A(rs1_data2_fwd), .B(alu_B2), .ctrl(alu_ctrl2), .Y(alu_Y2));

    // JAL/JALR & Branch
    wire is_jal1 = (opcode1 == `OP_JAL);
    wire is_jalr1 = (opcode1 == `OP_JALR);
    wire is_jal2 = (opcode2 == `OP_JAL);
    wire is_jalr2 = (opcode2 == `OP_JALR);
    wire [ADDR_W-1:0] link_addr1 = pc_inst1 + 4;
    wire [ADDR_W-1:0] link_addr2 = pc_inst2 + 4;
    wire [ADDR_W-1:0] jalr_target1 = (rs1_data1 + imm_i1) & ~32'd1;
    wire branch_taken1 = (opcode1 == `OP_BRANCH) && 
                        ((funct3_1 == 3'b000 && rs1_data1 == rs2_data1) || 
                         (funct3_1 == 3'b001 && rs1_data1 != rs2_data1));

    // =========================================================================
    // Hazard Detection (WITHOUT FFT)
    // =========================================================================
    reg load_in_ex, load_in_mem;
    reg [4:0] load_dest_ex, load_dest_mem;

    wire control_flow_taken = (is_jal1 || is_jalr1 || branch_taken1) && fd_valid;

    wire stall_due_to_load_in_ex = load_in_ex && (load_dest_ex != 5'd0) && fd_valid &&
                                  ((load_dest_ex == rs1_1 && rs1_1 != 5'd0) || 
                                   (load_dest_ex == rs2_1 && rs2_1 != 5'd0));
    wire stall_due_to_load_in_mem = load_in_mem && (load_dest_mem != 5'd0) && fd_valid &&
                                   ((load_dest_mem == rs1_1 && rs1_1 != 5'd0) || 
                                    (load_dest_mem == rs2_1 && rs2_1 != 5'd0));


    // ============================================================
    //                    FFT ACCELERATOR SECTION
    // ============================================================

    localparam FFT_DATA_W = 16;

    // --- Wires from control units ---
    wire FFTStart1, FFTStart2;  // From u_ctrl1 and u_ctrl2

    // --- Internal FFT control signals ---
    reg  fft_start;   // One-cycle pulse to fft8_core
    reg  fft_busy;    // CPU stall flag during FFT operation

    wire fft_done;    // Done flag from FFT core

    // --- FFT Data (hardcoded, same as testbench) ---
    wire signed [FFT_DATA_W*8-1:0] fft_in_re_flat = {
        //impulse
        //16'sd0, 16'sd0, 16'sd0, 16'sd0,
        //16'sd0, 16'sd0, 16'sd0, 16'sd32767
        //constant
        //16'sd32767, 16'sd32767, 16'sd32767, 16'sd32767,
        //16'sd32767, 16'sd32767, 16'sd32767, 16'sd32767
        //cosine
        16'sd32767, 16'sd23169, 16'sd0, -16'sd23169,
        -16'sd32767, -16'sd23169, 16'sd0, 16'sd23169
        //unit step
        //16'sd32767, 16'sd32767, 16'sd32767, 16'sd32767,
        //16'sd32767, 16'sd32767, 16'sd32767, 16'sd32767
    };
    wire signed [FFT_DATA_W*8-1:0] fft_in_im_flat = 128'd0;

    wire signed [FFT_DATA_W*8-1:0] fft_out_re_flat;
    wire signed [FFT_DATA_W*8-1:0] fft_out_im_flat;

    // --- FFT Core Instance ---
    fft8_core u_fft8 (
        .clk(clk),
        .reset(reset),
        .start(fft_start),
        .done(fft_done),
        .in_re_flat(fft_in_re_flat),
        .in_im_flat(fft_in_im_flat),
        .out_re_flat(fft_out_re_flat),
        .out_im_flat(fft_out_im_flat)
    );

    // ============================================================
    //               FFT CONTROL AND STALL LOGIC
    // ============================================================
    always @(posedge clk) begin
        if (reset) begin
            fft_start <= 1'b0;
            fft_busy  <= 1'b0;
        end else begin
            // Start FFT when custom instruction is issued
            if (!fft_busy && (FFTStart1 || FFTStart2)) begin
                fft_start <= 1'b1;  // Pulse start for 1 cycle
                fft_busy  <= 1'b1;  // Stall CPU
                $display("[%0t] FFT instruction detected — FFT started", $time);
            end else begin
                fft_start <= 1'b0;  // Clear after 1 cycle
            end

            // Release stall when FFT is done
            if (fft_busy && fft_done) begin
                fft_busy <= 1'b0;
                $display("[%0t] FFT done — releasing CPU stall", $time);
            end
        end
    end

    // ============================================================
    //               PC STALL INTEGRATION (important!)
    // ============================================================
    // Whenever PC updates or pipeline advances, gate with !fft_busy


    wire pipeline_stall = stall_due_to_load_in_ex || stall_due_to_load_in_mem || fft_busy;

    always @(posedge clk) begin
        if (reset || control_flow_taken) begin
            load_in_ex <= 1'b0;
            load_in_mem <= 1'b0;
            load_dest_ex <= 5'd0;
            load_dest_mem <= 5'd0;
        end else if (!pipeline_stall) begin
            load_in_mem <= load_in_ex;
            load_dest_mem <= load_dest_ex;
            
            if (fd_valid && (opcode1 == `OP_LOAD) && (rd1 != 5'd0)) begin
                load_in_ex <= 1'b1;
                load_dest_ex <= rd1;
            end else begin
                load_in_ex <= 1'b0;
                load_dest_ex <= 5'd0;
            end
        end
    end

    wire raw_rs1 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs1_2) && !(opcode1 == `OP_LOAD);
    wire raw_rs2 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs2_2) && !(opcode1 == `OP_LOAD);
    wire raw_hazard = raw_rs1 || raw_rs2;
    wire load_use_hazard_inst1_inst2 = fd_valid && (opcode1 == `OP_LOAD) && (rd1 != 5'd0) && 
                                      ((rd1 == rs1_2 && rs1_2 != 5'd0) || (rd1 == rs2_2 && rs2_2 != 5'd0));
    wire waw_hazard = (rd1 != 5'd0) && (rd1 == rd2) && RegWrite1 && RegWrite2;

    wire is_mem1 = (opcode1 == `OP_LOAD) || (opcode1 == `OP_STORE);
    wire is_mem2 = (opcode2 == `OP_LOAD) || (opcode2 == `OP_STORE);
    wire is_control_flow1 = is_jal1 || is_jalr1 || (opcode1 == `OP_BRANCH);
    wire is_control_flow2 = is_jal2 || is_jalr2 || (opcode2 == `OP_BRANCH);
    wire blocks_superscalar = is_mem1 || is_mem2 || is_control_flow1 || is_control_flow2 || load_use_hazard_inst1_inst2;
    wire issue_inst2 = ~raw_hazard && ~waw_hazard && ~blocks_superscalar && fd_valid && !pipeline_stall && !fft_busy;

    // =========================================================================
    // Instruction Name Decode
    // =========================================================================
    reg [8*8-1:0] inst1_name, inst2_name;
    (* keep = "true" *) reg [8*8-1:0] inst1_name_reg, inst2_name_reg;
    
    always @(*) begin
        inst1_name = "DECODE1?";
        inst2_name = "DECODE2?";
        
        if (!fd_valid) 
            inst1_name = "INVALID";
        else if (fd_inst1 == 32'h00000013) 
            inst1_name = "NOP";
        else begin
            case (opcode1)
                `OP_LOAD: inst1_name = "LOAD";
                `OP_STORE: inst1_name = "STORE";
                `OP_BRANCH: inst1_name = (funct3_1 == 3'b000) ? "BEQ" : "BNE";
                `OP_JAL: inst1_name = "JAL";
                `OP_JALR: inst1_name = "JALR";
                `OP_ITYPE: inst1_name = "ADDI";
                `OP_RTYPE: inst1_name = (funct7_1 == 7'b0100000) ? "SUB" : "ADD";
                `OP_LUI: inst1_name = "LUI";
                `OP_AUIPC: inst1_name = "AUIPC";
                `OP_CUSTOM0 :inst1_name = "FFT";
                default: inst1_name = "???";
            endcase
        end
        
        if (!fd_valid) 
            inst2_name = "INVALID";
        else if (pipeline_stall || !issue_inst2) 
            inst2_name = "STALLED";
        else if (fd_inst2 == 32'h00000013) 
            inst2_name = "NOP";
        else begin
            case (opcode2)
                `OP_LOAD: inst2_name = "LOAD";
                `OP_STORE: inst2_name = "STORE";
                `OP_BRANCH: inst2_name = "BRANCH";
                `OP_JAL: inst2_name = "JAL";
                `OP_JALR: inst2_name = "JALR";
                `OP_ITYPE: inst2_name = "ADDI";
                `OP_RTYPE: inst2_name = "ADD";
                `OP_LUI: inst2_name = "LUI";
                `OP_AUIPC: inst2_name = "AUIPC";
                `OP_CUSTOM0 :inst2_name = "FFT";
                default: inst2_name = "???";
            endcase
        end
    end
    
    always @(posedge clk) begin
        if (reset) begin
            inst1_name_reg <= "RESET   ";
            inst2_name_reg <= "RESET   ";
        end else begin
            inst1_name_reg <= inst1_name;
            inst2_name_reg <= inst2_name;
        end
    end

    // =========================================================================
    // Debug Registers
    // =========================================================================
    (* keep = "true" *) reg [ADDR_W-1:0] dbg_pc_reg;
    (* keep = "true" *) reg dbg_issue_inst2_reg;
    (* keep = "true" *) reg dbg_pipeline_stall_reg;
    (* keep = "true" *) reg dbg_control_flow_reg;
    
    always @(posedge clk) begin
        if (reset) begin
            dbg_pc_reg <= 0;
            dbg_issue_inst2_reg <= 0;
            dbg_pipeline_stall_reg <= 0;
            dbg_control_flow_reg <= 0;
        end else begin
            dbg_pc_reg <= fd_pc;
            dbg_issue_inst2_reg <= issue_inst2;
            dbg_pipeline_stall_reg <= pipeline_stall;
            dbg_control_flow_reg <= control_flow_taken;
        end
    end

    // =========================================================================
    // Memory Stage 
    // =========================================================================
    wire [CPU_DATA_W-1:0] dmem_rdata;
    wire [ADDR_W-1:0]     dmem_addr_mux;
    wire [CPU_DATA_W-1:0] dmem_wdata_mux;
    wire                   dmem_we_mux;

    // Regular memory access from pipeline
    assign dmem_addr_mux  = ex1_Y;                       // ALU result = address
    assign dmem_wdata_mux = rs2_data1;                   // Store data
    assign dmem_we_mux    = (MemWrite1 && is_mem1 && fd_valid);

    // Instantiate Data Memory
    Data_MEM u_dmem (
        .clk(clk),
        .reset(reset),
        .write_en(dmem_we_mux),
        .address(dmem_addr_mux),
        .write_DAT(dmem_wdata_mux),
        .read_DAT(dmem_rdata)
    );

    // Monitor writes (for debug)
    always @(posedge clk) begin
        if (dmem_we_mux)
            $display("[%0t] DMEM_WRITE addr=%0d data=%h", $time, dmem_addr_mux, dmem_wdata_mux);
    end


    // =========================================================================
    // Writeback Stage
    // =========================================================================
    wire [CPU_DATA_W-1:0] result1 = (is_jal1 || is_jalr1) ? link_addr1 : 
                                    (MemToReg1 ? dmem_rdata : ex1_Y);
    wire [CPU_DATA_W-1:0] result2 = ex2_Y;

    always @(posedge clk) begin
        if (reset) begin
            wb_we1_reg <= 1'b0;
            wb_we2_reg <= 1'b0;
            wb_rd1_reg <= 5'd0;
            wb_rd2_reg <= 5'd0;
            wb_wdata1_reg <= 32'd0;
            wb_wdata2_reg <= 32'd0;
        end else begin
            wb_we1_reg <= RegWrite1 && (rd1 != 5'd0) && fd_valid;
            wb_rd1_reg <= rd1;
            wb_wdata1_reg <= result1;
            
            wb_we2_reg <= issue_inst2 && RegWrite2 && (rd2 != 5'd0) && fd_valid;
            wb_rd2_reg <= rd2;
            wb_wdata2_reg <= result2;
        end
    end

    // =========================================================================
    // PC Update and Fetch Control  (CLEAN VERSION)
    // =========================================================================
    always @(*) begin
        if (reset)
            pc_next = 32'd0;
        else if (fft_busy || pipeline_stall)
            pc_next = pc;  // hold during FFT or any stall
        else if (control_flow_taken) begin
            if (is_jal1)       pc_next = jal_target1;
            else if (is_jalr1) pc_next = jalr_target1;
            else               pc_next = branch_target1;
        end else if (issue_inst2)
            pc_next = pc + 8;
        else
            pc_next = pc + 4;
    end

    always @(posedge clk) begin
        if (reset) begin
            pc_reg   <= 32'd0;
            fd_pc    <= 32'd0;
            fd_inst1 <= 32'd0;
            fd_inst2 <= 32'd0;
            fd_valid <= 1'b0;
        end 
        else if (pipeline_stall || fft_busy) begin
            // freeze everything
            pc_reg   <= pc;
            fd_pc    <= fd_pc;
            fd_inst1 <= fd_inst1;
            fd_inst2 <= fd_inst2;
            fd_valid <= 1'b0; // invalidate current decode to prevent RAW false triggers
        end 
        else if (control_flow_taken) begin
            // flush on branch/jump
            pc_reg   <= pc_next;
            fd_pc    <= pc_next;
            fd_inst1 <= 32'h00000013;
            fd_inst2 <= 32'h00000013;
            fd_valid <= 1'b0;
        end 
        else begin
            // normal advance
            pc_reg   <= pc_next;
            fd_pc    <= pc;
            fd_inst1 <= inst1_f;
            fd_inst2 <= inst2_f;
            fd_valid <= 1'b1;
        end
    end

    // =========================================================================
    // Debug Output
    // =========================================================================
    always @(posedge clk) begin
        if (!reset && fd_valid) begin
            //$display("t=%0t PC=%0d [%s|%s] issue2=%b stall=%b", 
             //        $time, dbg_pc_reg, inst1_name_reg, inst2_name_reg, 
              //       dbg_issue_inst2_reg, dbg_pipeline_stall_reg);
            
            //if (wb_we1_reg) 
            //    $display("  WB1: x%0d = 0x%h", wb_rd1_reg, wb_wdata1_reg);
            //if (wb_we2_reg) 
            //    $display("  WB2: x%0d = 0x%h", wb_rd2_reg, wb_wdata2_reg);

            if (fft_start)
                $display(">>> FFT started at t=%0t", $time);
            if (fft_done)
                $display("<<< FFT completed at t=%0t", $time);

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

    // =========================================================================
    // PC Stage
    // =========================================================================
    reg [31:0] pc_reg;
    wire [31:0] pc = pc_reg;
    wire [31:0] pc_plus_4 = pc + 32'd4;
    
    // =========================================================================
    // Fetch Stage  
    // =========================================================================
    wire [31:0] inst1_f, inst2_f;
    Inst_MEM u_Inst_MEM1(.address(pc), .inst(inst1_f));
    Inst_MEM u_Inst_MEM2(.address(pc_plus_4), .inst(inst2_f));
    
    // Fetch/Decode pipeline registers
    reg [31:0] fd_pc; // Pipelined PC (for fd_inst1)
    reg [31:0] fd_inst1, fd_inst2;
    reg fd_valid;
    
    // =========================================================================
    // Decode Stage (uses fd_inst1, fd_inst2)
    // =========================================================================
    wire [6:0] opcode1 = fd_inst1[6:0];
    wire [4:0] rd1 = fd_inst1[11:7];
    wire [2:0] funct3_1 = fd_inst1[14:12];
    wire [4:0] rs1_1 = fd_inst1[19:15];
    wire [4:0] rs2_1 = fd_inst1[24:20];
    wire [6:0] funct7_1 = fd_inst1[31:25];

    wire [6:0] opcode2 = fd_inst2[6:0];
    wire [4:0] rd2 = fd_inst2[11:7];
    wire [2:0] funct3_2 = fd_inst2[14:12];
    wire [4:0] rs1_2 = fd_inst2[19:15];
    wire [4:0] rs2_2 = fd_inst2[24:20];
    wire [6:0] funct7_2 = fd_inst2[31:25];

    // Immediate generation (RISC-V correct encoding)
    wire [31:0] imm_i1 = {{20{fd_inst1[31]}}, fd_inst1[31:20]};
    wire [31:0] imm_i2 = {{20{fd_inst2[31]}}, fd_inst2[31:20]};

    wire [31:0] imm_s1 = {{20{fd_inst1[31]}}, fd_inst1[31:25], fd_inst1[11:7]};
    wire [31:0] imm_s2 = {{20{fd_inst2[31]}}, fd_inst2[31:25], fd_inst2[11:7]};

    wire [31:0] imm_b1 = {{19{fd_inst1[31]}}, fd_inst1[31], fd_inst1[7],
                        fd_inst1[30:25], fd_inst1[11:8], 1'b0};
    wire [31:0] imm_b2 = {{19{fd_inst2[31]}}, fd_inst2[31], fd_inst2[7],
                        fd_inst2[30:25], fd_inst2[11:8], 1'b0};

    wire [31:0] imm_j1 = {{11{fd_inst1[31]}}, fd_inst1[31],
                        fd_inst1[19:12], fd_inst1[20],
                        fd_inst1[30:21], 1'b0};
    wire [31:0] imm_j2 = {{11{fd_inst2[31]}}, fd_inst2[31],
                        fd_inst2[19:12], fd_inst2[20],
                        fd_inst2[30:21], 1'b0};

    // Branch/Jump targets
    wire [31:0] pc_inst1 = fd_pc;
    wire [31:0] pc_inst2 = fd_pc + 32'd4;
    
    wire [31:0] jal_target1 = pc_inst1 + imm_j1;
    wire [31:0] jal_target2 = pc_inst2 + imm_j2;
    wire [31:0] branch_target1 = pc_inst1 + imm_b1;
    
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
    // <<< MOVED DECLARATIONS HERE (Before Hazard Detection) >>>
    // Forwarding logic needs results (ex1_Y) calculated here now
    // Execute Stage Results (needed for forwarding & writeback)
    // =========================================================================
    wire [31:0] alu_Y1, alu_Y2;
    // ALU inputs defined later after forwarding logic
    wire forward_rs1_2; // Declare forward signals early
    wire forward_rs2_2;
    wire [31:0] ex1_Y; // Declare result signal early
    wire [31:0] rs1_data2_fwd = forward_rs1_2 ? ex1_Y : rs1_data2;
    wire [31:0] rs2_data2_fwd = forward_rs2_2 ? ex1_Y : rs2_data2; // Corrected width
    
    wire [31:0] alu_B1 = ALUSrc1 ? ((opcode1 == `OP_STORE) ? imm_s1 : imm_i1) : rs2_data1;
    wire [31:0] alu_B2 = ALUSrc2 ? ((opcode2 == `OP_STORE) ? imm_s2 : imm_i2) : rs2_data2_fwd;

    ALU u_alu1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(alu_Y1));
    ALU u_alu2 (.A(rs1_data2_fwd), .B(alu_B2), .ctrl(alu_ctrl2), .Y(alu_Y2));
    
    assign ex1_Y = alu_Y1; // Assign result signal
    wire [31:0] ex2_Y = alu_Y2; // Local wire ok

    // Forwarding logic calculation
    assign forward_rs1_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs1_2) && !MemToReg1;
    assign forward_rs2_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs2_2) && !MemToReg1;


    // JAL/JALR Signals (needed for control_flow_taken)
    wire is_jal1 = (opcode1 == `OP_JAL);
    wire is_jalr1 = (opcode1 == `OP_JALR);
    wire is_jal2 = (opcode2 == `OP_JAL); 
    wire is_jalr2 = (opcode2 == `OP_JALR); 
    
    wire [31:0] link_addr1 = pc_inst1 + 32'd4;
    wire [31:0] link_addr2 = pc_inst2 + 32'd4;
    
    wire [31:0] jalr_target1 = (rs1_data1 + imm_i1) & ~32'd1;

    // Branch Logic (needed for control_flow_taken)
    wire branch_taken1 = (opcode1 == `OP_BRANCH) && (
        (funct3_1 == 3'b000 && rs1_data1 == rs2_data1) ||  // BEQ
        (funct3_1 == 3'b001 && rs1_data1 != rs2_data1)     // BNE
        // Add other branch types here if implemented
    );
    // =========================================================================

    // =========================================================================
    // Hazard Detection
    // =========================================================================
    wire is_mem1 = (opcode1 == `OP_LOAD) || (opcode1 == `OP_STORE);
    wire is_mem2 = (opcode2 == `OP_LOAD) || (opcode2 == `OP_STORE);
    
    // Track if there's a load in flight (in EX/MEM stage)
    reg load_in_ex;        
    reg load_in_mem;       
    reg [4:0] load_dest_ex;
    reg [4:0] load_dest_mem;
    
    // Control flow instruction check (used later)
    wire is_control_flow1 = is_jal1 || is_jalr1 || (opcode1 == `OP_BRANCH); // Use already declared signals
    wire is_control_flow2 = is_jal2 || is_jalr2 || (opcode2 == `OP_BRANCH); // Use already declared signals

    // Signal used for flushing hazard tracking and pipeline
    wire control_flow_taken = (is_jal1 || is_jalr1 || branch_taken1) && fd_valid; 

    // Load-use hazard: instruction in decode reads register that a load (in EX or MEM stage) will write
    wire load_use_rs1_ex_hazard = load_in_ex && (load_dest_ex == rs1_1) && (rs1_1 != 5'd0);
    wire load_use_rs2_ex_hazard = load_in_ex && (load_dest_ex == rs2_1) && (rs2_1 != 5'd0);
    wire load_use_rs1_mem_hazard = load_in_mem && (load_dest_mem == rs1_1) && (rs1_1 != 5'd0);
    wire load_use_rs2_mem_hazard = load_in_mem && (load_dest_mem == rs2_1) && (rs2_1 != 5'd0);
    
    // Original load hazard detection (based on EX/MEM stages)
    wire load_use_hazard_detected = (load_use_rs1_ex_hazard || load_use_rs2_ex_hazard || 
                                     load_use_rs1_mem_hazard || load_use_rs2_mem_hazard) && fd_valid;

    // Anticipate hazard between inst1 (load) and inst2 in Decode stage
    wire load_use_hazard_decode_stage = fd_valid && (opcode1 == `OP_LOAD) && (rd1 != 5'd0) &&
                                        ( (rd1 == rs1_2 && rs1_2 != 5'd0) || (rd1 == rs2_2 && rs2_2 != 5'd0) ); // Ensure non-zero registers

    // Combine current detection with anticipation
    wire load_use_hazard_detected_combined = load_use_hazard_detected || load_use_hazard_decode_stage;

    // Signal to determine if the pipeline should stall completely
    wire pipeline_stall = load_use_hazard_detected_combined; 

    // === REVISED Hazard Tracking Logic ===
    always @(posedge clk) begin
        if (reset || control_flow_taken) begin
            load_in_ex <= 1'b0;
            load_in_mem <= 1'b0;
            load_dest_ex <= 5'd0;
            load_dest_mem <= 5'd0;
        // Advance hazard tracking ONLY if the pipeline is NOT stalled
        end else if (!pipeline_stall) begin 
            // Shift state: EX -> MEM
            load_in_mem <= load_in_ex;
            load_dest_mem <= load_dest_ex;
            // Check instruction ENTERING EX (which is fd_inst1 from the *previous* cycle, 
            // but since fd_valid is checked here, it reflects the instruction just decoded)
            if (fd_valid && (opcode1 == `OP_LOAD) && (rd1 != 5'd0)) begin
                load_in_ex <= 1'b1;
                load_dest_ex <= rd1;
            end else begin
                load_in_ex <= 1'b0;
                load_dest_ex <= 5'd0;
            end
        // During a stall, the instruction in EX moves to MEM, and nothing new enters EX
        end else begin 
            load_in_mem <= load_in_ex; // EX moves to MEM
            load_dest_mem <= load_dest_ex;
            load_in_ex <= 1'b0; // EX becomes empty/idle as Decode is frozen
            load_dest_ex <= 5'd0;
        end
    end
                                     
    // RAW hazard: inst2 reads register that inst1 writes (non-load)
    wire raw_rs1 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs1_2) && !(opcode1 == `OP_LOAD); 
    wire raw_rs2 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs2_2) && !(opcode1 == `OP_LOAD); 
    wire raw_hazard = raw_rs1 || raw_rs2;
    
    // Load-use hazard specifically between inst1(LOAD) and inst2 in the same decode cycle
    // Use the decode stage detection wire here
    wire load_use_hazard_inst1_inst2 = load_use_hazard_decode_stage; 

    // WAW hazard
    wire waw_hazard = (rd1 != 5'd0) && (rd1 == rd2) && RegWrite1 && RegWrite2;
    
    // Blocking conditions for issuing instruction 2
    wire blocks_superscalar = is_mem1 || is_mem2 || is_control_flow1 || is_control_flow2 || load_use_hazard_inst1_inst2;
    
    // Issue logic for instruction 2
    // Must NOT issue inst2 if there's a pipeline stall
    wire issue_inst2 = ~raw_hazard && ~waw_hazard && ~blocks_superscalar && fd_valid && !pipeline_stall; 

    // =========================================================================
    // Instruction Name Decode (for waveform viewing)
    // =========================================================================
    reg [8*8-1:0] inst1_name;
    reg [8*8-1:0] inst2_name;
    
    (* keep = "true" *) (* mark_debug = "true" *) reg [8*8-1:0] inst1_name_reg;
    (* keep = "true" *) (* mark_debug = "true" *) reg [8*8-1:0] inst2_name_reg;
    
    always @(*) begin
        inst1_name = "DECODE1?";
        inst2_name = "DECODE2?";

        // Decode inst1
        if (!fd_valid) begin
            inst1_name = "INVALID";
        end else if (fd_inst1 == 32'h00000013) begin
            inst1_name = "NOP";
        end else begin
            case (opcode1)
                `OP_LOAD: inst1_name = "LOAD"; 
                `OP_STORE: inst1_name = "STORE";
                `OP_BRANCH: begin
                    case (funct3_1)
                        3'b000: inst1_name = "BEQ";
                        3'b001: inst1_name = "BNE";
                        default: inst1_name = "BRANCH?";
                    endcase
                end
                `OP_JAL: inst1_name = "JAL";
                `OP_JALR: inst1_name = "JALR";
                `OP_ITYPE: begin
                    case (funct3_1)
                        3'b000: inst1_name = "ADDI";
                        3'b010: inst1_name = "SLTI";
                        3'b011: inst1_name = "SLTIU";
                        3'b100: inst1_name = "XORI";
                        3'b110: inst1_name = "ORI";
                        3'b111: inst1_name = "ANDI";
                        default: inst1_name = "ITYPE?";
                    endcase
                end
                `OP_RTYPE: begin
                    case (funct3_1)
                        3'b000: inst1_name = (funct7_1 == 7'b0100000) ? "SUB" : "ADD";
                        default: inst1_name = "RTYPE";
                    endcase
                end
                `OP_LUI: inst1_name = "LUI";
                `OP_AUIPC: inst1_name = "AUIPC";
                default: inst1_name = "???";
            endcase
        end

        // Decode inst2
        if (!fd_valid) begin
            inst2_name = "INVALID";
        // If pipeline is stalled, inst2 is effectively stalled regardless of issue_inst2 signal
        end else if (pipeline_stall) begin 
            inst2_name = "STALLED";
        end else if (!issue_inst2) begin
            inst2_name = "STALLED"; 
        end else if (fd_inst2 == 32'h00000013) begin
            inst2_name = "NOP";
        end else begin
            case (opcode2)
                `OP_LOAD: inst2_name = "LOAD";
                `OP_STORE: inst2_name = "STORE";
                `OP_BRANCH: inst2_name = "BRANCH";
                `OP_JAL: inst2_name = "JAL";
                `OP_JALR: inst2_name = "JALR";
                `OP_ITYPE: begin
                    case (funct3_2)
                        3'b000: inst2_name = "ADDI";
                        3'b010: inst2_name = "SLTI";
                        3'b011: inst2_name = "SLTIU";
                        3'b100: inst2_name = "XORI";
                        3'b110: inst2_name = "ORI";
                        3'b111: inst2_name = "ANDI";
                        default: inst2_name = "ITYPE?";
                    endcase
                end
                `OP_RTYPE: begin
                    case (funct3_2)
                        3'b000: inst2_name = (funct7_2 == 7'b0100000) ? "SUB" : "ADD";
                        default: inst2_name = "RTYPE";
                    endcase
                end
                `OP_LUI: inst2_name = "LUI";
                `OP_AUIPC: inst2_name = "AUIPC";
                default: inst2_name = "???";
            endcase
        end
    end
    
    // Register instruction names for waveform viewing
    always @(posedge clk) begin
        if (reset) begin
            inst1_name_reg <= "RESET   ";
            inst2_name_reg <= "RESET   ";
        end else begin
            inst1_name_reg <= inst1_name;
            inst2_name_reg <= inst2_name;
        end
    end

    // =========================================================================
    // Additional Debug Registers
    // =========================================================================
    (* keep = "true" *) reg [31:0] dbg_pc_reg;
    (* keep = "true" *) reg dbg_issue_inst2_reg;
    (* keep = "true" *) reg dbg_load_use_stall_reg;
    (* keep = "true" *) reg dbg_control_flow_reg;
    (* keep = "true" *) reg dbg_load_in_ex_reg; // Debug register for hazard signal
    (* keep = "true" *) reg [4:0] dbg_load_dest_ex_reg; // Debug register for hazard signal
    (* keep = "true" *) reg dbg_load_in_mem_reg; // Debug register for hazard signal
    (* keep = "true" *) reg [4:0] dbg_load_dest_mem_reg; // Debug register for hazard signal
    (* keep = "true" *) reg dbg_hazard_detected_reg; // Debug register for hazard signal
    (* keep = "true" *) reg dbg_hazard_decode_stage_reg; // Debug register for hazard signal


    always @(posedge clk) begin
        if (reset) begin
            dbg_pc_reg <= 0;
            dbg_issue_inst2_reg <= 0;
            dbg_load_use_stall_reg <= 0;
            dbg_control_flow_reg <= 0;
            dbg_load_in_ex_reg <= 0; // Reset debug hazard signals
            dbg_load_dest_ex_reg <= 0;
            dbg_load_in_mem_reg <= 0;
            dbg_load_dest_mem_reg <= 0;
            dbg_hazard_detected_reg <= 0;
            dbg_hazard_decode_stage_reg <= 0;
        end else begin
            dbg_pc_reg <= fd_pc; // Latch fd_pc for debug alignment
            dbg_issue_inst2_reg <= issue_inst2;
            dbg_load_use_stall_reg <= pipeline_stall; // Use the final stall signal
            dbg_control_flow_reg <= control_flow_taken; // Use the actual taken signal
            // Latch hazard detection signals for easier viewing
            dbg_load_in_ex_reg <= load_in_ex; 
            dbg_load_dest_ex_reg <= load_dest_ex;
            dbg_load_in_mem_reg <= load_in_mem;
            dbg_load_dest_mem_reg <= load_dest_mem;
            dbg_hazard_detected_reg <= load_use_hazard_detected;
            dbg_hazard_decode_stage_reg <= load_use_hazard_decode_stage;
        end
    end

    // =========================================================================
    // Execute Stage (Original position - now only forwarding logic here)
    // =========================================================================
    // Forwarding logic handled earlier with moved ALU block

    // =========================================================================
    // Memory Stage
    // =========================================================================
    wire [31:0] mem_rdata;
    wire [31:0] mem_addr = ex1_Y; // Address comes from ALU result (for LW/SW)
    wire [31:0] mem_wdata = rs2_data1; // Data to write comes from rs2
    wire mem_we = MemWrite1 && is_mem1; // Write enable only for STORE type
    
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
    
    // Determine the result to write back for inst1
    wire [31:0] result1 = (is_jal1 || is_jalr1) ? link_addr1 : (MemToReg1 ? mem_rdata : ex1_Y);
    
    // Determine the result to write back for inst2
    wire [31:0] result2 = ex2_Y; // Only ALU result possible for inst2
    
    always @(posedge clk) begin
        if (reset) begin
            wb_we1 <= 1'b0;
            wb_we2 <= 1'b0;
            wb_rd1 <= 5'd0;
            wb_rd2 <= 5'd0;
            wb_wdata1 <= 32'd0;
            wb_wdata2 <= 32'd0;
        end else begin
            // Writeback depends on the instruction that *was* in Decode before the clock edge
            // Need pipelined control signals for perfect accuracy, but using Decode stage
            // signals directly is a common simplification (assuming 1 cycle EX/MEM)
            wb_we1 <= RegWrite1 && (rd1 != 5'd0) && fd_valid; 
            wb_rd1 <= rd1;
            wb_wdata1 <= result1;
            
            wb_we2 <= issue_inst2 && RegWrite2 && (rd2 != 5'd0) && fd_valid; 
            wb_rd2 <= rd2;
            wb_wdata2 <= result2;
        end
    end

    // =========================================================================
    // PC Update and Fetch Control
    // =========================================================================
    reg [31:0] pc_next;
    reg fetch_stall; // Indicates only inst1 issued, need to slide inst2->inst1
    // wire pipeline_stall; // Defined earlier
    
    // control_flow_taken defined earlier using moved signals
    
    // Determine if the pipeline needs a full stall
    // assign pipeline_stall = load_use_hazard_detected_combined; // Defined earlier
    
    always @(*) begin
        fetch_stall = 1'b0; // Default: assume superscalar or flush/stall
        
        if (reset) begin
            pc_next = 32'd0;
        end else if (!fd_valid) begin
            pc_next = pc; 
        // === HIGHEST PRIORITY: Load-use stall ===
        end else if (pipeline_stall) begin // Uses combined signal
            pc_next = pc; // Hold the PC
            fetch_stall = 1'b0; // It's a full pipeline stall
        // === Next Priority: Taken control flow ===
        end else if (control_flow_taken) begin 
            // Target PC is calculated based on type (using signals declared earlier)
            if (is_jal1)      pc_next = jal_target1;
            else if (is_jalr1) pc_next = jalr_target1;
            else              pc_next = branch_target1; // branch_taken1 is true
            fetch_stall = 1'b0; // Flush handles this
        // === Default: Advance PC ===
        end else begin
            if (issue_inst2) begin
                pc_next = pc + 8; // Advance by 2 instructions
                fetch_stall = 1'b0; 
            end else begin
                pc_next = pc + 4; // Advance by 1 instruction
                fetch_stall = 1'b1; // Signal that inst2 needs to slide into inst1 slot
            end
        end
    end
    
    // Pipeline Register Updates (FD Stage)
    always @(posedge clk) begin
        if (reset) begin
            pc_reg <= 32'd0;
            fd_pc <= 32'd0;
            fd_inst1 <= 32'd0;
            fd_inst2 <= 32'd0;
            fd_valid <= 1'b0;
        // === CHECK FOR FULL PIPELINE STALL FIRST ===
        end else if (pipeline_stall) begin // Uses combined signal
            // FREEZE PC and FD registers
            pc_reg <= pc; // Explicitly hold PC reg
            fd_pc <= fd_pc; // Hold pipelined PC
            fd_inst1 <= fd_inst1; // Hold instruction 1
            fd_inst2 <= fd_inst2; // Hold instruction 2
            fd_valid <= fd_valid; // Keep valid state
        // === If not stalled, proceed with normal updates ===
        end else begin 
            // Update PC to the calculated next value
            pc_reg <= pc_next; 
            
            // Update FD registers based on state
            if (!fd_valid) begin // Initial fetch
                fd_pc <= pc;
                fd_inst1 <= inst1_f;
                fd_inst2 <= inst2_f;
                fd_valid <= 1'b1;
            end else if (control_flow_taken) begin // Flush
                fd_pc <= pc_next; // NOPs are at the target PC
                fd_inst1 <= 32'h00000013; // NOP
                fd_inst2 <= 32'h00000013; // NOP
                fd_valid <= 1'b1; 
            end else if (fetch_stall) begin // Slide inst2 -> inst1
                fd_pc <= pc + 4; // PC for the instruction that was fd_inst2 (relative to old pc)
                fd_inst1 <= fd_inst2;
                fd_inst2 <= inst2_f; // Fetch new inst2 from pc+4 (relative to old pc)
                fd_valid <= 1'b1;
            end else begin // Normal superscalar fetch
                fd_pc <= pc; // PC for new inst1
                fd_inst1 <= inst1_f;
                fd_inst2 <= inst2_f;
                fd_valid <= 1'b1;
            end
        end // End else (!pipeline_stall)
    end // End always @(posedge clk)

    // =========================================================================
    // Debug Output (ADDED HAZARD SIGNALS)
    // =========================================================================
    always @(posedge clk) begin
         if (!reset && fd_valid) begin // Check fd_valid before displaying names
            // Use combinatorial signals for current PC, names, issue, stall status
            $display("t=%0t PC=%0d [%s|%s] issue2=%b pipeline_stall=%b",
                     $time, fd_pc, inst1_name, inst2_name, issue_inst2, pipeline_stall);

            // Display hazard tracking state *before* the clock edge (using registered dbg signals is correct here)
            $display("  Hazard State (Regs): LoadEX=%b(x%0d), LoadMEM=%b(x%0d)",
                     dbg_load_in_ex_reg, dbg_load_dest_ex_reg, dbg_load_in_mem_reg, dbg_load_dest_mem_reg);
            // Display combinatorial hazard signals for current cycle
            $display("  Hazard Signals (Comb): Detect=%b, DecodeStage=%b, CombinedStall=%b",
                     load_use_hazard_detected, load_use_hazard_decode_stage, pipeline_stall);
             // Use combinatorial signals for current decode cycle display - Shows what is being evaluated NOW
            $display("  Inst1 Decode: op=%h rd=%d rs1=%d rs2=%d | Inst2 Decode: op=%h rd=%d rs1=%d rs2=%d",
                     opcode1, rd1, rs1_1, rs2_1, opcode2, rd2, rs1_2, rs2_2);

            if (wb_we1)
                $display("  WB1: x%0d = 0x%h (%0d)", wb_rd1, wb_wdata1, wb_wdata1);
            if (wb_we2)
                $display("  WB2: x%0d = 0x%h (%0d)", wb_rd2, wb_wdata2, wb_wdata2);

        end else if (!reset) begin
             $display("t=%0t PC=%0d [INVALID|INVALID] Pipeline not valid.", $time, pc_reg);
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

    // =========================================================================
    // PC Stage
    // =========================================================================
    reg [31:0] pc_reg;
    wire [31:0] pc = pc_reg;
    wire [31:0] pc_plus_4 = pc + 32'd4;
    
    // =========================================================================
    // Fetch Stage  
    // =========================================================================
    wire [31:0] inst1_f, inst2_f;
    Inst_MEM u_Inst_MEM1(.address(pc), .inst(inst1_f));
    Inst_MEM u_Inst_MEM2(.address(pc_plus_4), .inst(inst2_f));
    
    // Fetch/Decode pipeline registers
    reg [31:0] fd_pc; // Pipelined PC (for fd_inst1)
    reg [31:0] fd_inst1, fd_inst2;
    reg fd_valid;
    
    // =========================================================================
    // Decode Stage (uses fd_inst1, fd_inst2)
    // =========================================================================
    wire [6:0] opcode1 = fd_inst1[6:0];
    wire [4:0] rd1 = fd_inst1[11:7];
    wire [2:0] funct3_1 = fd_inst1[14:12];
    wire [4:0] rs1_1 = fd_inst1[19:15];
    wire [4:0] rs2_1 = fd_inst1[24:20];
    wire [6:0] funct7_1 = fd_inst1[31:25];

    wire [6:0] opcode2 = fd_inst2[6:0];
    wire [4:0] rd2 = fd_inst2[11:7];
    wire [2:0] funct3_2 = fd_inst2[14:12];
    wire [4:0] rs1_2 = fd_inst2[19:15];
    wire [4:0] rs2_2 = fd_inst2[24:20];
    wire [6:0] funct7_2 = fd_inst2[31:25];

    // Immediate generation (RISC-V correct encoding)
    // I-type: [31:20]
    wire [31:0] imm_i1 = {{20{fd_inst1[31]}}, fd_inst1[31:20]};
    wire [31:0] imm_i2 = {{20{fd_inst2[31]}}, fd_inst2[31:20]};

    // S-type: [31:25 | 11:7]
    wire [31:0] imm_s1 = {{20{fd_inst1[31]}}, fd_inst1[31:25], fd_inst1[11:7]};
    wire [31:0] imm_s2 = {{20{fd_inst2[31]}}, fd_inst2[31:25], fd_inst2[11:7]};

    // B-type: [31 | 7 | 30:25 | 11:8 | 0]
    wire [31:0] imm_b1 = {{19{fd_inst1[31]}}, fd_inst1[31], fd_inst1[7],
                        fd_inst1[30:25], fd_inst1[11:8], 1'b0};
    wire [31:0] imm_b2 = {{19{fd_inst2[31]}}, fd_inst2[31], fd_inst2[7],
                        fd_inst2[30:25], fd_inst2[11:8], 1'b0};

    // J-type: [31 | 19:12 | 20 | 30:21 | 0]
    wire [31:0] imm_j1 = {{11{fd_inst1[31]}}, fd_inst1[31],
                        fd_inst1[19:12], fd_inst1[20],
                        fd_inst1[30:21], 1'b0};
    wire [31:0] imm_j2 = {{11{fd_inst2[31]}}, fd_inst2[31],
                        fd_inst2[19:12], fd_inst2[20],
                        fd_inst2[30:21], 1'b0};

    // Branch/Jump targets
    // Use the pipelined PC 'fd_pc' which corresponds to fd_inst1
    wire [31:0] pc_inst1 = fd_pc;
    wire [31:0] pc_inst2 = fd_pc + 32'd4;
    
    wire [31:0] jal_target1 = pc_inst1 + imm_j1;
    wire [31:0] jal_target2 = pc_inst1 + imm_j2; // Note: JAL2 target would be from pc_inst2
    wire [31:0] branch_target1 = pc_inst1 + imm_b1;
    
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
    
    // Control flow instructions block superscalar execution
    wire is_control_flow1 = (opcode1 == `OP_BRANCH) || (opcode1 == `OP_JAL) || (opcode1 == `OP_JALR);
    wire is_control_flow2 = (opcode2 == `OP_BRANCH) || (opcode2 == `OP_JAL) || (opcode2 == `OP_JALR);
    
    // RAW hazard: inst2 reads a register that inst1 writes
    wire raw_rs1 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs1_2);
    wire raw_rs2 = (rd1 != 5'd0) && RegWrite1 && (rd1 == rs2_2);
    wire raw_hazard = raw_rs1 || raw_rs2;
    
    // Load-use hazard
    wire load_use_hazard = (opcode1 == `OP_LOAD) && raw_hazard;
    
    // WAW hazard
    wire waw_hazard = (rd1 != 5'd0) && (rd1 == rd2) && RegWrite1 && RegWrite2;
    
    // Blocking conditions
    wire blocks_superscalar = is_mem1 || is_mem2 || is_control_flow1 || is_control_flow2;
    
    // Issue logic
    wire issue_inst2 = ~load_use_hazard && ~waw_hazard && ~blocks_superscalar && fd_valid;

    // =========================================================================
    // <<< NEW DEBUG LOGIC START >>>
    // =========================================================================
    reg [8*8:1] inst1_name ; // Try alternative keep style
    reg [8*8:1] inst2_name ; // Try alternative keep style
    // (* keep = "true" *) reg [8*8:1] inst1_name; // Keep this one too, just in case
    // (* keep = "true" *) reg [8*8:1] inst2_name; 

    always @(*) begin
        // Default assignment (optional, but can help ensure it's always driven)
        inst1_name = "DECODE1?";
        inst2_name = "DECODE2?";

        // Decode for inst1_name
        if (!fd_valid) begin
            inst1_name = "INVALID";
        end else if (fd_inst1 == 32'h00000013) begin // NOP
            inst1_name = "NOP";
        end else begin
            case (opcode1)
                `OP_LOAD: inst1_name = "LOAD"; 
                `OP_STORE: inst1_name = "STORE";
                `OP_BRANCH: begin
                    case (funct3_1)
                        3'b000: inst1_name = "BEQ";
                        3'b001: inst1_name = "BNE";
                        default: inst1_name = "BRANCH?";
                    endcase
                end
                `OP_JAL: inst1_name = "JAL";
                `OP_JALR: inst1_name = "JALR";
                `OP_ITYPE: begin // Use OP_ITYPE from opcodes.vh
                    case (funct3_1)
                        3'b000: inst1_name = "ADDI";
                        3'b010: inst1_name = "SLTI";
                        3'b011: inst1_name = "SLTIU";
                        3'b100: inst1_name = "XORI";
                        3'b110: inst1_name = "ORI";
                        3'b111: inst1_name = "ANDI";
                        default: inst1_name = "ITYPE?"; // Changed default name slightly
                    endcase
                end
                `OP_RTYPE: begin // Use OP_RTYPE from opcodes.vh
                    case (funct3_1)
                        3'b000: begin
                            if (funct7_1 == 7'b0100000)
                                inst1_name = "SUB";
                            else
                                inst1_name = "ADD";
                        end
                        // Add other R-type ops if needed
                        default: inst1_name = "RTYPE"; // Changed default name slightly
                    endcase
                end
                `OP_LUI: inst1_name = "LUI";
                `OP_AUIPC: inst1_name = "AUIPC";
                default: inst1_name = "???";
            endcase
        end

        // Decode for inst2_name
        if (!fd_valid) begin
            inst2_name = "INVALID";
        end else if (!issue_inst2) begin
             inst2_name = "STALLED"; 
        end else if (fd_inst2 == 32'h00000013) begin
            inst2_name = "NOP";
        end else begin
            case (opcode2)
                `OP_LOAD: inst2_name = "LOAD";
                `OP_STORE: inst2_name = "STORE";
                `OP_BRANCH: inst2_name = "BRANCH";
                `OP_JAL: inst2_name = "JAL";
                `OP_JALR: inst2_name = "JALR";
                `OP_ITYPE: begin // Use OP_ITYPE from opcodes.vh
                    case (funct3_2)
                        3'b000: inst2_name = "ADDI";
                        3'b010: inst2_name = "SLTI";
                        3'b011: inst2_name = "SLTIU";
                        3'b100: inst2_name = "XORI";
                        3'b110: inst2_name = "ORI";
                        3'b111: inst2_name = "ANDI";
                        default: inst2_name = "ITYPE?"; // Changed default name slightly
                    endcase
                end
                `OP_RTYPE: begin // Use OP_RTYPE from opcodes.vh
                    case (funct3_2)
                        3'b000: begin
                            if (funct7_2 == 7'b0100000)
                                inst2_name = "SUB";
                            else
                                inst2_name = "ADD";
                        end
                        default: inst2_name = "RTYPE"; // Changed default name slightly
                    endcase
                end
                `OP_LUI: inst2_name = "LUI";
                `OP_AUIPC: inst2_name = "AUIPC";
                default: inst2_name = "???";
            endcase
        end
    end // End always@(*)
    // =========================================================================
    // <<< NEW DEBUG LOGIC END >>>
    // =========================================================================

    // =========================================================================
    // Execute Stage
    // =========================================================================
    // EX-EX forwarding
    wire forward_rs1_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs1_2) && !MemToReg1;
    wire forward_rs2_2 = RegWrite1 && (rd1 != 5'd0) && (rd1 == rs2_2) && !MemToReg1;
    
    wire [31:0] rs1_data2_fwd = forward_rs1_2 ? ex1_Y : rs1_data2;
    wire [31:0] rs2_data2_fwd = forward_rs2_2 ? ex1_Y : rs2_data2; // Corrected width
    
    wire [31:0] alu_B1 = ALUSrc1 ? ((opcode1 == `OP_STORE) ? imm_s1 : imm_i1) : rs2_data1;
    wire [31:0] alu_B2 = ALUSrc2 ? ((opcode2 == `OP_STORE) ? imm_s2 : imm_i2) : rs2_data2_fwd;
    
    wire [31:0] alu_Y1, alu_Y2; 
    // Remove MAC instances if not used/defined
    // wire [31:0] mac_Y1, mac_Y2; 
    
    ALU u_alu1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(alu_Y1));
    // MAC u_mac1 (.A(rs1_data1), .B(alu_B1), .ctrl(alu_ctrl1), .Y(mac_Y1));
    
    ALU u_alu2 (.A(rs1_data2_fwd), .B(alu_B2), .ctrl(alu_ctrl2), .Y(alu_Y2));
    // MAC u_mac2 (.A(rs1_data2_fwd), .B(alu_B2), .ctrl(alu_ctrl2), .Y(mac_Y2));
    
    // Remove MAC logic if MAC unit is not present
    // wire uses_mac1 = (alu_ctrl1 >= 5'd10 && alu_ctrl1 <= 5'd13);
    // wire uses_mac2 = (alu_ctrl2 >= 5'd10 && alu_ctrl2 <= 5'd13);
    
    // wire [31:0] ex1_Y = uses_mac1 ? mac_Y1 : alu_Y1;
    // wire [31:0] ex2_Y = uses_mac2 ? mac_Y2 : alu_Y2;
    wire [31:0] ex1_Y = alu_Y1; // Assign directly if no MAC
    wire [31:0] ex2_Y = alu_Y2; // Assign directly if no MAC
    
    // JAL/JALR
    wire is_jal1 = (opcode1 == `OP_JAL);
    wire is_jalr1 = (opcode1 == `OP_JALR);
    wire is_jal2 = (opcode2 == `OP_JAL);
    wire is_jalr2 = (opcode2 == `OP_JALR);
    
    wire [31:0] link_addr1 = pc_inst1 + 32'd4;
    wire [31:0] link_addr2 = pc_inst2 + 32'd4; // pc + 8
    
    // JALR target
    wire [31:0] jalr_target1 = (rs1_data1 + imm_i1) & ~32'd1;

    // =========================================================================
    // Branch Logic
    // =========================================================================
    wire branch_taken1 = (opcode1 == `OP_BRANCH) && (
        (funct3_1 == 3'b000 && rs1_data1 == rs2_data1) ||  // BEQ
        (funct3_1 == 3'b001 && rs1_data1 != rs2_data1)     // BNE
    );

    // =========================================================================
    // Memory Stage
    // =========================================================================
    wire [31:0] mem_rdata;
    wire [31:0] mem_addr = ex1_Y; // Address comes from ALU result (for LW/SW)
    wire [31:0] mem_wdata = rs2_data1; // Data to write comes from rs2
    wire mem_we = MemWrite1 && is_mem1; // Write enable only for STORE type
    
    // <<< RE-INSERTED DATA MEMORY INSTANCE >>>
    Data_MEM u_dmem (
        .clk(clk),
        .reset(reset),
        .write_en(mem_we),
        .address(mem_addr),
        .write_DAT(mem_wdata),
        .read_DAT(mem_rdata)
    );
    // <<< END RE-INSERTED BLOCK >>>

    // =========================================================================
    // Writeback Stage
    // =========================================================================
    reg wb_we1, wb_we2;
    reg [4:0] wb_rd1, wb_rd2;
    reg [31:0] wb_wdata1, wb_wdata2;
    
    // Determine the result to write back for inst1
    // If JAL/JALR, write link address. If LOAD, write memory data. Otherwise, write ALU result.
    wire [31:0] result1 = (is_jal1 || is_jalr1) ? link_addr1 : (MemToReg1 ? mem_rdata : ex1_Y);
    
    // Determine the result to write back for inst2
    // JAL2/JALR2 are blocked, LOAD2/STORE2 are blocked. Only ALU/MAC result needed.
    wire [31:0] result2 = ex2_Y; // Since MEM/JAL/JALR for inst2 are blocked by 'blocks_superscalar'
    
    always @(posedge clk) begin
        if (reset) begin
            wb_we1 <= 1'b0;
            wb_we2 <= 1'b0;
            wb_rd1 <= 5'd0;
            wb_rd2 <= 5'd0;
            wb_wdata1 <= 32'd0;
            wb_wdata2 <= 32'd0;
        end else begin
            // We only writeback if the instruction was valid when it was decoded
            wb_we1 <= RegWrite1 && (rd1 != 5'd0) && fd_valid;
            wb_rd1 <= rd1;
            wb_wdata1 <= result1;
            
            // We only writeback if inst2 was valid AND issued
            wb_we2 <= issue_inst2 && RegWrite2 && (rd2 != 5'd0) && fd_valid;
            wb_rd2 <= rd2;
            wb_wdata2 <= result2;
        end
    end

    // =========================================================================
    // PC Update and Fetch Control
    // =========================================================================
    reg [31:0] pc_next;
    reg fetch_stall;
    
    // Detect control flow changes that require pipeline flush
    // This is a combinatorial signal based on the *current* decoded instruction
    wire control_flow_taken = (is_jal1 || is_jalr1 || branch_taken1) && fd_valid;
    
    always @(*) begin
        fetch_stall = 1'b0;
        
        if (reset) begin
            pc_next = 32'd0;
        end else if (!fd_valid) begin
            // Initial state: hold PC while fetching
            pc_next = pc;
        // Check for taken control flow instructions first
        end else if (is_jal1) begin
            pc_next = jal_target1;
        end else if (is_jalr1) begin
            pc_next = jalr_target1;
        end else if (branch_taken1) begin
            pc_next = branch_target1;
        // If no control flow, check for superscalar/stall
        end else begin
            if (issue_inst2) begin
                pc_next = pc + 8;
            end else begin
                // This stall happens if we only issue inst1
                pc_next = pc + 4;
                fetch_stall = 1'b1;
            end
        end
    end
    
    
    always @(posedge clk) begin
        if (reset) begin
            pc_reg <= 32'd0;
            fd_pc <= 32'd0; // Reset pipelined PC
            fd_inst1 <= 32'd0;
            fd_inst2 <= 32'd0;
            fd_valid <= 1'b0;
        end else begin
            // Update PC to the (potentially new) target
            pc_reg <= pc_next;
            
            if (!fd_valid) begin
                // Initial fetch
                fd_pc <= pc; // Latch PC for inst1
                fd_inst1 <= inst1_f;
                fd_inst2 <= inst2_f;
                fd_valid <= 1'b1;
            end else if (control_flow_taken) begin
                // Pipeline flush: insert NOPs after control flow change
                fd_pc <= pc_next; // NOPs are at the new target PC
                fd_inst1 <= 32'h00000013;  // NOP (addi x0, x0, 0)
                fd_inst2 <= 32'h00000013;  // NOP
                fd_valid <= 1'b1; // Keep pipeline valid, but with NOPs
            end else if (fetch_stall) begin
                // Stall: slide inst2 to inst1
                // inst2_f is fetched from pc + 4 (which is pc_next)
                fd_pc <= pc_inst2; // The new fd_inst1 was at fd_pc+4 (pc_inst2)
                fd_inst1 <= fd_inst2;
                fd_inst2 <= inst2_f; // Fetch new inst2 from pc_next
                fd_valid <= 1'b1;
            end else begin
                // Normal fetch (issue_inst2 was true)
                // inst1_f/inst2_f are fetched from pc and pc+4 (pc_next-8 and pc_next-4)
                fd_pc <= pc; // The new fd_inst1 is at current pc
                fd_inst1 <= inst1_f;
                fd_inst2 <= inst2_f;
                fd_valid <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Debug Output
    // =========================================================================
    // ... other debug signals ...
    
    always @(posedge clk) begin
        if (!reset && fd_valid) begin
            $display("t=%0t PC=%0d [%s|%s] issue2=%b", 
                    $time, fd_pc, inst1_name, inst2_name, issue_inst2);
        
            if (load_use_hazard_detected)
                $display("  >>> LOAD-USE STALL DETECTED <<<");
                
            if (control_flow_taken)
                $display("  >>> CONTROL FLOW: target=%0d <<<", pc_next);
                
            if (wb_we1)
                $display("  WB1: x%0d = 0x%h", wb_rd1, wb_wdata1);
            if (wb_we2)
                $display("  WB2: x%0d = 0x%h", wb_rd2, wb_wdata2);
        end
    end // End always @(posedge clk)

endmodule

*/