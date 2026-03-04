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
    wire waw_hazard = (rd1 != 5'd0) && (rd1 == rd2) && RegWrite1 && RegWrite2;issue_inst2

    wire is_mem1 = (opcode1 == `OP_LOAD) || (opcode1 == `OP_STORE);
    wire is_mem2 = (opcode2 == `OP_LOAD) || (opcode2 == `OP_STORE);
    wire is_control_flow1 = is_jal1 || is_jalr1 || (opcode1 == `OP_BRANCH);
    wire is_control_flow2 = is_jal2 || is_jalr2 || (opcode2 == `OP_BRANCH);
    wire blocks_superscalar = is_mem1 || is_mem2 || is_control_flow1 || is_control_flow2 || load_use_hazard_inst1_inst2;
    //wire issue_inst2 = ~raw_hazard && ~waw_hazard && ~blocks_superscalar && fd_valid && !pipeline_stall && !fft_busy;
    wire issue_inst2 = 0;
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


